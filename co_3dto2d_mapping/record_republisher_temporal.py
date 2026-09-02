#!/usr/bin/env python3
"""Record republisher with temporal fusion driven by local observation grids.

The original merged map rebuilt an occupied-wins union of the two persistent
per-robot global maps.  That makes an occupied cell from a robot that no longer
observes the area impossible to clear.  This wrapper keeps the existing
republishing/TF behavior, seeds from each global map once, and then updates the
merged map only from new current-frame local occupancy observations.
"""

import math
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid

from co_3dto2d_mapping.record_republisher import ToyRecordRepublisher
from co_3dto2d_mapping.temporal_fusion import (
    FusionUpdateStats,
    TemporalFusionConfig,
    TemporalFusionGrid,
    expand_free_observations,
)


StampKey = Tuple[int, int, int, int]
PlanarAlignment = Optional[Tuple[float, float, float]]


def occupancy_stamp_key(msg: OccupancyGrid) -> StampKey:
    return (
        int(msg.header.stamp.sec),
        int(msg.header.stamp.nanosec),
        int(msg.info.width),
        int(msg.info.height),
    )


def quantized_grid_observations(
    msg: OccupancyGrid,
    output_resolution: float,
    occupied_threshold: int,
    alignment: PlanarAlignment = None,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Transform known grid cells and quantize them in a common lattice."""

    width = int(msg.info.width)
    height = int(msg.info.height)
    input_resolution = float(msg.info.resolution)
    output_resolution = float(output_resolution)
    if (
        width <= 0
        or height <= 0
        or input_resolution <= 0.0
        or output_resolution <= 0.0
        or len(msg.data) != width * height
    ):
        empty = np.empty(0, dtype=np.int64)
        return empty, empty.copy(), np.empty(0, dtype=np.int8)

    data = np.asarray(msg.data, dtype=np.int16).reshape((height, width))
    rows, cols = np.nonzero(data >= 0)
    if rows.size == 0:
        empty = np.empty(0, dtype=np.int64)
        return empty, empty.copy(), np.empty(0, dtype=np.int8)

    local_x = (cols.astype(np.float64) + 0.5) * input_resolution
    local_y = (rows.astype(np.float64) + 0.5) * input_resolution
    origin = msg.info.origin
    yaw = math.atan2(
        2.0
        * (
            origin.orientation.w * origin.orientation.z
            + origin.orientation.x * origin.orientation.y
        ),
        1.0
        - 2.0
        * (
            origin.orientation.y * origin.orientation.y
            + origin.orientation.z * origin.orientation.z
        ),
    )
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    world_x = (
        float(origin.position.x) + cos_yaw * local_x - sin_yaw * local_y
    )
    world_y = (
        float(origin.position.y) + sin_yaw * local_x + cos_yaw * local_y
    )

    if alignment is not None:
        align_x, align_y, align_yaw = alignment
        align_cos = math.cos(align_yaw)
        align_sin = math.sin(align_yaw)
        source_x = world_x
        source_y = world_y
        world_x = align_x + align_cos * source_x - align_sin * source_y
        world_y = align_y + align_sin * source_x + align_cos * source_y

    output_cols = np.floor(world_x / output_resolution).astype(np.int64)
    output_rows = np.floor(world_y / output_resolution).astype(np.int64)
    output_values = np.where(
        data[rows, cols] > int(occupied_threshold), 100, 0
    ).astype(np.int8)
    return output_cols, output_rows, output_values


class TemporalToyRecordRepublisher(ToyRecordRepublisher):
    def __init__(self) -> None:
        self.temporal_fusion: Optional[TemporalFusionGrid] = None
        self.last_local_stamps: Dict[int, StampKey] = {}
        self.seeded_robots = set()
        self.live_robots = set()
        self.temporal_cleared_total = 0
        self.last_temporal_log_ns = 0
        super().__init__()

        self.declare_parameter("merged_temporal_filter_enabled", True)
        self.declare_parameter("merged_dynamic_free_clear_count", 4)
        self.declare_parameter("merged_dynamic_occupied_confirm_count", 3)
        self.declare_parameter("merged_dynamic_counter_decay", 1)
        self.declare_parameter("merged_dynamic_evidence_timeout_frames", 30)
        self.declare_parameter("merged_free_observation_inflation_m", 0.05)
        self.declare_parameter("merged_alignment_reset_translation_m", 0.05)
        self.declare_parameter("merged_alignment_reset_yaw_deg", 0.5)

        self.merged_temporal_filter_enabled = bool(
            self.get_parameter("merged_temporal_filter_enabled").value
        )
        self.temporal_fusion_config = TemporalFusionConfig(
            free_clear_count=int(
                self.get_parameter("merged_dynamic_free_clear_count").value
            ),
            occupied_confirm_count=int(
                self.get_parameter("merged_dynamic_occupied_confirm_count").value
            ),
            counter_decay=int(
                self.get_parameter("merged_dynamic_counter_decay").value
            ),
            evidence_timeout_frames=int(
                self.get_parameter(
                    "merged_dynamic_evidence_timeout_frames"
                ).value
            ),
        ).validated()
        self.free_observation_inflation_m = max(
            0.0,
            float(
                self.get_parameter("merged_free_observation_inflation_m").value
            ),
        )
        self.alignment_reset_translation_m = max(
            0.0,
            float(
                self.get_parameter("merged_alignment_reset_translation_m").value
            ),
        )
        self.alignment_reset_yaw_rad = math.radians(
            max(
                0.0,
                float(self.get_parameter("merged_alignment_reset_yaw_deg").value),
            )
        )

        self.get_logger().info(
            "Temporal merged occupancy enabled=%s source=local_observation "
            "free_clear=%d occupied_confirm=%d free_inflation=%.3fm. "
            "The persistent robot global "
            "maps are bootstrap-only, so stale occupied cells cannot be "
            "reinserted every publish cycle."
            % (
                "true" if self.merged_temporal_filter_enabled else "false",
                self.temporal_fusion_config.free_clear_count,
                self.temporal_fusion_config.occupied_confirm_count,
                self.free_observation_inflation_m,
            )
        )

    def _alignment_for_robot(self, robot_id: int) -> PlanarAlignment:
        if robot_id == 0:
            return None
        return self.alignment

    def _ensure_temporal_fusion(self, msg: OccupancyGrid) -> bool:
        resolution = float(msg.info.resolution)
        if resolution <= 0.0:
            return False
        if self.temporal_fusion is None:
            self.temporal_fusion = TemporalFusionGrid(
                resolution=resolution,
                config=self.temporal_fusion_config,
                padding_m=self.merged_padding_m,
            )
            return True
        if not math.isclose(
            resolution,
            self.temporal_fusion.resolution,
            rel_tol=1e-6,
            abs_tol=1e-9,
        ):
            self.get_logger().warning(
                "Skipping merged observation with resolution %.6f; temporal "
                "fusion uses %.6f"
                % (resolution, self.temporal_fusion.resolution)
            )
            return False
        return True

    def _seed_global_map(self, msg: OccupancyGrid, robot_id: int) -> bool:
        if robot_id in self.seeded_robots or robot_id in self.live_robots:
            return False
        alignment = self._alignment_for_robot(robot_id)
        if robot_id != 0 and alignment is None:
            return False
        if not self._ensure_temporal_fusion(msg):
            return False
        cols, rows, values = quantized_grid_observations(
            msg,
            self.temporal_fusion.resolution,
            self.occupied_threshold,
            alignment,
        )
        self.temporal_fusion.seed(cols, rows, values)
        self.seeded_robots.add(robot_id)
        self.get_logger().info(
            "Seeded temporal merged occupancy from r%d global map: %d known cells"
            % (robot_id, int(values.size))
        )
        return True

    def _ingest_local_map(
        self, msg: OccupancyGrid, robot_id: int
    ) -> Optional[FusionUpdateStats]:
        alignment = self._alignment_for_robot(robot_id)
        if robot_id != 0 and alignment is None:
            return None
        stamp_key = occupancy_stamp_key(msg)
        if self.last_local_stamps.get(robot_id) == stamp_key:
            return None
        if not self._ensure_temporal_fusion(msg):
            return None

        # A global map may be used once as a startup bootstrap.  It is never
        # processed again after live local observations start.
        if robot_id not in self.seeded_robots and robot_id not in self.live_robots:
            global_map = self.latest_global_maps.get(robot_id)
            if global_map is not None:
                self._seed_global_map(global_map, robot_id)

        cols, rows, values = quantized_grid_observations(
            msg,
            self.temporal_fusion.resolution,
            self.occupied_threshold,
            alignment,
        )
        inflation_cells = int(
            round(
                self.free_observation_inflation_m
                / self.temporal_fusion.resolution
            )
        )
        cols, rows, values = expand_free_observations(
            cols, rows, values, inflation_cells
        )
        stats = self.temporal_fusion.observe(cols, rows, values)
        self.last_local_stamps[robot_id] = stamp_key
        self.live_robots.add(robot_id)
        self._log_temporal_update(robot_id, stats)
        return stats

    def _log_temporal_update(
        self, robot_id: int, stats: FusionUpdateStats
    ) -> None:
        self.temporal_cleared_total += stats.cleared_cells
        now_ns = int(self.get_clock().now().nanoseconds)
        periodic = now_ns - self.last_temporal_log_ns >= 5_000_000_000
        if not periodic and stats.cleared_cells == 0:
            return
        self.last_temporal_log_ns = now_ns
        self.get_logger().info(
            "Temporal merged occupancy update r%d: observed=%d free=%d "
            "occupied=%d cleared=%d (%d total) committed_occupied=%d"
            % (
                robot_id,
                stats.observed_cells,
                stats.free_observed_cells,
                stats.occupied_observed_cells,
                stats.cleared_cells,
                self.temporal_cleared_total,
                stats.committed_occupied_cells,
            )
        )

    def _reset_temporal_fusion(self, reason: str) -> None:
        if self.temporal_fusion is not None:
            self.get_logger().warning(
                "Resetting temporal merged occupancy: %s" % reason
            )
        self.temporal_fusion = None
        self.last_local_stamps.clear()
        self.seeded_robots.clear()
        self.live_robots.clear()

    def _alignment_changed_significantly(
        self,
        previous: Tuple[float, float, float],
        current: Tuple[float, float, float],
    ) -> bool:
        translation = math.hypot(
            current[0] - previous[0], current[1] - previous[1]
        )
        yaw = abs(
            math.atan2(
                math.sin(current[2] - previous[2]),
                math.cos(current[2] - previous[2]),
            )
        )
        return (
            translation > self.alignment_reset_translation_m
            or yaw > self.alignment_reset_yaw_rad
        )

    def alignment_callback(self, msg) -> None:
        previous = self.alignment
        super().alignment_callback(msg)
        if not self.merged_temporal_filter_enabled:
            return
        if previous is not None and self._alignment_changed_significantly(
            previous, self.alignment
        ):
            self._reset_temporal_fusion("inter-robot alignment changed")

        # Process data that arrived before the first alignment message, or
        # rebuild after a significant alignment correction.
        for robot_id, global_map in sorted(self.latest_global_maps.items()):
            self._seed_global_map(global_map, robot_id)
        for robot_id, local_map in sorted(self.latest_local_maps.items()):
            self._ingest_local_map(local_map, robot_id)

    def global_map_callback(self, msg: OccupancyGrid, robot_id: int) -> None:
        super().global_map_callback(msg, robot_id)
        if self.merged_temporal_filter_enabled:
            self._seed_global_map(msg, robot_id)

    def local_map_callback(self, msg: OccupancyGrid, robot_id: int) -> None:
        super().local_map_callback(msg, robot_id)
        if self.merged_temporal_filter_enabled:
            self._ingest_local_map(msg, robot_id)

    def build_merged_global(self, stamp) -> Optional[OccupancyGrid]:
        if (
            not self.merged_temporal_filter_enabled
            or self.temporal_fusion is None
            or not self.temporal_fusion.ready
        ):
            return super().build_merged_global(stamp)

        origin_col, origin_row, data = self.temporal_fusion.dense_snapshot()
        output = OccupancyGrid()
        output.header.stamp = stamp
        output.header.frame_id = self.common_frame_id
        output.info.resolution = float(self.temporal_fusion.resolution)
        output.info.width = int(data.shape[1])
        output.info.height = int(data.shape[0])
        output.info.origin.position.x = (
            float(origin_col) * self.temporal_fusion.resolution
        )
        output.info.origin.position.y = (
            float(origin_row) * self.temporal_fusion.resolution
        )
        output.info.origin.position.z = 0.0
        output.info.origin.orientation.w = 1.0
        output.data = data.reshape(-1).tolist()
        return output


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TemporalToyRecordRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
