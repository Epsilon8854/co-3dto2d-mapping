#!/usr/bin/env python3
"""Build a startup-only 2D occupancy map in the raw LiDAR frame.

No odometry, ground plane, or sensor-to-base transform is required.  The node
collects a small stationary window, keeps returns in a symmetric band around
LiDAR z=0, applies the normal body/range/rear filters, projects the points to XY,
and publishes one transient-local OccupancyGrid for planar inter-robot ICP.
"""

from __future__ import annotations

from collections import deque
import math
import time
from typing import Deque, List, Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2, PointField

from co_3dto2d_mapping.startup_lidar_slice import (
    LidarSliceConfig,
    SliceFilterStats,
    filter_lidar_slice,
    limit_xy_points,
    rasterize_occupancy,
)


class StartupLidarOccupancy(Node):
    def __init__(self) -> None:
        super().__init__("startup_lidar_occupancy")
        self.declare_parameter("input_cloud_topic", "/livox/lidar")
        self.declare_parameter("output_map_topic", "startup/lidar_slice_occupancy")
        self.declare_parameter("output_frame_id", "")
        self.declare_parameter("frame_count", 5)
        self.declare_parameter("startup_delay_sec", 3.0)
        self.declare_parameter("slice_center_z_m", 0.0)
        self.declare_parameter("slice_half_height_m", 0.40)
        self.declare_parameter("center_box_half_extent_m", 0.80)
        self.declare_parameter("range_min_m", 0.80)
        self.declare_parameter("range_max_m", 12.0)
        self.declare_parameter("rear_filter_enabled", False)
        self.declare_parameter("rear_filter_angle_deg", 120.0)
        self.declare_parameter("rear_filter_axis", "-x")
        self.declare_parameter("rear_filter_min_xy_range_m", 0.0)
        self.declare_parameter("grid_resolution_m", 0.05)
        self.declare_parameter("map_half_extent_m", 12.0)
        self.declare_parameter("occupied_threshold_points", 1)
        self.declare_parameter("min_occupied_cells", 100)
        self.declare_parameter("max_points_per_frame", 30000)
        self.declare_parameter("lock_after_first_map", True)
        self.declare_parameter("publish_period_sec", 1.0)

        value = lambda name: self.get_parameter(name).value
        self.input_cloud_topic = str(value("input_cloud_topic"))
        self.output_map_topic = str(value("output_map_topic"))
        self.output_frame_id = str(value("output_frame_id"))
        self.frame_count = max(1, int(value("frame_count")))
        self.startup_delay_sec = float(value("startup_delay_sec"))
        self.grid_resolution_m = float(value("grid_resolution_m"))
        self.map_half_extent_m = float(value("map_half_extent_m"))
        self.occupied_threshold_points = max(
            1, int(value("occupied_threshold_points"))
        )
        self.min_occupied_cells = max(1, int(value("min_occupied_cells")))
        self.max_points_per_frame = max(1, int(value("max_points_per_frame")))
        self.lock_after_first_map = bool(value("lock_after_first_map"))
        self.publish_period_sec = float(value("publish_period_sec"))
        self.filter_config = LidarSliceConfig(
            slice_center_z_m=float(value("slice_center_z_m")),
            slice_half_height_m=float(value("slice_half_height_m")),
            center_box_half_extent_m=float(value("center_box_half_extent_m")),
            range_min_m=float(value("range_min_m")),
            range_max_m=float(value("range_max_m")),
            rear_filter_enabled=bool(value("rear_filter_enabled")),
            rear_filter_angle_deg=float(value("rear_filter_angle_deg")),
            rear_filter_axis=str(value("rear_filter_axis")),
            rear_filter_min_xy_range_m=float(
                value("rear_filter_min_xy_range_m")
            ),
        )
        self._validate_parameters()

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            OccupancyGrid,
            self.output_map_topic,
            map_qos,
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_cloud_topic,
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(
            self.publish_period_sec,
            self._republish_map,
        )

        self.frames: Deque[np.ndarray] = deque(maxlen=self.frame_count)
        self.latest_map: Optional[OccupancyGrid] = None
        self.first_input_monotonic: Optional[float] = None
        self.delay_complete_logged = False
        self.last_input_frame_id = ""
        self.latest_stamp = None
        self.build_attempts = 0

        lower = self.filter_config.slice_center_z_m - self.filter_config.slice_half_height_m
        upper = self.filter_config.slice_center_z_m + self.filter_config.slice_half_height_m
        self.get_logger().info(
            "Startup LiDAR occupancy: cloud=%s map=%s frames=%d delay=%.2fs "
            "raw_lidar_z=[%.2f, %.2f]m symmetric=true body_box=%.2fm "
            "range=[%.2f, %.2f]m resolution=%.3fm half_extent=%.2fm rear_filter=%s"
            % (
                self.input_cloud_topic,
                self.output_map_topic,
                self.frame_count,
                self.startup_delay_sec,
                lower,
                upper,
                self.filter_config.center_box_half_extent_m,
                self.filter_config.range_min_m,
                self.filter_config.range_max_m,
                self.grid_resolution_m,
                self.map_half_extent_m,
                "true" if self.filter_config.rear_filter_enabled else "false",
            )
        )
        self.get_logger().info(
            "The startup slice is evaluated before TF in the LiDAR frame; "
            "positive and negative z are both retained within the same 0-centered band."
        )

    def _validate_parameters(self) -> None:
        self.filter_config.validate()
        if not self.input_cloud_topic:
            raise ValueError("input_cloud_topic must not be empty")
        if not self.output_map_topic:
            raise ValueError("output_map_topic must not be empty")
        if not math.isfinite(self.startup_delay_sec) or self.startup_delay_sec < 0.0:
            raise ValueError("startup_delay_sec must be non-negative and finite")
        if not math.isfinite(self.grid_resolution_m) or self.grid_resolution_m <= 0.0:
            raise ValueError("grid_resolution_m must be positive and finite")
        if not math.isfinite(self.map_half_extent_m) or self.map_half_extent_m <= 0.0:
            raise ValueError("map_half_extent_m must be positive and finite")
        if (
            self.filter_config.range_max_m > 0.0
            and self.map_half_extent_m + 1e-9 < self.filter_config.range_max_m
        ):
            raise ValueError(
                "map_half_extent_m must cover range_max_m so accepted points are not clipped"
            )
        if not math.isfinite(self.publish_period_sec) or self.publish_period_sec <= 0.0:
            raise ValueError("publish_period_sec must be positive and finite")

    @staticmethod
    def _field_dtype(msg: PointCloud2, name: str) -> Tuple[int, np.dtype]:
        field = next((field for field in msg.fields if field.name == name), None)
        if field is None:
            raise ValueError("PointCloud2 is missing field '%s'" % name)
        order = ">" if msg.is_bigendian else "<"
        if field.datatype == PointField.FLOAT32:
            return int(field.offset), np.dtype(order + "f4")
        if field.datatype == PointField.FLOAT64:
            return int(field.offset), np.dtype(order + "f8")
        raise ValueError("PointCloud2 field '%s' must be FLOAT32 or FLOAT64" % name)

    @classmethod
    def _read_cloud_field(cls, msg: PointCloud2, name: str) -> np.ndarray:
        offset, dtype = cls._field_dtype(msg, name)
        width = int(msg.width)
        height = int(msg.height)
        point_step = int(msg.point_step)
        row_step = int(msg.row_step) or width * point_step
        if width <= 0 or height <= 0 or point_step <= 0:
            return np.empty(0, dtype=np.float64)

        data = bytes(msg.data)
        rows: List[np.ndarray] = []
        for row in range(height):
            row_start = row * row_step
            row_end = min(len(data), row_start + row_step)
            start = row_start + offset
            if start + dtype.itemsize > row_end:
                break
            available = 1 + (row_end - start - dtype.itemsize) // point_step
            count = min(width, max(0, available))
            if count:
                values = np.ndarray(
                    (count,),
                    dtype=dtype,
                    buffer=data,
                    offset=start,
                    strides=(point_step,),
                )
                rows.append(np.asarray(values, dtype=np.float64))
        return np.concatenate(rows) if rows else np.empty(0, dtype=np.float64)

    @classmethod
    def _cloud_xyz(cls, msg: PointCloud2) -> np.ndarray:
        fields = [cls._read_cloud_field(msg, name) for name in ("x", "y", "z")]
        count = min(map(len, fields), default=0)
        if count == 0:
            return np.empty((0, 3), dtype=np.float64)
        return np.column_stack([field[:count] for field in fields])

    def _startup_delay_elapsed(self) -> bool:
        now = time.monotonic()
        if self.first_input_monotonic is None:
            self.first_input_monotonic = now
            if self.startup_delay_sec > 0.0:
                self.get_logger().info(
                    "First startup cloud received; ignoring %.2fs before map collection."
                    % self.startup_delay_sec
                )
        if now - self.first_input_monotonic < self.startup_delay_sec:
            return False
        if not self.delay_complete_logged:
            self.delay_complete_logged = True
            self.get_logger().info(
                "Startup LiDAR settle delay complete; collecting 2D map frames."
            )
        return True

    def _cloud_callback(self, msg: PointCloud2) -> None:
        if self.latest_map is not None and self.lock_after_first_map:
            return
        self.last_input_frame_id = msg.header.frame_id
        self.latest_stamp = msg.header.stamp
        if not self._startup_delay_elapsed():
            return

        try:
            xyz = self._cloud_xyz(msg)
            xy, stats = filter_lidar_slice(xyz, self.filter_config)
        except ValueError as exc:
            self.get_logger().warn("Startup LiDAR map skipped: %s" % exc)
            return
        xy = limit_xy_points(xy, self.max_points_per_frame)
        self.frames.append(xy)
        self._log_frame(stats, len(xy))
        if len(self.frames) < self.frame_count:
            return
        self._try_build_map()

    def _log_frame(self, stats: SliceFilterStats, stored_points: int) -> None:
        self.get_logger().info(
            "Startup 2D slice frame %d/%d: raw=%d finite=%d kept=%d stored=%d "
            "below/above=%d/%d rejected(z/range/body/rear)=%d/%d/%d/%d"
            % (
                len(self.frames),
                self.frame_count,
                stats.input_points,
                stats.finite_points,
                stats.kept_points,
                stored_points,
                stats.kept_below_center,
                stats.kept_at_or_above_center,
                stats.rejected_z,
                stats.rejected_range,
                stats.rejected_center,
                stats.rejected_rear,
            )
        )

    def _try_build_map(self) -> None:
        self.build_attempts += 1
        non_empty = [frame for frame in self.frames if len(frame)]
        combined = (
            np.vstack(non_empty)
            if non_empty
            else np.empty((0, 2), dtype=np.float64)
        )
        raster = rasterize_occupancy(
            combined,
            resolution_m=self.grid_resolution_m,
            half_extent_m=self.map_half_extent_m,
            occupied_threshold_points=self.occupied_threshold_points,
        )
        if raster.occupied_cells < self.min_occupied_cells:
            self.get_logger().warn(
                "Startup 2D map attempt %d has only %d occupied cells (need %d); "
                "keeping the rolling frame window and retrying."
                % (
                    self.build_attempts,
                    raster.occupied_cells,
                    self.min_occupied_cells,
                )
            )
            return

        msg = OccupancyGrid()
        if self.latest_stamp is not None:
            msg.header.stamp = self.latest_stamp
            msg.info.map_load_time = self.latest_stamp
        msg.header.frame_id = (
            self.output_frame_id or self.last_input_frame_id or "livox_frame"
        )
        msg.info.resolution = float(raster.resolution_m)
        msg.info.width = int(raster.width)
        msg.info.height = int(raster.height)
        msg.info.origin.position.x = raster.origin_x_m
        msg.info.origin.position.y = raster.origin_y_m
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = raster.data.astype(np.int8, copy=False).tolist()
        self.latest_map = msg
        self.publisher.publish(msg)
        self.get_logger().info(
            "Published startup LiDAR-frame 2D map on %s: frames=%d points=%d "
            "occupied_cells=%d clipped=%d size=%dx%d resolution=%.3fm."
            % (
                self.output_map_topic,
                len(self.frames),
                len(combined),
                raster.occupied_cells,
                raster.out_of_bounds_points,
                raster.width,
                raster.height,
                raster.resolution_m,
            )
        )

    def _republish_map(self) -> None:
        if self.latest_map is None:
            return
        self.publisher.publish(self.latest_map)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StartupLidarOccupancy()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
