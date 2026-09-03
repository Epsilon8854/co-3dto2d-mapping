#!/usr/bin/env python3
"""Frame-correct temporal compositor with r0/r1 single-robot fallback."""
from __future__ import annotations

from copy import deepcopy
import json
import math
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from co_3dto2d_mapping.fusion_frame_utils import (
    reference_transform_for_robot, seed_unknown_observations,
)
from co_3dto2d_mapping.record_republisher import (
    ToyRecordRepublisher, apply_planar_alignment, known_cell_centers,
    merge_global_grids,
)
from co_3dto2d_mapping.record_republisher_ground_fused import (
    GroundFusedTemporalRecordRepublisher,
)
from co_3dto2d_mapping.record_republisher_temporal import (
    occupancy_stamp_key, quantized_grid_observations,
)
from co_3dto2d_mapping.runtime_stability import choose_fusion_reference
from co_3dto2d_mapping.temporal_fusion import expand_free_observations

PlanarAlignment = Optional[Tuple[float, float, float]]


class RobustGroundFusedRecordRepublisher(GroundFusedTemporalRecordRepublisher):
    def __init__(self) -> None:
        self.active_reference_robot: Optional[int] = None
        self.map_receipt_ns: Dict[int, int] = {}
        self.odom_receipt_ns: Dict[int, int] = {}
        self.first_usable_ns: Dict[int, int] = {}
        self.last_global_seed_stamps = {}
        self.global_seed_added_cells: Dict[int, int] = {}
        self.last_fusion_status_ns = self.last_fusion_viz_ns = 0
        super().__init__()
        parameters = {
            "auto_single_robot_fallback": True,
            "single_robot_fallback_timeout_sec": 2.0,
            "fusion_source_timeout_sec": 5.0,
            "merged_min_global_seed_cells": 100,
            "fusion_status_topic": "/toy_record/fusion_status",
            "fusion_marker_topic": "/toy_record/fusion_markers",
            "fusion_status_period_sec": 1.0,
            "fusion_visualization_period_sec": 1.0,
            "fusion_visualization_max_occupied_points": 5000,
            "publish_common_odometry": True,
            "common_odometry_topic_format": "/toy_record/r{robot_id}/odom_in_map",
        }
        for name, value in parameters.items():
            self.declare_parameter(name, value)
        self.auto_single_robot_fallback = bool(self.get_parameter("auto_single_robot_fallback").value)
        self.single_robot_fallback_timeout_sec = max(0.0, float(self.get_parameter("single_robot_fallback_timeout_sec").value))
        self.fusion_source_timeout_sec = max(0.0, float(self.get_parameter("fusion_source_timeout_sec").value))
        self.merged_min_global_seed_cells = max(1, int(self.get_parameter("merged_min_global_seed_cells").value))
        self.fusion_status_period_sec = max(0.1, float(self.get_parameter("fusion_status_period_sec").value))
        self.fusion_visualization_period_sec = max(0.1, float(self.get_parameter("fusion_visualization_period_sec").value))
        self.fusion_visualization_max_occupied_points = max(100, int(self.get_parameter("fusion_visualization_max_occupied_points").value))
        self.publish_common_odometry = bool(self.get_parameter("publish_common_odometry").value)
        topic_format = str(self.get_parameter("common_odometry_topic_format").value)
        self.fusion_status_pub = self.create_publisher(String, str(self.get_parameter("fusion_status_topic").value), 10)
        self.fusion_marker_pub = self.create_publisher(MarkerArray, str(self.get_parameter("fusion_marker_topic").value), 10)
        self.common_odom_pubs = {
            rid: self.create_publisher(Odometry, topic_format.format(robot_id=rid), 10)
            for rid in self.robot_ids
        } if self.publish_common_odometry else {}

    def _now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _fresh(self, receipt_ns: int, now_ns: int) -> bool:
        return receipt_ns > 0 and (
            self.fusion_source_timeout_sec == 0.0
            or now_ns - receipt_ns <= int(self.fusion_source_timeout_sec * 1e9)
        )

    def _usable(self, rid: int, now_ns: int) -> bool:
        usable = (
            (rid in self.latest_global_maps or rid in self.latest_local_maps)
            and rid in self.latest_odom
            and self._fresh(self.map_receipt_ns.get(rid, 0), now_ns)
            and self._fresh(self.odom_receipt_ns.get(rid, 0), now_ns)
        )
        if usable and rid not in self.first_usable_ns:
            self.first_usable_ns[rid] = now_ns
        return usable

    def _r1_fallback_ready(self, now_ns: int, usable: bool) -> bool:
        return bool(
            usable and self.auto_single_robot_fallback
            and now_ns - self.first_usable_ns.get(1, now_ns)
            >= int(self.single_robot_fallback_timeout_sec * 1e9)
        )

    def _refresh_reference(self, now_ns: Optional[int] = None) -> None:
        now_ns = self._now_ns() if now_ns is None else now_ns
        r0, r1 = self._usable(0, now_ns), self._usable(1, now_ns)
        selected = choose_fusion_reference(
            self.alignment is not None, r0, self._r1_fallback_ready(now_ns, r1),
            self.active_reference_robot, self.auto_single_robot_fallback,
        )
        if selected == self.active_reference_robot:
            return
        previous, self.active_reference_robot = self.active_reference_robot, selected
        self._reset_temporal_fusion("common reference %s -> %s" % (previous, selected))
        self.last_global_seed_stamps.clear(); self.global_seed_added_cells.clear()
        if selected is not None:
            self.get_logger().warning(
                "Common map reference is now r%d/odom%s" % (
                    selected, " (single-robot fallback)" if selected == 1 and self.alignment is None else ""
                )
            )
        self._rebuild_latest()

    def _transform_for_robot(self, rid: int):
        return reference_transform_for_robot(self.active_reference_robot, self.alignment, rid)

    def _rebuild_latest(self) -> None:
        if self.active_reference_robot is None:
            return
        for rid, msg in sorted(self.latest_global_maps.items()):
            self._seed_unknown(msg, rid, True)
        for rid, msg in sorted(self.latest_local_maps.items()):
            self._ingest_local(msg, rid, True)

    def _seed_unknown(self, msg: OccupancyGrid, rid: int, force: bool = False) -> int:
        available, transform = self._transform_for_robot(rid)
        key = occupancy_stamp_key(msg)
        if not available or (not force and self.last_global_seed_stamps.get(rid) == key) or not self._ensure_temporal_fusion(msg):
            return 0
        cols, rows, values = quantized_grid_observations(
            msg, self.temporal_fusion.resolution, self.occupied_threshold, transform
        )
        if values.size < self.merged_min_global_seed_cells:
            return 0
        added = seed_unknown_observations(self.temporal_fusion, cols, rows, values)
        self.last_global_seed_stamps[rid] = key
        self.global_seed_added_cells[rid] = self.global_seed_added_cells.get(rid, 0) + added
        return added

    def _ingest_local(self, msg: OccupancyGrid, rid: int, force: bool = False):
        available, transform = self._transform_for_robot(rid)
        key = occupancy_stamp_key(msg)
        if not available or (not force and self.last_local_stamps.get(rid) == key) or not self._ensure_temporal_fusion(msg):
            return None
        if rid in self.latest_global_maps:
            self._seed_unknown(self.latest_global_maps[rid], rid)
        cols, rows, values = quantized_grid_observations(
            msg, self.temporal_fusion.resolution, self.occupied_threshold, transform
        )
        radius = int(round(self.free_observation_inflation_m / self.temporal_fusion.resolution))
        cols, rows, values = expand_free_observations(cols, rows, values, radius)
        stats = self.temporal_fusion.observe(cols, rows, values)
        self.last_local_stamps[rid] = key; self.live_robots.add(rid)
        self._log_temporal_update(rid, stats)
        return stats

    def alignment_callback(self, msg) -> None:
        previous = self.alignment
        ToyRecordRepublisher.alignment_callback(self, msg)
        if self.active_reference_robot != 0 or previous is None or self._alignment_changed_significantly(previous, self.alignment):
            self.active_reference_robot = 0
            self._reset_temporal_fusion("verified alignment selected canonical r0/map")
            self.last_global_seed_stamps.clear(); self.global_seed_added_cells.clear()
            self._rebuild_latest()

    def global_map_callback(self, msg: OccupancyGrid, rid: int) -> None:
        ToyRecordRepublisher.global_map_callback(self, msg, rid)
        now = self._now_ns(); self.map_receipt_ns[rid] = now
        self._refresh_reference(now); self._seed_unknown(msg, rid)

    def local_map_callback(self, msg: OccupancyGrid, rid: int) -> None:
        ToyRecordRepublisher.local_map_callback(self, msg, rid)
        now = self._now_ns(); self.map_receipt_ns[rid] = now
        self._refresh_reference(now); self._ingest_local(msg, rid)

    def odom_callback(self, msg: Odometry, rid: int) -> None:
        super().odom_callback(msg, rid)
        now = self._now_ns(); self.odom_receipt_ns[rid] = now; self._refresh_reference(now)

    def ground_fused_odom_callback(self, msg: Odometry, rid: int) -> None:
        super().ground_fused_odom_callback(msg, rid)
        now = self._now_ns(); self.odom_receipt_ns[rid] = now; self._refresh_reference(now)

    def build_merged_global(self, stamp) -> Optional[OccupancyGrid]:
        if self.merged_temporal_filter_enabled and self.temporal_fusion is not None and self.temporal_fusion.ready:
            origin_col, origin_row, data = self.temporal_fusion.dense_snapshot()
            output = OccupancyGrid(); output.header.stamp = stamp; output.header.frame_id = self.common_frame_id
            output.info.resolution = float(self.temporal_fusion.resolution)
            output.info.width, output.info.height = int(data.shape[1]), int(data.shape[0])
            output.info.origin.position.x = origin_col * self.temporal_fusion.resolution
            output.info.origin.position.y = origin_row * self.temporal_fusion.resolution
            output.info.origin.orientation.w = 1.0; output.data = data.reshape(-1).tolist()
            return output
        if self.active_reference_robot == 0:
            maps = {0: self.latest_global_maps[0]} if 0 in self.latest_global_maps else {}
            if self.alignment is not None and 1 in self.latest_global_maps:
                maps[1] = self.latest_global_maps[1]
            return merge_global_grids(maps, self.alignment, self.common_frame_id, self.merged_padding_m, self.occupied_threshold, stamp)
        if self.active_reference_robot == 1 and 1 in self.latest_global_maps:
            return merge_global_grids({0: self.latest_global_maps[1]}, None, self.common_frame_id, self.merged_padding_m, self.occupied_threshold, stamp)
        return None

    def publish_transforms(self, stamp) -> None:
        transforms = []
        if self.active_reference_robot is not None:
            transforms.append(self.identity_transform(stamp, self.common_frame_id, self.odom_frame(self.active_reference_robot)))
        if self.active_reference_robot == 0 and self.alignment_transform is not None:
            alignment = deepcopy(self.alignment_transform); alignment.header.stamp = stamp
            alignment.header.frame_id = self.common_frame_id; alignment.child_frame_id = self.odom_frame(1)
            transforms.append(alignment)
        transforms.extend(self.odom_to_base_transform(msg, stamp, rid) for rid, msg in self.latest_odom.items())
        for transform in transforms:
            self.tf_broadcaster.sendTransform(transform)

    def _common_odom(self, msg: Odometry, rid: int, stamp):
        available, transform = self._transform_for_robot(rid)
        if not available:
            return None
        output = self.copy_odom(msg, stamp, rid); output.header.frame_id = self.common_frame_id
        if transform is None:
            return output
        x, y, yaw = transform; pose = msg.pose.pose; c, s = math.cos(yaw), math.sin(yaw)
        output.pose.pose.position.x = x + c * pose.position.x - s * pose.position.y
        output.pose.pose.position.y = y + s * pose.position.x + c * pose.position.y
        output.pose.pose.orientation = self.multiply_quaternion(self.quaternion_from_yaw(yaw), pose.orientation)
        return output

    def _occupied_points(self, rid: int):
        msg = self.latest_global_maps.get(rid)
        available, transform = self._transform_for_robot(rid)
        if msg is None or not available:
            return []
        points = [(x, y) for x, y, value in known_cell_centers(msg, self.occupied_threshold) if value > 50]
        if transform is not None:
            points = [apply_planar_alignment(point, transform) for point in points]
        stride = max(1, len(points) // self.fusion_visualization_max_occupied_points)
        return points[::stride][:self.fusion_visualization_max_occupied_points]

    def _publish_diagnostics(self, stamp, now_ns: int) -> None:
        if now_ns - self.last_fusion_viz_ns >= int(self.fusion_visualization_period_sec * 1e9):
            self.last_fusion_viz_ns = now_ns; array = MarkerArray()
            clear = Marker(); clear.action = Marker.DELETEALL; array.markers.append(clear)
            for rid, color in ((0, (0.1, 0.55, 1.0)), (1, (1.0, 0.35, 0.05))):
                marker = Marker(); marker.header.stamp = stamp; marker.header.frame_id = self.common_frame_id
                marker.ns, marker.id, marker.type, marker.action = "map_contribution", rid, Marker.POINTS, Marker.ADD
                marker.scale.x = marker.scale.y = 0.07
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = (*color, 0.65)
                for x, y in self._occupied_points(rid):
                    point = Point(); point.x, point.y, point.z = x, y, 0.03 + 0.02 * rid; marker.points.append(point)
                array.markers.append(marker)
            text = Marker(); text.header.stamp = stamp; text.header.frame_id = self.common_frame_id
            text.ns, text.id, text.type, text.action = "fusion_status", 100, Marker.TEXT_VIEW_FACING, Marker.ADD
            text.pose.position.z, text.scale.z = 1.0, 0.35
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = "reference: %s\nalignment: %s\nseed-added: %s" % (
                "none" if self.active_reference_robot is None else f"r{self.active_reference_robot}",
                "ready" if self.alignment is not None else "waiting", self.global_seed_added_cells,
            )
            array.markers.append(text); self.fusion_marker_pub.publish(array)
        if now_ns - self.last_fusion_status_ns >= int(self.fusion_status_period_sec * 1e9):
            self.last_fusion_status_ns = now_ns; msg = String()
            msg.data = json.dumps({
                "active_reference_robot": self.active_reference_robot,
                "single_robot_fallback": self.active_reference_robot == 1 and self.alignment is None,
                "alignment_ready": self.alignment is not None,
                "maps_received": sorted(self.latest_global_maps),
                "local_maps_received": sorted(self.latest_local_maps),
                "odometry_received": sorted(self.latest_odom),
                "global_seed_added_cells": dict(sorted(self.global_seed_added_cells.items())),
                "temporal_fusion_ready": bool(self.temporal_fusion is not None and self.temporal_fusion.ready),
            }, sort_keys=True)
            self.fusion_status_pub.publish(msg)

    def publish_outputs(self) -> None:
        now = self._now_ns(); self._refresh_reference(now); super().publish_outputs()
        stamp = self.get_clock().now().to_msg()
        for rid, msg in self.latest_odom.items():
            output = self._common_odom(msg, rid, stamp)
            if output is not None and rid in self.common_odom_pubs:
                self.common_odom_pubs[rid].publish(output)
        self._publish_diagnostics(stamp, now)


def main(args=None) -> None:
    rclpy.init(args=args); node = RobustGroundFusedRecordRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
