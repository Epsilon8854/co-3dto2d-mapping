#!/usr/bin/env python3
"""Stationary-safe policy wrapper around the CPU-only 2-D place recognizer."""
from __future__ import annotations

from collections import deque
from dataclasses import replace
import math
import os
from typing import Deque, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

from co_3dto2d_mapping.inter_robot_place_alignment import (
    InterRobotPlaceAlignment, PlaceKeyframe, _grid_snapshot, _odom_pose, _stamp_ns,
)
from co_3dto2d_mapping.occupancy_place_recognition import (
    AlignmentMeasurement, Pose2, angular_distance, build_polar_descriptor,
    compose_pose, estimate_se2_consensus, extract_local_submap,
    map_alignment_from_keyframes, match_polar_descriptors, register_submaps,
    transform_points,
)
from co_3dto2d_mapping.runtime_stability import (
    RuntimePoseSample, motion_keyframe_allowed, stationarity_metrics,
)


def _env_float(name: str, default: float) -> float:
    value = os.environ.get(name)
    return default if value is None else float(value)


class StableInterRobotPlaceAlignment(InterRobotPlaceAlignment):
    """Lock only after repeated map updates agree at one frozen stationary pose."""

    def _declare_parameters(self) -> None:
        super()._declare_parameters()
        extra = {
            "robot0_motion_odom_topic": "/r0/odom",
            "robot1_motion_odom_topic": "/r1/odom",
            "initial_stationary_alignment_enabled": True,
            "initial_stationary_window_sec": 2.0,
            "initial_stationary_max_translation_m": 0.04,
            "initial_stationary_max_yaw_rad": math.radians(1.5),
            "initial_stationary_max_linear_speed_mps": 0.04,
            "initial_stationary_max_angular_speed_rps": math.radians(3.0),
            "initial_corrected_max_translation_m": 0.10,
            "initial_corrected_max_yaw_rad": math.radians(4.0),
            "initial_min_map_known_cells": 300,
            "initial_min_snapshot_interval_sec": 1.0,
            "initial_required_estimates": int(_env_float("CO3DTO2D_PLACE_INITIAL_ESTIMATES", 3)),
            "initial_consistency_translation_m": 0.20,
            "initial_consistency_yaw_rad": math.radians(3.0),
            "initial_yaw_search_range_rad": math.radians(12.0),
            "initial_max_yaw_deviation_rad": math.radians(
                _env_float("CO3DTO2D_PLACE_INITIAL_MAX_YAW_DEVIATION_DEG", 35.0)
            ),
            "initial_translation_search_range_m": _env_float(
                "CO3DTO2D_PLACE_INITIAL_SEARCH_TRANSLATION_M", 3.0
            ),
            "initial_max_translation_m": _env_float(
                "CO3DTO2D_PLACE_INITIAL_MAX_TRANSLATION_M", 3.0
            ),
            "stationary_keyframe_large_motion_factor": 1.75,
            "alignment_marker_topic": "/toy/place_alignment/markers",
        }
        for name, value in extra.items():
            self.declare_parameter(name, value)
        self._parameter_names = tuple(self._parameter_names) + tuple(extra)

    def __init__(self) -> None:
        self.motion_samples: List[Deque[RuntimePoseSample]] = [deque(maxlen=400), deque(maxlen=400)]
        self.corrected_samples: List[Deque[RuntimePoseSample]] = [deque(maxlen=400), deque(maxlen=400)]
        self.motion_stability = [None, None]
        self.corrected_stability = [None, None]
        self.startup_anchor_pose: List[Optional[Pose2]] = [None, None]
        self.startup_measurements: List[AlignmentMeasurement] = []
        self.startup_snapshot_index = 0
        self.last_startup_versions = (-1, -1)
        self.last_startup_attempt_ns = 0
        self.last_visual_pair: Optional[Tuple[PlaceKeyframe, PlaceKeyframe, Pose2, Pose2]] = None
        super().__init__()
        for rid in (0, 1):
            self.create_subscription(
                Odometry, str(self.p[f"robot{rid}_motion_odom_topic"]),
                lambda msg, robot=rid: self._motion_odom_callback(msg, robot), 50,
            )
        self.marker_pub = self.create_publisher(
            MarkerArray, str(self.p["alignment_marker_topic"]), 10
        )

    @staticmethod
    def _sample(msg: Odometry, receipt_ns: int) -> RuntimePoseSample:
        pose, twist = _odom_pose(msg), msg.twist.twist
        return RuntimePoseSample(
            receipt_ns, _stamp_ns(msg.header.stamp), pose.x, pose.y, pose.yaw,
            math.hypot(float(twist.linear.x), float(twist.linear.y)),
            abs(float(twist.angular.z)),
        )

    def _odom_callback(self, msg: Odometry, robot_id: int) -> None:
        super()._odom_callback(msg, robot_id)
        now = self.get_clock().now().nanoseconds
        self.corrected_samples[robot_id].append(self._sample(msg, now))

    def _motion_odom_callback(self, msg: Odometry, robot_id: int) -> None:
        now = self.get_clock().now().nanoseconds
        self.motion_samples[robot_id].append(self._sample(msg, now))

    def _update_stability(self, now_ns: int) -> None:
        window = float(self.p["initial_stationary_window_sec"])
        for rid in (0, 1):
            self.motion_stability[rid] = stationarity_metrics(
                self.motion_samples[rid], now_ns, window,
                float(self.p["initial_stationary_max_translation_m"]),
                float(self.p["initial_stationary_max_yaw_rad"]),
                float(self.p["initial_stationary_max_linear_speed_mps"]),
                float(self.p["initial_stationary_max_angular_speed_rps"]),
            )
            self.corrected_stability[rid] = stationarity_metrics(
                self.corrected_samples[rid], now_ns, window,
                float(self.p["initial_corrected_max_translation_m"]),
                float(self.p["initial_corrected_max_yaw_rad"]), 1e6, 1e6,
            )

    def _known_cells(self, rid: int) -> int:
        return 0 if rid not in self.maps else int(
            np.count_nonzero(np.asarray(self.maps[rid].data, dtype=np.int16) >= 0)
        )

    def _stationary_ready(self) -> bool:
        return all(
            self.motion_stability[rid] is not None
            and self.motion_stability[rid].stable
            and self.corrected_stability[rid] is not None
            and self.corrected_stability[rid].stable
            for rid in (0, 1)
        )

    def _reset_stationary_search(self) -> None:
        self.startup_anchor_pose = [None, None]
        self.startup_measurements.clear()
        self.last_startup_versions = (-1, -1)
        self.state = "WAITING_STATIONARY"

    def _freeze_anchors(self) -> None:
        for rid in (0, 1):
            metric = self.corrected_stability[rid]
            self.startup_anchor_pose[rid] = Pose2(metric.center_x, metric.center_y, metric.center_yaw)
        self.state = "STATIONARY_SEARCH"

    def _snapshot_keyframe(self, rid: int, snapshot_id: int) -> Optional[PlaceKeyframe]:
        grid, pose = _grid_snapshot(self.maps[rid]), self.startup_anchor_pose[rid]
        if grid is None or pose is None:
            return None
        submap = extract_local_submap(
            grid, pose, float(self.p["submap_radius_m"]),
            float(self.p["submap_resolution_m"]), int(self.p["occupied_threshold"]),
            int(self.p["max_boundary_points"]),
        )
        if submap.known_ratio < float(self.p["min_known_ratio"]) or len(
            submap.boundary_points
        ) < int(self.p["min_boundary_points"]):
            return None
        descriptor = build_polar_descriptor(
            submap, int(self.p["descriptor_num_rings"]),
            int(self.p["descriptor_num_sectors"]),
            int(self.p["descriptor_min_known_cells_per_bin"]),
        )
        return PlaceKeyframe(rid, snapshot_id, grid.stamp_ns, pose, submap, descriptor)

    def _register_snapshot(self, r0: PlaceKeyframe, r1: PlaceKeyframe):
        similarity, descriptor_yaw = match_polar_descriptors(r0.descriptor, r1.descriptor)
        expected_kf_yaw = (
            float(self.p["expected_yaw_rad"]) - r0.pose_in_robot_map.yaw + r1.pose_in_robot_map.yaw
        )
        hypotheses = [expected_kf_yaw]
        if angular_distance(descriptor_yaw, expected_kf_yaw) <= float(
            self.p["initial_max_yaw_deviation_rad"]
        ):
            hypotheses.append(descriptor_yaw)
        options = replace(
            self.registration,
            coarse_translation_range_m=float(self.p["initial_translation_search_range_m"]),
            coarse_yaw_range_rad=float(self.p["initial_yaw_search_range_rad"]),
        )
        candidates = []
        for hypothesis in hypotheses:
            result = register_submaps(r0.submap, r1.submap, hypothesis, options)
            if not result.success:
                continue
            alignment = map_alignment_from_keyframes(
                r0.pose_in_robot_map, result.transform_source_to_target, r1.pose_in_robot_map
            )
            if angular_distance(alignment.yaw, float(self.p["expected_yaw_rad"])) > float(
                self.p["initial_max_yaw_deviation_rad"]
            ) or math.hypot(alignment.x, alignment.y) > float(self.p["initial_max_translation_m"]):
                continue
            candidates.append((result.score, result, alignment))
        return (similarity, None, None) if not candidates else (
            similarity, *max(candidates, key=lambda item: item[0])[1:]
        )

    def _process_stationary_startup(self, now_ns: int) -> None:
        ready = all(rid in self.maps and self.odom[rid] and self.motion_samples[rid] for rid in (0, 1))
        if not ready:
            self.inputs_ready_since_ns, self.state = None, "WAITING_INPUTS"
            return
        if self.inputs_ready_since_ns is None:
            self.inputs_ready_since_ns, self.state = now_ns, "SETTLING"
            return
        if now_ns - self.inputs_ready_since_ns < int(float(self.p["startup_delay_sec"]) * 1e9):
            self.state = "SETTLING"
            return
        if any(self._known_cells(rid) < int(self.p["initial_min_map_known_cells"]) for rid in (0, 1)):
            self.state = "WAITING_MAP_GROWTH"
            return
        if not self._stationary_ready():
            self._reset_stationary_search()
            return
        if self.startup_anchor_pose[0] is None:
            self._freeze_anchors()
        versions = tuple(self.map_version)
        if versions == self.last_startup_versions or now_ns - self.last_startup_attempt_ns < int(
            float(self.p["initial_min_snapshot_interval_sec"]) * 1e9
        ):
            return
        self.last_startup_versions, self.last_startup_attempt_ns = versions, now_ns
        snapshot_id = self.startup_snapshot_index
        self.startup_snapshot_index += 1
        r0, r1 = self._snapshot_keyframe(0, snapshot_id), self._snapshot_keyframe(1, snapshot_id)
        if r0 is None or r1 is None:
            self.last_registration = "stationary snapshot lacks map support"
            return
        similarity, result, alignment = self._register_snapshot(r0, r1)
        if result is None:
            self.last_registration = "stationary snapshot rejected"
            return
        self.startup_measurements.append(AlignmentMeasurement(
            alignment, snapshot_id, snapshot_id, similarity, result.symmetric_overlap,
            result.symmetric_rmse_m, result.free_space_conflict_ratio,
            max(r0.stamp_ns, r1.stamp_ns),
        ))
        self.startup_measurements = self.startup_measurements[-9:]
        self.last_visual_pair = r0, r1, result.transform_source_to_target, alignment
        self.last_registration = "stationary estimate %d: %.2f,%.2f,%.1fdeg" % (
            snapshot_id, alignment.x, alignment.y, math.degrees(alignment.yaw)
        )
        self.state = "TENTATIVE"
        required = int(self.p["initial_required_estimates"])
        consensus = estimate_se2_consensus(
            self.startup_measurements,
            float(self.p["initial_consistency_translation_m"]),
            float(self.p["initial_consistency_yaw_rad"]), required, required,
        )
        if consensus is not None:
            self.measurements = list(consensus.measurements)
            self._accept_consensus(consensus, "stationary startup lock")

    def _motion_is_enough(self, robot_id: int, pose: Pose2) -> bool:
        if not self.keyframes[robot_id]:
            return True
        previous = self.keyframes[robot_id][-1].pose_in_robot_map
        return motion_keyframe_allowed(
            previous.x, previous.y, previous.yaw, pose.x, pose.y, pose.yaw,
            self.motion_stability[robot_id], float(self.p["keyframe_translation_m"]),
            float(self.p["keyframe_rotation_rad"]),
            float(self.p["stationary_keyframe_large_motion_factor"]),
        )

    def _publish_visualization(self) -> None:
        stamp, array = self.get_clock().now().to_msg(), MarkerArray()
        clear = Marker(); clear.action = Marker.DELETEALL; array.markers.append(clear)
        if self.last_visual_pair is not None:
            r0, r1, k0_from_k1, alignment = self.last_visual_pair
            point_sets = (
                ("r0_boundary", transform_points(r0.pose_in_robot_map, r0.submap.boundary_points), (0.1, 0.55, 1.0)),
                ("r1_registered", transform_points(compose_pose(r0.pose_in_robot_map, k0_from_k1), r1.submap.boundary_points), (1.0, 0.35, 0.05)),
            )
            for marker_id, (namespace, points, color) in enumerate(point_sets):
                marker = Marker(); marker.header.stamp = stamp
                marker.header.frame_id = str(self.p["target_frame_id"])
                marker.ns, marker.id, marker.type, marker.action = namespace, marker_id, Marker.POINTS, Marker.ADD
                marker.scale.x = marker.scale.y = 0.06
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = (*color, 0.9)
                for x, y in points[::max(1, len(points) // 2500)]:
                    point = Point(); point.x, point.y, point.z = float(x), float(y), 0.05
                    marker.points.append(point)
                array.markers.append(marker)
            arrow = Marker(); arrow.header.stamp = stamp
            arrow.header.frame_id = str(self.p["target_frame_id"])
            arrow.ns, arrow.id, arrow.type, arrow.action = "map1_to_map0", 10, Marker.ARROW, Marker.ADD
            start, end = Point(), Point(); end.x, end.y = alignment.x, alignment.y
            arrow.points = [start, end]; arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.08, 0.18, 0.18
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 1.0, 0.1, 0.9
            array.markers.append(arrow)
        text = Marker(); text.header.stamp = stamp
        text.header.frame_id = str(self.p["target_frame_id"])
        text.ns, text.id, text.type, text.action = "status", 100, Marker.TEXT_VIEW_FACING, Marker.ADD
        text.pose.position.z, text.scale.z = 1.2, 0.32
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = "%s\nestimates %d/%d\n%s" % (
            self.state, len(self.startup_measurements),
            int(self.p["initial_required_estimates"]), self.last_registration,
        )
        array.markers.append(text); self.marker_pub.publish(array)

    def _process(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        self._update_stability(now_ns)
        if self.alignment is None:
            self._process_stationary_startup(now_ns)
        else:
            self._make_keyframe(0); self._make_keyframe(1)
            self._verify_candidates(); self._update_consensus()
            self._publish_alignment(); self._publish_relative_transform()
        self._publish_status(now_ns); self._publish_visualization()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StableInterRobotPlaceAlignment()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
