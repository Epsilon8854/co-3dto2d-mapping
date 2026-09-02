#!/usr/bin/env python3
"""ROS 2 front-end for CPU-only, PGO-free two-robot map alignment.

The node keeps both local SLAM systems untouched.  It freezes 2-D occupancy
submaps at corrected-odometry keyframes, detects inter-robot places with a
Scan-Context-like polar descriptor, verifies candidates by correlative matching
and trimmed ICP, then estimates one robust ``T_map0_map1`` transform.  The
published alignment topic is intentionally identical to the previous startup
ICP node, so the existing TF and merged-map compositor continue to work.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import math
import os
from typing import Deque, Dict, List, Optional, Sequence, Set, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from co_3dto2d_mapping.occupancy_place_recognition import (
    AlignmentMeasurement,
    ConsensusResult,
    GridSnapshot,
    LocalSubmap,
    PolarDescriptor,
    Pose2,
    RegistrationOptions,
    angular_distance,
    build_polar_descriptor,
    compose_pose,
    estimate_se2_consensus,
    extract_local_submap,
    inverse_pose,
    map_alignment_from_keyframes,
    match_polar_descriptors,
    register_submaps,
)


@dataclass(frozen=True)
class TimedPose:
    stamp_ns: int
    pose: Pose2


@dataclass(frozen=True)
class PlaceKeyframe:
    robot_id: int
    keyframe_id: int
    stamp_ns: int
    pose_in_robot_map: Pose2
    submap: LocalSubmap
    descriptor: PolarDescriptor


@dataclass(frozen=True)
class PendingMatch:
    robot0: PlaceKeyframe
    robot1: PlaceKeyframe
    similarity: float
    yaw_robot1_to_robot0: float

    @property
    def pair(self) -> Tuple[int, int]:
        return self.robot0.keyframe_id, self.robot1.keyframe_id


def _env_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"{name} must be a boolean value")


def _env_float(name: str, default: float) -> float:
    value = os.environ.get(name)
    return default if value is None else float(value)


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _yaw(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _odom_pose(msg: Odometry) -> Pose2:
    pose = msg.pose.pose
    return Pose2(float(pose.position.x), float(pose.position.y), _yaw(pose.orientation))


def _grid_snapshot(msg: OccupancyGrid) -> Optional[GridSnapshot]:
    width, height = int(msg.info.width), int(msg.info.height)
    resolution = float(msg.info.resolution)
    if width <= 0 or height <= 0 or resolution <= 0.0:
        return None
    if len(msg.data) != width * height:
        return None
    origin = msg.info.origin
    return GridSnapshot(
        resolution=resolution,
        origin=Pose2(
            float(origin.position.x),
            float(origin.position.y),
            _yaw(origin.orientation),
        ),
        data=np.asarray(msg.data, dtype=np.int16).reshape((height, width)),
        frame_id=msg.header.frame_id,
        stamp_ns=_stamp_ns(msg.header.stamp),
    )


class InterRobotPlaceAlignment(Node):
    """Detect places and estimate a single robust inter-map SE(2) transform."""

    def __init__(self) -> None:
        super().__init__("inter_robot_place_alignment")
        self._declare_parameters()
        self.p = {name: self.get_parameter(name).value for name in self._parameter_names}
        self._validate_parameters()
        self.registration = RegistrationOptions(
            coarse_translation_range_m=float(self.p["registration_coarse_translation_range_m"]),
            coarse_translation_step_m=float(self.p["registration_coarse_translation_step_m"]),
            coarse_yaw_range_rad=float(self.p["registration_coarse_yaw_range_rad"]),
            coarse_yaw_step_rad=float(self.p["registration_coarse_yaw_step_rad"]),
            fine_translation_range_m=float(self.p["registration_fine_translation_range_m"]),
            fine_translation_step_m=float(self.p["registration_fine_translation_step_m"]),
            fine_yaw_range_rad=float(self.p["registration_fine_yaw_range_rad"]),
            fine_yaw_step_rad=float(self.p["registration_fine_yaw_step_rad"]),
            search_max_distance_m=float(self.p["registration_search_max_distance_m"]),
            search_max_points=int(self.p["registration_search_max_points"]),
            icp_max_iterations=int(self.p["registration_icp_max_iterations"]),
            icp_max_correspondence_m=float(self.p["registration_icp_max_correspondence_m"]),
            icp_trim_ratio=float(self.p["registration_icp_trim_ratio"]),
            min_correspondences=int(self.p["registration_min_correspondences"]),
            min_symmetric_overlap=float(self.p["registration_min_symmetric_overlap"]),
            max_symmetric_rmse_m=float(self.p["registration_max_symmetric_rmse_m"]),
            max_free_space_conflict_ratio=float(
                self.p["registration_max_free_space_conflict_ratio"]
            ),
            free_space_conflict_clearance_m=float(
                self.p["registration_free_space_conflict_clearance_m"]
            ),
        )
        self.registration.validate()

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.alignment_pub = self.create_publisher(
            TransformStamped, str(self.p["alignment_topic"]), latched
        )
        self.relative_pub = self.create_publisher(
            TransformStamped, str(self.p["relative_transform_topic"]), 10
        )
        self.status_pub = self.create_publisher(String, str(self.p["status_topic"]), 10)

        self.maps: Dict[int, OccupancyGrid] = {}
        self.map_version = [0, 0]
        self.used_map_version = [-1, -1]
        history_size = max(10, int(self.p["odom_history_size"]))
        self.odom: List[Deque[TimedPose]] = [
            deque(maxlen=history_size),
            deque(maxlen=history_size),
        ]
        self.keyframes: List[List[PlaceKeyframe]] = [[], []]
        self.next_keyframe_id = [0, 0]
        self.pending: List[PendingMatch] = []
        self.queued_pairs: Set[Tuple[int, int]] = set()
        self.attempted_pairs: Set[Tuple[int, int]] = set()
        self.measurements: List[AlignmentMeasurement] = []
        self.alignment: Optional[Pose2] = None
        self.consensus: Optional[ConsensusResult] = None
        self.state = "SEARCHING"
        self.inputs_ready_since_ns: Optional[int] = None
        self.lock_mismatch_count = 0
        self.last_status_ns = 0
        self.last_registration = "none"

        for robot_id in (0, 1):
            self.create_subscription(
                OccupancyGrid,
                str(self.p[f"robot{robot_id}_map_topic"]),
                lambda msg, rid=robot_id: self._map_callback(msg, rid),
                10,
            )
            self.create_subscription(
                Odometry,
                str(self.p[f"robot{robot_id}_odom_topic"]),
                lambda msg, rid=robot_id: self._odom_callback(msg, rid),
                50,
            )
        self.timer = self.create_timer(max(0.05, float(self.p["processing_period_sec"])), self._process)
        self.get_logger().info(
            "2D place alignment: submap=%.1fm@%.2fm descriptor=%dx%d, "
            "min_similarity=%.2f, consensus=%d supports/%d distinct keyframes"
            % (
                float(self.p["submap_radius_m"]),
                float(self.p["submap_resolution_m"]),
                int(self.p["descriptor_num_rings"]),
                int(self.p["descriptor_num_sectors"]),
                float(self.p["descriptor_min_similarity"]),
                int(self.p["required_consistent_results"]),
                int(self.p["consensus_min_distinct_keyframes_per_robot"]),
            )
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "robot0_map_topic": "/r0/toy/global_occupancy",
            "robot1_map_topic": "/r1/toy/global_occupancy",
            "robot0_odom_topic": "/r0/toy/corrected_odometry",
            "robot1_odom_topic": "/r1/toy/corrected_odometry",
            "alignment_topic": "/toy/initial_xy_alignment",
            "relative_transform_topic": "/toy/inter_robot_relative_transform",
            "status_topic": "/toy/inter_robot_alignment_status",
            "target_frame_id": "map",
            "source_frame_id": "r1/odom",
            "robot0_base_frame_id": "r0/base_link",
            "robot1_base_frame_id": "r1/base_link",
            "processing_period_sec": 0.50,
            "status_period_sec": 1.0,
            "startup_delay_sec": 3.0,
            "max_map_odom_delta_sec": 0.30,
            "max_relative_odom_delta_sec": 0.20,
            "odom_history_size": 200,
            "keyframe_translation_m": _env_float(
                "CO3DTO2D_PLACE_KEYFRAME_TRANSLATION_M", 1.0
            ),
            "keyframe_rotation_rad": math.radians(
                _env_float("CO3DTO2D_PLACE_KEYFRAME_ROTATION_DEG", 10.0)
            ),
            "max_keyframes_per_robot": 200,
            "submap_radius_m": _env_float("CO3DTO2D_PLACE_SUBMAP_RADIUS_M", 15.0),
            "submap_resolution_m": _env_float(
                "CO3DTO2D_PLACE_SUBMAP_RESOLUTION_M", 0.10
            ),
            "occupied_threshold": 50,
            "min_known_ratio": 0.12,
            "min_boundary_points": 120,
            "max_boundary_points": 3000,
            "descriptor_num_rings": 20,
            "descriptor_num_sectors": 60,
            "descriptor_min_known_cells_per_bin": 2,
            "descriptor_top_k": 5,
            "descriptor_candidate_multiplier": 3,
            "descriptor_min_similarity": _env_float(
                "CO3DTO2D_PLACE_MIN_DESCRIPTOR_SIMILARITY", 0.45
            ),
            "descriptor_min_similarity_margin": 0.0,
            "require_mutual_match": _env_bool(
                "CO3DTO2D_PLACE_REQUIRE_MUTUAL_MATCH", False
            ),
            "candidate_budget_per_cycle": 1,
            "registration_coarse_translation_range_m": 4.0,
            "registration_coarse_translation_step_m": 0.25,
            "registration_coarse_yaw_range_rad": math.radians(30.0),
            "registration_coarse_yaw_step_rad": math.radians(2.0),
            "registration_fine_translation_range_m": 0.50,
            "registration_fine_translation_step_m": 0.05,
            "registration_fine_yaw_range_rad": math.radians(2.0),
            "registration_fine_yaw_step_rad": math.radians(0.5),
            "registration_search_max_distance_m": 0.75,
            "registration_search_max_points": 600,
            "registration_icp_max_iterations": 30,
            "registration_icp_max_correspondence_m": 0.40,
            "registration_icp_trim_ratio": 0.75,
            "registration_min_correspondences": 60,
            "registration_min_symmetric_overlap": _env_float(
                "CO3DTO2D_PLACE_MIN_SYMMETRIC_OVERLAP", 0.35
            ),
            "registration_max_symmetric_rmse_m": _env_float(
                "CO3DTO2D_PLACE_MAX_SYMMETRIC_RMSE_M", 0.20
            ),
            "registration_max_free_space_conflict_ratio": 0.10,
            "registration_free_space_conflict_clearance_m": 0.20,
            # Existing two-live launch names are retained for compatibility.
            "required_consistent_results": int(
                _env_float("CO3DTO2D_PLACE_MIN_SUPPORTS", 2)
            ),
            "max_consistency_translation_m": 0.40,
            "max_consistency_rotation_rad": math.radians(4.0),
            "consensus_min_distinct_keyframes_per_robot": int(
                _env_float("CO3DTO2D_PLACE_MIN_DISTINCT_KEYFRAMES_PER_ROBOT", 2)
            ),
            "consensus_max_measurements": 50,
            "enforce_heading_prior": _env_bool("CO3DTO2D_ENFORCE_HEADING_PRIOR", True),
            "expected_yaw_rad": math.radians(
                _env_float("CO3DTO2D_EXPECTED_RELATIVE_YAW_DEG", 0.0)
            ),
            "max_yaw_deviation_rad": math.radians(
                _env_float("CO3DTO2D_MAX_RELATIVE_YAW_DEVIATION_DEG", 90.0)
            ),
            "lock_after_first_alignment": True,
            "allow_relock": False,
            "locked_monitor_translation_m": 0.60,
            "locked_monitor_yaw_rad": math.radians(6.0),
            "degraded_after_inconsistent": 3,
        }
        self._parameter_names = tuple(defaults)
        for name, default in defaults.items():
            self.declare_parameter(name, default)

    def _validate_parameters(self) -> None:
        ratios = (
            "min_known_ratio",
            "descriptor_min_similarity",
            "registration_min_symmetric_overlap",
            "registration_max_free_space_conflict_ratio",
        )
        for name in ratios:
            if not 0.0 <= float(self.p[name]) <= 1.0:
                raise ValueError(f"{name} must be in [0, 1]")
        if not 0.0 <= float(self.p["max_yaw_deviation_rad"]) <= math.pi:
            raise ValueError("max_yaw_deviation_rad must be in [0, pi]")
        if int(self.p["required_consistent_results"]) < 1:
            raise ValueError("required_consistent_results must be positive")
        if int(self.p["consensus_min_distinct_keyframes_per_robot"]) < 1:
            raise ValueError("consensus distinct-keyframe count must be positive")

    def _map_callback(self, msg: OccupancyGrid, robot_id: int) -> None:
        self.maps[robot_id] = msg
        self.map_version[robot_id] += 1

    def _odom_callback(self, msg: Odometry, robot_id: int) -> None:
        stamp_ns = _stamp_ns(msg.header.stamp) or self.get_clock().now().nanoseconds
        self.odom[robot_id].append(TimedPose(stamp_ns, _odom_pose(msg)))

    def _startup_elapsed(self, now_ns: int) -> bool:
        ready = all(robot_id in self.maps and self.odom[robot_id] for robot_id in (0, 1))
        if not ready:
            self.inputs_ready_since_ns = None
            return False
        if self.inputs_ready_since_ns is None:
            self.inputs_ready_since_ns = now_ns
            self.get_logger().info(
                "Both maps and corrected odometry streams are available; "
                "accumulating for %.1fs before place recognition."
                % float(self.p["startup_delay_sec"])
            )
        return now_ns - self.inputs_ready_since_ns >= int(
            max(0.0, float(self.p["startup_delay_sec"])) * 1e9
        )

    def _nearest_pose(self, robot_id: int, stamp_ns: int) -> Optional[TimedPose]:
        history = self.odom[robot_id]
        if not history:
            return None
        if stamp_ns <= 0:
            return history[-1]
        nearest = min(history, key=lambda item: abs(item.stamp_ns - stamp_ns))
        max_delta_ns = int(max(0.0, float(self.p["max_map_odom_delta_sec"])) * 1e9)
        return nearest if abs(nearest.stamp_ns - stamp_ns) <= max_delta_ns else None

    def _motion_is_enough(self, robot_id: int, pose: Pose2) -> bool:
        if not self.keyframes[robot_id]:
            return True
        previous = self.keyframes[robot_id][-1].pose_in_robot_map
        return (
            math.hypot(pose.x - previous.x, pose.y - previous.y)
            >= max(0.0, float(self.p["keyframe_translation_m"]))
            or angular_distance(pose.yaw, previous.yaw)
            >= max(0.0, float(self.p["keyframe_rotation_rad"]))
        )

    def _make_keyframe(self, robot_id: int) -> None:
        if self.map_version[robot_id] == self.used_map_version[robot_id]:
            return
        self.used_map_version[robot_id] = self.map_version[robot_id]
        msg = self.maps[robot_id]
        stamp_ns = _stamp_ns(msg.header.stamp)
        timed_pose = self._nearest_pose(robot_id, stamp_ns)
        if timed_pose is None or not self._motion_is_enough(robot_id, timed_pose.pose):
            return
        snapshot = _grid_snapshot(msg)
        if snapshot is None:
            self.get_logger().warn(f"Ignoring invalid r{robot_id} occupancy grid")
            return
        submap = extract_local_submap(
            snapshot,
            timed_pose.pose,
            radius_m=float(self.p["submap_radius_m"]),
            output_resolution_m=float(self.p["submap_resolution_m"]),
            occupied_threshold=int(self.p["occupied_threshold"]),
            max_boundary_points=int(self.p["max_boundary_points"]),
        )
        if (
            submap.known_ratio < float(self.p["min_known_ratio"])
            or len(submap.boundary_points) < int(self.p["min_boundary_points"])
        ):
            return
        descriptor = build_polar_descriptor(
            submap,
            num_rings=int(self.p["descriptor_num_rings"]),
            num_sectors=int(self.p["descriptor_num_sectors"]),
            min_known_cells_per_bin=int(self.p["descriptor_min_known_cells_per_bin"]),
        )
        keyframe = PlaceKeyframe(
            robot_id,
            self.next_keyframe_id[robot_id],
            stamp_ns or timed_pose.stamp_ns,
            timed_pose.pose,
            submap,
            descriptor,
        )
        self.next_keyframe_id[robot_id] += 1
        self.keyframes[robot_id].append(keyframe)
        maximum = max(2, int(self.p["max_keyframes_per_robot"]))
        self.keyframes[robot_id] = self.keyframes[robot_id][-maximum:]
        self.get_logger().info(
            "r%d place keyframe %d pose=(%.2f, %.2f, %.1fdeg), known=%.2f, boundaries=%d"
            % (
                robot_id,
                keyframe.keyframe_id,
                timed_pose.pose.x,
                timed_pose.pose.y,
                math.degrees(timed_pose.pose.yaw),
                submap.known_ratio,
                len(submap.boundary_points),
            )
        )
        self._enqueue_matches(keyframe)

    @staticmethod
    def _score_pair(robot0: PlaceKeyframe, robot1: PlaceKeyframe) -> Tuple[float, float, float]:
        similarity, yaw = match_polar_descriptors(robot0.descriptor, robot1.descriptor)
        ring_distance = float(np.linalg.norm(robot0.descriptor.ring_key - robot1.descriptor.ring_key))
        return similarity, yaw, ring_distance

    def _prefilter(self, keyframe: PlaceKeyframe, others: Sequence[PlaceKeyframe]) -> List[PlaceKeyframe]:
        limit = min(
            len(others),
            int(self.p["descriptor_top_k"]) * int(self.p["descriptor_candidate_multiplier"]),
        )
        return sorted(
            others,
            key=lambda other: float(
                np.linalg.norm(keyframe.descriptor.ring_key - other.descriptor.ring_key)
            ),
        )[:limit]

    def _best_partner(self, keyframe: PlaceKeyframe) -> Optional[int]:
        others = self.keyframes[1 - keyframe.robot_id]
        scored = []
        for other in self._prefilter(keyframe, others):
            r0, r1 = (keyframe, other) if keyframe.robot_id == 0 else (other, keyframe)
            scored.append((self._score_pair(r0, r1)[0], other.keyframe_id))
        return max(scored)[1] if scored else None

    def _enqueue_matches(self, keyframe: PlaceKeyframe) -> None:
        others = self.keyframes[1 - keyframe.robot_id]
        scored = []
        for other in self._prefilter(keyframe, others):
            r0, r1 = (keyframe, other) if keyframe.robot_id == 0 else (other, keyframe)
            similarity, yaw, ring_distance = self._score_pair(r0, r1)
            scored.append((similarity, -ring_distance, yaw, r0, r1))
        scored.sort(key=lambda item: (item[0], item[1]), reverse=True)
        top = scored[: max(1, int(self.p["descriptor_top_k"]))]
        if not top:
            return
        if len(top) > 1 and (
            top[0][0] - top[1][0] < float(self.p["descriptor_min_similarity_margin"])
        ):
            return
        for similarity, _, yaw, r0, r1 in top:
            pair = r0.keyframe_id, r1.keyframe_id
            if similarity < float(self.p["descriptor_min_similarity"]):
                continue
            if pair in self.attempted_pairs or pair in self.queued_pairs:
                continue
            if bool(self.p["require_mutual_match"]) and not (
                self._best_partner(r0) == r1.keyframe_id
                and self._best_partner(r1) == r0.keyframe_id
            ):
                continue
            self.pending.append(PendingMatch(r0, r1, similarity, yaw))
            self.queued_pairs.add(pair)
        self.pending.sort(key=lambda item: item.similarity, reverse=True)

    def _heading_allowed(self, alignment: Pose2) -> bool:
        return (
            not bool(self.p["enforce_heading_prior"])
            or angular_distance(alignment.yaw, float(self.p["expected_yaw_rad"]))
            <= float(self.p["max_yaw_deviation_rad"])
        )

    def _verify_candidates(self) -> None:
        budget = min(max(1, int(self.p["candidate_budget_per_cycle"])), len(self.pending))
        for _ in range(budget):
            candidate = self.pending.pop(0)
            self.queued_pairs.discard(candidate.pair)
            self.attempted_pairs.add(candidate.pair)
            result = register_submaps(
                target=candidate.robot0.submap,
                source=candidate.robot1.submap,
                descriptor_yaw_source_to_target=candidate.yaw_robot1_to_robot0,
                options=self.registration,
            )
            if not result.success:
                self.last_registration = f"pair={candidate.pair} rejected: {result.reason}"
                continue
            alignment = map_alignment_from_keyframes(
                candidate.robot0.pose_in_robot_map,
                result.transform_source_to_target,
                candidate.robot1.pose_in_robot_map,
            )
            if not self._heading_allowed(alignment):
                self.last_registration = f"pair={candidate.pair} rejected by heading prior"
                continue
            measurement = AlignmentMeasurement(
                alignment,
                candidate.robot0.keyframe_id,
                candidate.robot1.keyframe_id,
                candidate.similarity,
                result.symmetric_overlap,
                result.symmetric_rmse_m,
                result.free_space_conflict_ratio,
                max(candidate.robot0.stamp_ns, candidate.robot1.stamp_ns),
            )
            self.measurements.append(measurement)
            self.measurements = self.measurements[-max(1, int(self.p["consensus_max_measurements"])) :]
            self.last_registration = (
                "pair=%s accepted map1_to_map0=(%.2f,%.2f,%.1fdeg)"
                % (candidate.pair, alignment.x, alignment.y, math.degrees(alignment.yaw))
            )
            self.get_logger().info(
                "Verified r0:%d <-> r1:%d: descriptor=%.3f, overlap=%.3f, "
                "rmse=%.3f, conflict=%.3f, map1_to_map0=(%.2f, %.2f, %.1fdeg)"
                % (
                    candidate.robot0.keyframe_id,
                    candidate.robot1.keyframe_id,
                    candidate.similarity,
                    result.symmetric_overlap,
                    result.symmetric_rmse_m,
                    result.free_space_conflict_ratio,
                    alignment.x,
                    alignment.y,
                    math.degrees(alignment.yaw),
                )
            )
            self._monitor_lock(measurement)

    def _monitor_lock(self, measurement: AlignmentMeasurement) -> None:
        if self.alignment is None:
            return
        value = measurement.transform_map1_to_map0
        consistent = (
            math.hypot(value.x - self.alignment.x, value.y - self.alignment.y)
            <= float(self.p["locked_monitor_translation_m"])
            and angular_distance(value.yaw, self.alignment.yaw)
            <= float(self.p["locked_monitor_yaw_rad"])
        )
        if consistent:
            self.lock_mismatch_count = 0
            if self.state == "DEGRADED":
                self.state = "LOCKED"
            return
        self.lock_mismatch_count += 1
        if self.lock_mismatch_count >= max(1, int(self.p["degraded_after_inconsistent"])):
            self.state = "DEGRADED"
            self.get_logger().warn(
                "Alignment monitor is DEGRADED; retaining the last locked transform"
            )

    def _update_consensus(self) -> None:
        consensus = estimate_se2_consensus(
            self.measurements,
            translation_threshold_m=float(self.p["max_consistency_translation_m"]),
            yaw_threshold_rad=float(self.p["max_consistency_rotation_rad"]),
            min_supports=max(1, int(self.p["required_consistent_results"])),
            min_distinct_keyframes_per_robot=max(
                1, int(self.p["consensus_min_distinct_keyframes_per_robot"])
            ),
        )
        if consensus is None:
            if self.alignment is None:
                self.state = "TENTATIVE" if self.measurements else "SEARCHING"
            return
        if self.alignment is None:
            self._accept_consensus(consensus, "initial lock")
            return
        if bool(self.p["lock_after_first_alignment"]) and not bool(self.p["allow_relock"]):
            return
        delta_xy = math.hypot(
            consensus.transform_map1_to_map0.x - self.alignment.x,
            consensus.transform_map1_to_map0.y - self.alignment.y,
        )
        delta_yaw = angular_distance(consensus.transform_map1_to_map0.yaw, self.alignment.yaw)
        consistent = (
            delta_xy <= float(self.p["locked_monitor_translation_m"])
            and delta_yaw <= float(self.p["locked_monitor_yaw_rad"])
        )
        if consistent and not bool(self.p["lock_after_first_alignment"]):
            self._accept_consensus(consensus, "consistent refresh")
        elif self.state == "DEGRADED" and bool(self.p["allow_relock"]):
            self._accept_consensus(consensus, "relock")

    def _accept_consensus(self, consensus: ConsensusResult, reason: str) -> None:
        self.alignment = consensus.transform_map1_to_map0
        self.consensus = consensus
        self.state = "LOCKED"
        self.lock_mismatch_count = 0
        self._publish_alignment()
        self.get_logger().info(
            "Alignment %s: map1_to_map0=(%.3f, %.3f, %.2fdeg), supports=%d, "
            "distinct=(%d,%d), spread=(%.3fm, %.2fdeg)"
            % (
                reason,
                self.alignment.x,
                self.alignment.y,
                math.degrees(self.alignment.yaw),
                consensus.support_count,
                consensus.distinct_robot0_keyframes,
                consensus.distinct_robot1_keyframes,
                consensus.translation_spread_m,
                math.degrees(consensus.yaw_spread_rad),
            )
        )

    def _transform_message(self, transform: Pose2, parent: str, child: str) -> TransformStamped:
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x = transform.x
        msg.transform.translation.y = transform.y
        msg.transform.rotation.z = math.sin(0.5 * transform.yaw)
        msg.transform.rotation.w = math.cos(0.5 * transform.yaw)
        return msg

    def _publish_alignment(self) -> None:
        if self.alignment is not None:
            self.alignment_pub.publish(
                self._transform_message(
                    self.alignment,
                    str(self.p["target_frame_id"]),
                    str(self.p["source_frame_id"]),
                )
            )

    def _publish_relative_transform(self) -> None:
        if self.alignment is None or not self.odom[0] or not self.odom[1]:
            return
        latest0, latest1 = self.odom[0][-1], self.odom[1][-1]
        if abs(latest0.stamp_ns - latest1.stamp_ns) > int(
            max(0.0, float(self.p["max_relative_odom_delta_sec"])) * 1e9
        ):
            return
        robot0_from_robot1 = compose_pose(
            inverse_pose(latest0.pose), compose_pose(self.alignment, latest1.pose)
        )
        self.relative_pub.publish(
            self._transform_message(
                robot0_from_robot1,
                str(self.p["robot0_base_frame_id"]),
                str(self.p["robot1_base_frame_id"]),
            )
        )

    def _publish_status(self, now_ns: int) -> None:
        period_ns = int(max(0.1, float(self.p["status_period_sec"])) * 1e9)
        if now_ns - self.last_status_ns < period_ns:
            return
        self.last_status_ns = now_ns
        payload = {
            "state": self.state,
            "keyframes": [len(self.keyframes[0]), len(self.keyframes[1])],
            "pending_candidates": len(self.pending),
            "attempted_pairs": len(self.attempted_pairs),
            "verified_measurements": len(self.measurements),
            "last_registration": self.last_registration,
            "lock_mismatch_count": self.lock_mismatch_count,
        }
        if self.alignment is not None:
            payload["map1_to_map0"] = {
                "x": self.alignment.x,
                "y": self.alignment.y,
                "yaw_rad": self.alignment.yaw,
                "yaw_deg": math.degrees(self.alignment.yaw),
            }
        if self.consensus is not None:
            payload["consensus"] = {
                "supports": self.consensus.support_count,
                "distinct_robot0_keyframes": self.consensus.distinct_robot0_keyframes,
                "distinct_robot1_keyframes": self.consensus.distinct_robot1_keyframes,
                "translation_spread_m": self.consensus.translation_spread_m,
                "yaw_spread_deg": math.degrees(self.consensus.yaw_spread_rad),
            }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)

    def _process(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._startup_elapsed(now_ns):
            self._make_keyframe(0)
            self._make_keyframe(1)
            self._verify_candidates()
            self._update_consensus()
            self._publish_alignment()
            self._publish_relative_transform()
        self._publish_status(now_ns)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InterRobotPlaceAlignment()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
