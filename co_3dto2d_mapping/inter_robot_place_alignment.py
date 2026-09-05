#!/usr/bin/env python3
"""CPU-only two-robot occupancy place recognition and map alignment.

Frozen robot-centric occupancy keyframes are retrieved with a polar descriptor,
verified by symmetric correlative matching plus trimmed ICP, converted into
``r0/odom <- r1/odom`` samples, and reduced to one robust SE(2) consensus. No
pose graph is constructed; the existing world/merged-map compositor consumes
the published transform.
"""

from __future__ import annotations

from collections import deque
from copy import deepcopy
from dataclasses import dataclass
import json
import math
from typing import Deque, Dict, List, Optional, Set, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from co_3dto2d_mapping.alignment_consensus import (
    AlignmentMeasurement,
    ConsensusResult,
    estimate_alignment_consensus,
)
from co_3dto2d_mapping.occupancy_submap import (
    GridGeometry,
    LocalOccupancyPatch,
    extract_local_patch,
)
from co_3dto2d_mapping.planar_transform_utils import (
    PlanarTransform,
    normalize_angle,
    world_from_source_odom,
)
from co_3dto2d_mapping.polar_occupancy_context import (
    ContextMatch,
    PolarContext,
    PolarContextConfig,
    build_polar_context,
    match_polar_context,
    rank_ring_candidates,
)
from co_3dto2d_mapping.se2_map_registration import (
    RegistrationConfig,
    RegistrationResult,
    register_submaps,
)


PARAMETER_DEFAULTS = {
    # Topics and frame contract.
    "robot0_map_topic": "/r0/toy/global_occupancy",
    "robot1_map_topic": "/r1/toy/global_occupancy",
    "robot0_odom_topic": "/r0/toy/planar_odometry",
    "robot1_odom_topic": "/r1/toy/planar_odometry",
    "robot_odom_frame_format": "r{robot_id}/odom",
    "alignment_topic": "/toy/initial_xy_alignment",
    "status_topic": "/toy/place_recognition/status",
    "target_frame_id": "map",
    "source_frame_id": "r1/odom",
    "strict_frame_validation": True,
    # Scheduling and time matching.
    "processing_period_sec": 1.0,
    "startup_delay_sec": 3.0,
    "map_timeout_sec": 5.0,
    "odom_history_sec": 12.0,
    "max_map_odom_stamp_delta_sec": 0.40,
    "status_publish_period_sec": 1.0,
    # Keyframe extraction.
    "keyframe_translation_m": 1.0,
    "keyframe_rotation_rad": math.radians(10.0),
    "keyframe_min_interval_sec": 1.0,
    "stationary_keyframe_period_sec": 2.0,
    "max_keyframes_per_robot": 40,
    "submap_radius_m": 15.0,
    "submap_resolution_m": 0.10,
    "occupied_threshold": 50,
    "min_known_ratio": 0.12,
    "min_boundary_points": 120,
    # Descriptor and candidate retrieval.
    "descriptor_num_rings": 20,
    "descriptor_num_sectors": 60,
    "descriptor_recenter_on_occupied_centroid": False,
    "descriptor_top_k": 8,
    "descriptor_max_distance": 0.45,
    "descriptor_ratio_test": 1.0,
    "require_mutual_best_match": False,
    "geometric_candidates_per_keyframe": 2,
    "use_descriptor_yaw_hypothesis": True,
    "use_odometry_yaw_hypothesis": True,
    "expected_map_yaw_rad": 0.0,
    "yaw_hypothesis_duplicate_rad": math.radians(3.0),
    "try_opposite_descriptor_yaw": False,
    # Correlative search and ICP verification.
    "registration_coarse_translation_range_m": 2.5,
    "registration_coarse_translation_step_m": 0.25,
    "registration_coarse_yaw_range_rad": math.radians(12.0),
    "registration_coarse_yaw_step_rad": math.radians(2.0),
    "registration_fine_translation_range_m": 0.35,
    "registration_fine_translation_step_m": 0.05,
    "registration_fine_yaw_range_rad": math.radians(2.0),
    "registration_fine_yaw_step_rad": math.radians(0.5),
    "registration_search_max_points": 700,
    "registration_search_batch_size": 256,
    "registration_distance_clip_m": 0.80,
    "registration_overlap_distance_m": 0.25,
    "registration_overlap_penalty_weight": 0.20,
    "registration_icp_max_iterations": 35,
    "registration_icp_max_correspondence_m": 0.50,
    "registration_icp_trim_ratio": 0.75,
    "registration_icp_max_points": 2500,
    "registration_free_conflict_clearance_m": 0.15,
    "registration_max_free_samples": 1200,
    "registration_min_correspondences": 80,
    "registration_min_symmetric_overlap": 0.30,
    "registration_max_symmetric_rmse_m": 0.25,
    "registration_max_free_conflict_ratio": 0.14,
    # Weighting and single-transform consensus.
    "measurement_descriptor_sigma": 0.25,
    "measurement_rmse_sigma_m": 0.15,
    "measurement_history_sec": 180.0,
    "max_measurements": 80,
    "consensus_translation_cluster_m": 0.40,
    "consensus_yaw_cluster_rad": math.radians(4.0),
    "consensus_min_measurements": 3,
    "consensus_min_distinct_keyframes": 2,
    "consensus_max_translation_rms_m": 0.18,
    "consensus_max_yaw_rms_rad": math.radians(2.0),
    "lock_after_consensus": True,
    "stop_processing_after_lock": True,
}


@dataclass(frozen=True)
class StampedOdomPose:
    stamp_ns: int
    pose: PlanarTransform
    frame_id: str


@dataclass(frozen=True)
class OccupancyKeyframe:
    robot_id: int
    keyframe_id: int
    stamp_ns: int
    created_ns: int
    pose: PlanarTransform
    frame_id: str
    patch: LocalOccupancyPatch
    context: PolarContext


@dataclass(frozen=True)
class DescriptorCandidate:
    target: OccupancyKeyframe
    source: OccupancyKeyframe
    match: ContextMatch
    ring_distance: float


class InterRobotPlaceAlignment(Node):
    def __init__(self) -> None:
        super().__init__("inter_robot_place_alignment")
        for name, default in PARAMETER_DEFAULTS.items():
            self.declare_parameter(name, default)
        self._read_parameters()
        self._validate_parameters()

        map_qos = QoSProfile(depth=3)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.VOLATILE
        odom_qos = QoSProfile(depth=50)
        odom_qos.reliability = ReliabilityPolicy.RELIABLE
        odom_qos.durability = DurabilityPolicy.VOLATILE
        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.alignment_publisher = self.create_publisher(
            TransformStamped, self.alignment_topic, transient_qos
        )
        self.status_publisher = self.create_publisher(
            String, self.status_topic, transient_qos
        )
        self._map_subscriptions = [
            self.create_subscription(
                OccupancyGrid,
                topic,
                lambda msg, rid=robot_id: self._map_callback(msg, rid),
                map_qos,
            )
            for robot_id, topic in enumerate(self.robot_map_topics)
        ]
        self._odom_subscriptions = [
            self.create_subscription(
                Odometry,
                topic,
                lambda msg, rid=robot_id: self._odom_callback(msg, rid),
                odom_qos,
            )
            for robot_id, topic in enumerate(self.robot_odom_topics)
        ]

        self.latest_maps: List[Optional[OccupancyGrid]] = [None, None]
        self.latest_map_receipt_ns = [0, 0]
        self.odom_histories: Tuple[
            Deque[StampedOdomPose], Deque[StampedOdomPose]
        ] = (deque(), deque())
        self.keyframes: Tuple[List[OccupancyKeyframe], List[OccupancyKeyframe]] = (
            [],
            [],
        )
        self.next_keyframe_ids = [0, 0]
        self.attempted_pairs: Set[Tuple[int, int]] = set()
        self.measurements: List[AlignmentMeasurement] = []
        self.alignment_message: Optional[TransformStamped] = None
        self.last_consensus: Optional[ConsensusResult] = None
        self.state = "SEARCHING"
        self.ready_since_ns: Optional[int] = None
        self.last_status_publish_ns = 0
        self.last_warning_ns: Dict[str, int] = {}
        self.last_match_summary: Dict[str, object] = {}

        self.timer = self.create_timer(
            self.processing_period_sec, self._processing_timer
        )
        self.get_logger().info(
            "2-D occupancy place alignment started. maps=(%s, %s) "
            "odom=(%s, %s) descriptor=%dx%d radius=%.1fm "
            "keyframe=%.2fm/%.1fdeg stationary=%.1fs consensus=%d/%d "
            "output=%s (%s <- %s), lock=%s"
            % (
                *self.robot_map_topics,
                *self.robot_odom_topics,
                self.context_config.num_rings,
                self.context_config.num_sectors,
                self.submap_radius_m,
                self.keyframe_translation_m,
                math.degrees(self.keyframe_rotation_rad),
                self.stationary_keyframe_period_sec,
                self.consensus_min_measurements,
                self.consensus_min_distinct_keyframes,
                self.alignment_topic,
                self.target_frame_id,
                self.source_frame_id,
                "true" if self.lock_after_consensus else "false",
            )
        )

    def _parameter(self, name: str):
        return self.get_parameter(name).value

    def _read_parameters(self) -> None:
        p = self._parameter
        self.robot_map_topics = (
            str(p("robot0_map_topic")),
            str(p("robot1_map_topic")),
        )
        self.robot_odom_topics = (
            str(p("robot0_odom_topic")),
            str(p("robot1_odom_topic")),
        )
        for name in (
            "robot_odom_frame_format",
            "alignment_topic",
            "status_topic",
            "target_frame_id",
            "source_frame_id",
        ):
            setattr(self, name, str(p(name)))
        self.strict_frame_validation = bool(p("strict_frame_validation"))

        float_names = (
            "processing_period_sec",
            "startup_delay_sec",
            "map_timeout_sec",
            "odom_history_sec",
            "max_map_odom_stamp_delta_sec",
            "status_publish_period_sec",
            "keyframe_translation_m",
            "keyframe_rotation_rad",
            "keyframe_min_interval_sec",
            "stationary_keyframe_period_sec",
            "submap_radius_m",
            "submap_resolution_m",
            "min_known_ratio",
            "descriptor_max_distance",
            "descriptor_ratio_test",
            "expected_map_yaw_rad",
            "yaw_hypothesis_duplicate_rad",
            "measurement_descriptor_sigma",
            "measurement_rmse_sigma_m",
            "measurement_history_sec",
            "consensus_translation_cluster_m",
            "consensus_yaw_cluster_rad",
            "consensus_max_translation_rms_m",
            "consensus_max_yaw_rms_rad",
        )
        for name in float_names:
            setattr(self, name, float(p(name)))
        int_names = (
            "max_keyframes_per_robot",
            "occupied_threshold",
            "min_boundary_points",
            "descriptor_top_k",
            "geometric_candidates_per_keyframe",
            "max_measurements",
            "consensus_min_measurements",
            "consensus_min_distinct_keyframes",
        )
        for name in int_names:
            setattr(self, name, int(p(name)))
        bool_names = (
            "require_mutual_best_match",
            "use_descriptor_yaw_hypothesis",
            "use_odometry_yaw_hypothesis",
            "try_opposite_descriptor_yaw",
            "lock_after_consensus",
            "stop_processing_after_lock",
        )
        for name in bool_names:
            setattr(self, name, bool(p(name)))

        self.context_config = PolarContextConfig(
            max_radius_m=self.submap_radius_m,
            num_rings=int(p("descriptor_num_rings")),
            num_sectors=int(p("descriptor_num_sectors")),
            recenter_on_occupied_centroid=bool(
                p("descriptor_recenter_on_occupied_centroid")
            ),
        ).validated()
        self.registration_config = RegistrationConfig(
            coarse_translation_range_m=float(
                p("registration_coarse_translation_range_m")
            ),
            coarse_translation_step_m=float(
                p("registration_coarse_translation_step_m")
            ),
            coarse_yaw_range_rad=float(
                p("registration_coarse_yaw_range_rad")
            ),
            coarse_yaw_step_rad=float(
                p("registration_coarse_yaw_step_rad")
            ),
            fine_translation_range_m=float(
                p("registration_fine_translation_range_m")
            ),
            fine_translation_step_m=float(
                p("registration_fine_translation_step_m")
            ),
            fine_yaw_range_rad=float(p("registration_fine_yaw_range_rad")),
            fine_yaw_step_rad=float(p("registration_fine_yaw_step_rad")),
            search_max_points=int(p("registration_search_max_points")),
            search_batch_size=int(p("registration_search_batch_size")),
            distance_clip_m=float(p("registration_distance_clip_m")),
            overlap_distance_m=float(p("registration_overlap_distance_m")),
            overlap_penalty_weight=float(
                p("registration_overlap_penalty_weight")
            ),
            icp_max_iterations=int(p("registration_icp_max_iterations")),
            icp_max_correspondence_m=float(
                p("registration_icp_max_correspondence_m")
            ),
            icp_trim_ratio=float(p("registration_icp_trim_ratio")),
            icp_max_points=int(p("registration_icp_max_points")),
            free_conflict_clearance_m=float(
                p("registration_free_conflict_clearance_m")
            ),
            max_free_samples=int(p("registration_max_free_samples")),
            min_correspondences=int(p("registration_min_correspondences")),
            min_symmetric_overlap=float(
                p("registration_min_symmetric_overlap")
            ),
            max_symmetric_rmse_m=float(
                p("registration_max_symmetric_rmse_m")
            ),
            max_free_conflict_ratio=float(
                p("registration_max_free_conflict_ratio")
            ),
        ).validated()

    def _validate_parameters(self) -> None:
        if not all(self.robot_map_topics + self.robot_odom_topics):
            raise ValueError("map and odometry topics must be non-empty")
        if not all(
            (
                self.alignment_topic,
                self.status_topic,
                self.target_frame_id,
                self.source_frame_id,
            )
        ):
            raise ValueError("alignment topics and frame IDs must be non-empty")
        positive = (
            self.processing_period_sec,
            self.odom_history_sec,
            self.keyframe_min_interval_sec,
            self.stationary_keyframe_period_sec,
            self.submap_radius_m,
            self.submap_resolution_m,
            self.measurement_descriptor_sigma,
            self.measurement_rmse_sigma_m,
            self.consensus_translation_cluster_m,
            self.consensus_yaw_cluster_rad,
        )
        if not all(np.isfinite(item) and item > 0.0 for item in positive):
            raise ValueError("positive place-recognition parameters are invalid")
        non_negative = (
            self.startup_delay_sec,
            self.map_timeout_sec,
            self.max_map_odom_stamp_delta_sec,
            self.keyframe_translation_m,
            self.keyframe_rotation_rad,
            self.measurement_history_sec,
            self.consensus_max_translation_rms_m,
            self.consensus_max_yaw_rms_rad,
        )
        if not all(
            np.isfinite(item) and item >= 0.0 for item in non_negative
        ):
            raise ValueError("non-negative place-recognition parameters are invalid")
        if not 0.0 <= self.min_known_ratio <= 1.0:
            raise ValueError("min_known_ratio must be in [0, 1]")
        if not 0.0 < self.descriptor_ratio_test <= 1.0:
            raise ValueError("descriptor_ratio_test must be in (0, 1]")
        counts = (
            self.max_keyframes_per_robot,
            self.min_boundary_points,
            self.descriptor_top_k,
            self.geometric_candidates_per_keyframe,
            self.max_measurements,
            self.consensus_min_measurements,
            self.consensus_min_distinct_keyframes,
        )
        if not all(item >= 1 for item in counts):
            raise ValueError("keyframe/candidate/consensus counts must be positive")

    @staticmethod
    def _stamp_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @staticmethod
    def _yaw_from_quaternion(quaternion) -> float:
        return math.atan2(
            2.0
            * (
                quaternion.w * quaternion.z
                + quaternion.x * quaternion.y
            ),
            1.0
            - 2.0
            * (
                quaternion.y * quaternion.y
                + quaternion.z * quaternion.z
            ),
        )

    @classmethod
    def _pose_from_odom(cls, msg: Odometry) -> PlanarTransform:
        pose = msg.pose.pose
        return (
            float(pose.position.x),
            float(pose.position.y),
            float(cls._yaw_from_quaternion(pose.orientation)),
        )

    def _map_callback(self, msg: OccupancyGrid, robot_id: int) -> None:
        self.latest_maps[robot_id] = msg
        self.latest_map_receipt_ns[robot_id] = self.get_clock().now().nanoseconds

    def _odom_callback(self, msg: Odometry, robot_id: int) -> None:
        if msg.pose.covariance[0] > 1000.0:
            return
        stamp_ns = self._stamp_ns(msg.header.stamp)
        if stamp_ns <= 0:
            return
        history = self.odom_histories[robot_id]
        if history and stamp_ns < history[-1].stamp_ns:
            history.clear()
        history.append(
            StampedOdomPose(
                stamp_ns=stamp_ns,
                pose=self._pose_from_odom(msg),
                frame_id=str(msg.header.frame_id),
            )
        )
        cutoff_ns = stamp_ns - int(self.odom_history_sec * 1e9)
        while history and history[0].stamp_ns < cutoff_ns:
            history.popleft()

    def _warn_throttled(
        self, key: str, text: str, period_sec: float = 2.0
    ) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_warning_ns.get(key, 0) < int(period_sec * 1e9):
            return
        self.last_warning_ns[key] = now_ns
        self.get_logger().warn(text)

    def _nearest_odom_pose(
        self, robot_id: int, stamp_ns: int
    ) -> Optional[StampedOdomPose]:
        history = self.odom_histories[robot_id]
        if not history:
            return None
        if stamp_ns <= 0:
            return history[-1]
        nearest = min(history, key=lambda sample: abs(sample.stamp_ns - stamp_ns))
        if (
            self.max_map_odom_stamp_delta_sec > 0.0
            and abs(nearest.stamp_ns - stamp_ns)
            > int(self.max_map_odom_stamp_delta_sec * 1e9)
        ):
            return None
        return nearest

    def _inputs_ready(self, now_ns: int) -> bool:
        if any(msg is None for msg in self.latest_maps):
            return False
        if any(not history for history in self.odom_histories):
            return False
        if self.map_timeout_sec > 0.0:
            timeout_ns = int(self.map_timeout_sec * 1e9)
            for robot_id, receipt_ns in enumerate(self.latest_map_receipt_ns):
                if not receipt_ns or now_ns - receipt_ns > timeout_ns:
                    self._warn_throttled(
                        f"stale_map_{robot_id}",
                        "Waiting for a fresh occupancy map on %s"
                        % self.robot_map_topics[robot_id],
                    )
                    return False
        if self.ready_since_ns is None:
            self.ready_since_ns = now_ns
            self.get_logger().info(
                "Both robots have map and odometry data; waiting %.1fs before "
                "creating place-recognition keyframes."
                % self.startup_delay_sec
            )
        return now_ns - self.ready_since_ns >= int(self.startup_delay_sec * 1e9)

    def _expected_odom_frame(self, robot_id: int) -> str:
        return self.robot_odom_frame_format.format(robot_id=robot_id)

    def _frame_pair_valid(
        self, robot_id: int, map_frame: str, odom_frame: str
    ) -> bool:
        expected = self._expected_odom_frame(robot_id)
        frames = [frame for frame in (map_frame, odom_frame) if frame]
        valid = all(frame == expected for frame in frames)
        if not valid:
            self._warn_throttled(
                f"frame_{robot_id}",
                "r%d keyframe requires map/odom in %s, got map=%r odom=%r"
                % (robot_id, expected, map_frame, odom_frame),
                5.0,
            )
        return valid or not self.strict_frame_validation

    def _should_create_keyframe(
        self,
        robot_id: int,
        stamp_ns: int,
        pose: PlanarTransform,
        now_ns: int,
    ) -> bool:
        frames = self.keyframes[robot_id]
        if not frames:
            return True
        previous = frames[-1]
        if stamp_ns > 0 and previous.stamp_ns > 0 and stamp_ns <= previous.stamp_ns:
            return False
        elapsed = (now_ns - previous.created_ns) / 1e9
        if elapsed < self.keyframe_min_interval_sec:
            return False
        translation = math.hypot(
            pose[0] - previous.pose[0], pose[1] - previous.pose[1]
        )
        rotation = abs(normalize_angle(pose[2] - previous.pose[2]))
        if (
            translation >= self.keyframe_translation_m
            or rotation >= self.keyframe_rotation_rad
        ):
            return True
        # Stationary snapshots provide time-distinct evidence while startup
        # occupancy settles; consensus prevents a one-off pair from locking.
        return (
            self.alignment_message is None
            and elapsed >= self.stationary_keyframe_period_sec
        )

    def _create_keyframe(
        self, robot_id: int, now_ns: int
    ) -> Optional[OccupancyKeyframe]:
        msg = self.latest_maps[robot_id]
        if msg is None:
            return None
        stamp_ns = self._stamp_ns(msg.header.stamp)
        odom = self._nearest_odom_pose(robot_id, stamp_ns)
        if odom is None:
            self._warn_throttled(
                f"odom_match_{robot_id}",
                "Waiting for odometry within %.3fs of the r%d map stamp."
                % (self.max_map_odom_stamp_delta_sec, robot_id),
            )
            return None
        if not self._frame_pair_valid(
            robot_id, str(msg.header.frame_id), odom.frame_id
        ):
            return None
        if not self._should_create_keyframe(robot_id, stamp_ns, odom.pose, now_ns):
            return None

        origin = msg.info.origin
        geometry = GridGeometry(
            resolution=float(msg.info.resolution),
            width=int(msg.info.width),
            height=int(msg.info.height),
            origin_x=float(origin.position.x),
            origin_y=float(origin.position.y),
            origin_yaw=float(self._yaw_from_quaternion(origin.orientation)),
        )
        try:
            patch = extract_local_patch(
                np.asarray(msg.data, dtype=np.int16),
                geometry,
                odom.pose,
                self.submap_radius_m,
                output_resolution=self.submap_resolution_m,
                occupied_threshold=self.occupied_threshold,
            )
        except ValueError as exc:
            self._warn_throttled(
                f"patch_{robot_id}",
                "Could not extract r%d occupancy keyframe: %s"
                % (robot_id, exc),
            )
            return None
        if patch.known_ratio < self.min_known_ratio:
            self._warn_throttled(
                f"known_{robot_id}",
                "r%d keyframe is only %.1f%% observed; need %.1f%%."
                % (
                    robot_id,
                    100.0 * patch.known_ratio,
                    100.0 * self.min_known_ratio,
                ),
            )
            return None
        if patch.occupied_boundary_count < self.min_boundary_points:
            self._warn_throttled(
                f"boundary_{robot_id}",
                "r%d keyframe has %d boundary cells; need %d."
                % (
                    robot_id,
                    patch.occupied_boundary_count,
                    self.min_boundary_points,
                ),
            )
            return None

        keyframe = OccupancyKeyframe(
            robot_id=robot_id,
            keyframe_id=self.next_keyframe_ids[robot_id],
            stamp_ns=stamp_ns,
            created_ns=now_ns,
            pose=odom.pose,
            frame_id=self._expected_odom_frame(robot_id),
            patch=patch,
            context=build_polar_context(patch, self.context_config),
        )
        self.next_keyframe_ids[robot_id] += 1
        frames = self.keyframes[robot_id]
        frames.append(keyframe)
        if len(frames) > self.max_keyframes_per_robot:
            frames.pop(0)
        self.get_logger().info(
            "Created r%d occupancy keyframe %d: pose=(%.2f, %.2f, %.1fdeg) "
            "known=%.1f%% boundary=%d database=(%d, %d)"
            % (
                robot_id,
                keyframe.keyframe_id,
                *keyframe.pose[:2],
                math.degrees(keyframe.pose[2]),
                100.0 * patch.known_ratio,
                patch.occupied_boundary_count,
                len(self.keyframes[0]),
                len(self.keyframes[1]),
            )
        )
        return keyframe

    def _mutual_best(self, candidate: DescriptorCandidate) -> bool:
        if not self.require_mutual_best_match:
            return True
        query = candidate.source
        best_id = None
        best_distance = float("inf")
        for target in self.keyframes[0]:
            match = match_polar_context(
                target.context, query.context, self.context_config
            )
            if match.distance < best_distance:
                best_distance = match.distance
                best_id = target.keyframe_id
        return best_id == candidate.target.keyframe_id

    def _descriptor_candidates(
        self, new_keyframe: OccupancyKeyframe
    ) -> List[DescriptorCandidate]:
        database = self.keyframes[1 - new_keyframe.robot_id]
        if not database:
            return []
        ring_rank = rank_ring_candidates(
            new_keyframe.context,
            [keyframe.context for keyframe in database],
            self.descriptor_top_k,
        )
        candidates: List[DescriptorCandidate] = []
        for index, ring_distance in ring_rank:
            other = database[index]
            target, source = (
                (new_keyframe, other)
                if new_keyframe.robot_id == 0
                else (other, new_keyframe)
            )
            pair = (target.keyframe_id, source.keyframe_id)
            if pair in self.attempted_pairs:
                continue
            match = match_polar_context(
                target.context, source.context, self.context_config
            )
            if match.distance > self.descriptor_max_distance:
                continue
            candidate = DescriptorCandidate(
                target=target,
                source=source,
                match=match,
                ring_distance=float(ring_distance),
            )
            if self._mutual_best(candidate):
                candidates.append(candidate)
        candidates.sort(
            key=lambda item: (item.match.distance, item.ring_distance)
        )
        if (
            self.descriptor_ratio_test < 1.0
            and len(candidates) >= 2
            and candidates[0].match.distance
            > self.descriptor_ratio_test
            * max(candidates[1].match.distance, 1e-9)
        ):
            self.get_logger().info(
                "Descriptor candidate for r%d keyframe %d is ambiguous: %.3f/%.3f"
                % (
                    new_keyframe.robot_id,
                    new_keyframe.keyframe_id,
                    candidates[0].match.distance,
                    candidates[1].match.distance,
                )
            )
            return []
        return candidates[: self.geometric_candidates_per_keyframe]

    def _yaw_hypotheses(
        self, candidate: DescriptorCandidate
    ) -> List[Tuple[str, float]]:
        hypotheses: List[Tuple[str, float]] = []
        if self.use_odometry_yaw_hypothesis:
            hypotheses.append(
                (
                    "odom_prior",
                    normalize_angle(
                        self.expected_map_yaw_rad
                        + candidate.source.pose[2]
                        - candidate.target.pose[2]
                    ),
                )
            )
        if self.use_descriptor_yaw_hypothesis:
            hypotheses.append(("polar_context", candidate.match.yaw_rad))
            if self.try_opposite_descriptor_yaw:
                hypotheses.append(
                    (
                        "polar_context_opposite",
                        normalize_angle(candidate.match.yaw_rad + math.pi),
                    )
                )
        unique: List[Tuple[str, float]] = []
        for label, yaw in hypotheses:
            if any(
                abs(normalize_angle(yaw - existing))
                <= self.yaw_hypothesis_duplicate_rad
                for _, existing in unique
            ):
                continue
            unique.append((label, yaw))
        return unique or [("zero", 0.0)]

    @staticmethod
    def _registration_rank(result: RegistrationResult) -> Tuple[float, ...]:
        return (
            float(int(result.accepted)),
            result.symmetric_overlap,
            -result.symmetric_rmse_m,
            -result.free_conflict_ratio,
            -result.search_score,
            float(result.correspondences),
        )

    def _verify_candidate(
        self, candidate: DescriptorCandidate
    ) -> Optional[Tuple[RegistrationResult, str]]:
        results = [
            (
                register_submaps(
                    candidate.target.patch,
                    candidate.source.patch,
                    yaw,
                    self.registration_config,
                ),
                label,
            )
            for label, yaw in self._yaw_hypotheses(candidate)
        ]
        return (
            max(results, key=lambda item: self._registration_rank(item[0]))
            if results
            else None
        )

    def _measurement_weight(
        self, descriptor_distance: float, result: RegistrationResult
    ) -> float:
        descriptor_quality = math.exp(
            -max(0.0, descriptor_distance)
            / self.measurement_descriptor_sigma
        )
        rmse_quality = math.exp(
            -0.5
            * (
                result.symmetric_rmse_m
                / max(self.measurement_rmse_sigma_m, 1e-9)
            )
            ** 2
        )
        return max(
            1e-6,
            descriptor_quality
            * result.symmetric_overlap**2
            * rmse_quality
            * max(0.0, 1.0 - result.free_conflict_ratio),
        )

    def _prune_measurements(self, now_ns: int) -> None:
        if self.measurement_history_sec > 0.0:
            cutoff = now_ns - int(self.measurement_history_sec * 1e9)
            self.measurements = [
                item
                for item in self.measurements
                if item.stamp_ns <= 0 or item.stamp_ns >= cutoff
            ]
        if len(self.measurements) > self.max_measurements:
            self.measurements = self.measurements[-self.max_measurements :]

    def _update_consensus(self, now_ns: int) -> None:
        self._prune_measurements(now_ns)
        consensus = estimate_alignment_consensus(
            self.measurements,
            translation_cluster_m=self.consensus_translation_cluster_m,
            yaw_cluster_rad=self.consensus_yaw_cluster_rad,
            min_measurements=self.consensus_min_measurements,
            min_distinct_keyframes_per_robot=(
                self.consensus_min_distinct_keyframes
            ),
        )
        self.last_consensus = consensus
        if consensus is None:
            self.state = "SEARCHING"
            return
        self.state = "TENTATIVE"
        spread_ok = (
            consensus.translation_rms_m
            <= self.consensus_max_translation_rms_m
            and consensus.yaw_rms_rad <= self.consensus_max_yaw_rms_rad
        )
        if not consensus.accepted or not spread_ok:
            return
        if self.alignment_message is not None and self.lock_after_consensus:
            self.state = "LOCKED"
            return

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.target_frame_id
        transform.child_frame_id = self.source_frame_id
        transform.transform.translation.x = consensus.transform[0]
        transform.transform.translation.y = consensus.transform[1]
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = math.sin(
            0.5 * consensus.transform[2]
        )
        transform.transform.rotation.w = math.cos(
            0.5 * consensus.transform[2]
        )
        self.alignment_message = transform
        self.alignment_publisher.publish(transform)
        self.state = "LOCKED" if self.lock_after_consensus else "TRACKING"
        self.get_logger().info(
            "Inter-robot occupancy alignment accepted: %s <- %s = "
            "(x=%.3f y=%.3f yaw=%.2fdeg), supports=%d target_kf=%d "
            "source_kf=%d rms=%.3fm/%.2fdeg weight=%.3f lock=%s"
            % (
                self.target_frame_id,
                self.source_frame_id,
                *consensus.transform[:2],
                math.degrees(consensus.transform[2]),
                consensus.measurement_count,
                consensus.target_keyframe_count,
                consensus.source_keyframe_count,
                consensus.translation_rms_m,
                math.degrees(consensus.yaw_rms_rad),
                consensus.total_weight,
                "true" if self.lock_after_consensus else "false",
            )
        )

    def _process_candidate(
        self, candidate: DescriptorCandidate, now_ns: int
    ) -> None:
        pair = (candidate.target.keyframe_id, candidate.source.keyframe_id)
        self.attempted_pairs.add(pair)
        verified = self._verify_candidate(candidate)
        if verified is None:
            return
        result, yaw_label = verified
        self.last_match_summary = {
            "target_keyframe": candidate.target.keyframe_id,
            "source_keyframe": candidate.source.keyframe_id,
            "descriptor_distance": round(candidate.match.distance, 5),
            "descriptor_yaw_deg": round(
                math.degrees(candidate.match.yaw_rad), 3
            ),
            "yaw_hypothesis": yaw_label,
            "registration_reason": result.reason,
            "overlap": round(result.symmetric_overlap, 5),
            "rmse_m": round(result.symmetric_rmse_m, 5),
            "free_conflict": round(result.free_conflict_ratio, 5),
        }
        if not result.accepted:
            self.get_logger().info(
                "Rejected place pair r0:k%d <-> r1:k%d: descriptor=%.3f "
                "yaw=%s overlap=%.3f rmse=%.3f conflict=%.3f reason=%s"
                % (
                    pair[0],
                    pair[1],
                    candidate.match.distance,
                    yaw_label,
                    result.symmetric_overlap,
                    result.symmetric_rmse_m,
                    result.free_conflict_ratio,
                    result.reason,
                )
            )
            return

        map0_from_map1 = world_from_source_odom(
            candidate.target.pose,
            result.transform,
            candidate.source.pose,
        )
        measurement = AlignmentMeasurement(
            target_keyframe_id=pair[0],
            source_keyframe_id=pair[1],
            transform=map0_from_map1,
            weight=self._measurement_weight(
                candidate.match.distance, result
            ),
            stamp_ns=now_ns,
            descriptor_distance=candidate.match.distance,
            rmse_m=result.symmetric_rmse_m,
            overlap=result.symmetric_overlap,
            free_conflict_ratio=result.free_conflict_ratio,
        )
        self.measurements.append(measurement)
        self.get_logger().info(
            "Verified place pair r0:k%d <-> r1:k%d using %s: "
            "keyframe_T=(%.2f, %.2f, %.1fdeg) "
            "map_T=(%.2f, %.2f, %.1fdeg) descriptor=%.3f "
            "overlap=%.3f rmse=%.3f conflict=%.3f weight=%.3f"
            % (
                pair[0],
                pair[1],
                yaw_label,
                *result.transform[:2],
                math.degrees(result.transform[2]),
                *map0_from_map1[:2],
                math.degrees(map0_from_map1[2]),
                candidate.match.distance,
                result.symmetric_overlap,
                result.symmetric_rmse_m,
                result.free_conflict_ratio,
                measurement.weight,
            )
        )
        self._update_consensus(now_ns)

    def _match_new_keyframe(
        self, keyframe: OccupancyKeyframe, now_ns: int
    ) -> None:
        for candidate in self._descriptor_candidates(keyframe):
            self._process_candidate(candidate, now_ns)
            if (
                self.alignment_message is not None
                and self.stop_processing_after_lock
            ):
                break

    def _publish_status(self, now_ns: int) -> None:
        if now_ns - self.last_status_publish_ns < int(
            self.status_publish_period_sec * 1e9
        ):
            return
        self.last_status_publish_ns = now_ns
        consensus = self.last_consensus
        alignment = None
        if self.alignment_message is not None:
            alignment = {
                "x": self.alignment_message.transform.translation.x,
                "y": self.alignment_message.transform.translation.y,
                "yaw_deg": math.degrees(
                    self._yaw_from_quaternion(
                        self.alignment_message.transform.rotation
                    )
                ),
            }
        payload = {
            "state": self.state,
            "keyframes": [len(self.keyframes[0]), len(self.keyframes[1])],
            "attempted_pairs": len(self.attempted_pairs),
            "verified_measurements": len(self.measurements),
            "alignment": alignment,
            "consensus": (
                None
                if consensus is None
                else {
                    "measurements": consensus.measurement_count,
                    "target_keyframes": consensus.target_keyframe_count,
                    "source_keyframes": consensus.source_keyframe_count,
                    "translation_rms_m": consensus.translation_rms_m,
                    "yaw_rms_deg": math.degrees(consensus.yaw_rms_rad),
                    "accepted": consensus.accepted,
                }
            ),
            "last_match": self.last_match_summary,
        }
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self.status_publisher.publish(message)

    def _processing_timer(self) -> None:
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        if self.alignment_message is not None:
            message = deepcopy(self.alignment_message)
            message.header.stamp = now.to_msg()
            self.alignment_publisher.publish(message)
            if self.stop_processing_after_lock and self.lock_after_consensus:
                self.state = "LOCKED"
                self._publish_status(now_ns)
                return

        if not self._inputs_ready(now_ns):
            self._publish_status(now_ns)
            return
        for robot_id in (0, 1):
            keyframe = self._create_keyframe(robot_id, now_ns)
            if keyframe is not None:
                self._match_new_keyframe(keyframe, now_ns)
            if (
                self.alignment_message is not None
                and self.stop_processing_after_lock
            ):
                break
        self._update_consensus(now_ns)
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
