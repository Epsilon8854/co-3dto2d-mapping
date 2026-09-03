#!/usr/bin/env python3
"""Odom-aware startup registration for two live robots.

The cropped XYZ ICP implementation estimates ``r0/base_link <- r1/base_link``.
A merged occupancy map, however, needs ``r0/odom <- r1/odom`` because each
robot's persistent grid is expressed in its own odometry frame.  This wrapper
samples the mapper's planar odometry while collecting the startup clouds and
composes the accepted base-to-base registration with both odometry poses before
publishing the existing alignment topic.
"""

from __future__ import annotations

from collections import deque
import math
from typing import Deque, List, Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

from co_3dto2d_mapping.alignment_utils import (
    angular_distance,
    mean_planar_candidate,
)
from co_3dto2d_mapping.cropped_xyz_initial_icp_alignment import (
    CroppedXyzInitialIcpAlignment,
)
from co_3dto2d_mapping.initial_xy_icp_alignment import InitialXyIcpAlignment
from co_3dto2d_mapping.planar_transform_utils import (
    PlanarTransform,
    invert_planar,
    world_from_source_odom,
)
from co_3dto2d_mapping.pointcloud_registration import (
    invert_transform,
    rotation_rpy,
)


StampedPlanarPose = Tuple[int, PlanarTransform]


class OdomAwareCroppedXyzAlignment(CroppedXyzInitialIcpAlignment):
    """Publish an odom-frame alignment instead of a raw base-frame match."""

    def __init__(self) -> None:
        # These members must exist before the inherited cloud subscriptions can
        # dispatch to this class after construction and spinning begin.
        self._odom_histories: Tuple[
            Deque[StampedPlanarPose], Deque[StampedPlanarPose]
        ] = (deque(), deque())
        self._submap_odom_poses: Tuple[
            List[PlanarTransform], List[PlanarTransform]
        ] = ([], [])
        self._submap_reference_odom: List[Optional[PlanarTransform]] = [None, None]
        self._last_odom_wait_log_ns = [0, 0]
        self._last_motion_reset_log_ns = [0, 0]
        super().__init__()

        self.declare_parameter(
            "robot0_odom_topic", "/r0/toy/planar_odometry"
        )
        self.declare_parameter(
            "robot1_odom_topic", "/r1/toy/planar_odometry"
        )
        self.declare_parameter("odom_history_sec", 5.0)
        self.declare_parameter("max_odom_stamp_delta_sec", 0.25)
        self.declare_parameter("require_odom_compensation", True)
        self.declare_parameter("max_submap_motion_translation_m", 0.12)
        self.declare_parameter(
            "max_submap_motion_rotation_rad", math.radians(4.0)
        )

        self.robot_odom_topics = (
            str(self.get_parameter("robot0_odom_topic").value),
            str(self.get_parameter("robot1_odom_topic").value),
        )
        self.odom_history_sec = max(
            0.1, float(self.get_parameter("odom_history_sec").value)
        )
        self.max_odom_stamp_delta_sec = max(
            0.0,
            float(self.get_parameter("max_odom_stamp_delta_sec").value),
        )
        self.require_odom_compensation = bool(
            self.get_parameter("require_odom_compensation").value
        )
        self.max_submap_motion_translation_m = max(
            0.0,
            float(
                self.get_parameter("max_submap_motion_translation_m").value
            ),
        )
        self.max_submap_motion_rotation_rad = max(
            0.0,
            float(self.get_parameter("max_submap_motion_rotation_rad").value),
        )
        if not all(self.robot_odom_topics):
            raise ValueError("both robot odometry topics must be non-empty")

        self._odom_subscriptions = [
            self.create_subscription(
                Odometry,
                topic,
                lambda msg, rid=robot_index: self._odom_callback(msg, rid),
                50,
            )
            for robot_index, topic in enumerate(self.robot_odom_topics)
        ]
        self.get_logger().info(
            "Odom-aware startup alignment enabled: odom=(%s, %s) history=%.1fs "
            "max_stamp_delta=%.3fs max_stationary_motion=%.3fm/%.2fdeg "
            "required=%s. ICP base matches will be composed into %s <- %s."
            % (
                *self.robot_odom_topics,
                self.odom_history_sec,
                self.max_odom_stamp_delta_sec,
                self.max_submap_motion_translation_m,
                math.degrees(self.max_submap_motion_rotation_rad),
                "true" if self.require_odom_compensation else "false",
                str(self.get_parameter("target_frame_id").value),
                str(self.get_parameter("source_frame_id").value),
            )
        )

    @staticmethod
    def _stamp_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @staticmethod
    def _planar_pose_from_odom(msg: Odometry) -> PlanarTransform:
        pose = msg.pose.pose
        orientation = pose.orientation
        yaw = math.atan2(
            2.0
            * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0
            - 2.0
            * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        return float(pose.position.x), float(pose.position.y), float(yaw)

    def _odom_callback(self, msg: Odometry, robot_index: int) -> None:
        if msg.pose.covariance[0] > 1000.0:
            return
        stamp_ns = self._stamp_ns(msg.header.stamp)
        if stamp_ns <= 0:
            return
        history = self._odom_histories[robot_index]
        sample = (stamp_ns, self._planar_pose_from_odom(msg))
        if history and stamp_ns < history[-1][0]:
            # A bag restart or a remote clock reset invalidates nearest-sample
            # assumptions. Start a clean history rather than mixing epochs.
            history.clear()
        history.append(sample)
        cutoff_ns = stamp_ns - int(self.odom_history_sec * 1e9)
        while history and history[0][0] < cutoff_ns:
            history.popleft()

    def _nearest_odom_pose(
        self, robot_index: int, stamp
    ) -> Optional[PlanarTransform]:
        history = self._odom_histories[robot_index]
        if not history:
            return None
        stamp_ns = self._stamp_ns(stamp)
        if stamp_ns <= 0:
            return history[-1][1]
        nearest_stamp_ns, nearest_pose = min(
            history, key=lambda sample: abs(sample[0] - stamp_ns)
        )
        if (
            self.max_odom_stamp_delta_sec > 0.0
            and abs(nearest_stamp_ns - stamp_ns)
            > int(self.max_odom_stamp_delta_sec * 1e9)
        ):
            return None
        return nearest_pose

    def _throttled_wait_for_odom(self, robot_index: int) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_odom_wait_log_ns[robot_index] < 2_000_000_000:
            return
        self._last_odom_wait_log_ns[robot_index] = now_ns
        self.get_logger().warn(
            "Waiting for time-matched planar odometry on %s before caching r%d "
            "startup alignment clouds."
            % (self.robot_odom_topics[robot_index], robot_index)
        )

    def _submap_motion_exceeded(
        self, robot_index: int, pose: PlanarTransform
    ) -> bool:
        poses = self._submap_odom_poses[robot_index]
        if not poses:
            return False
        reference = poses[0]
        translation = math.hypot(pose[0] - reference[0], pose[1] - reference[1])
        rotation = angular_distance(pose[2], reference[2])
        return (
            translation > self.max_submap_motion_translation_m
            or rotation > self.max_submap_motion_rotation_rad
        )

    def _reset_robot_submap(self, robot_index: int, frames: List[np.ndarray]) -> None:
        frames.clear()
        self._submap_odom_poses[robot_index].clear()
        self._submap_reference_odom[robot_index] = None
        if robot_index == 0:
            self.robot0_points = None
        else:
            self.robot1_points = None

    def _cache_robot_cloud(
        self,
        robot_index: int,
        robot_name: str,
        frames: List[np.ndarray],
        msg: PointCloud2,
    ) -> None:
        self._mark_input_seen(robot_index)
        points_ready = self.robot0_points if robot_index == 0 else self.robot1_points
        if (
            not self._startup_delay_elapsed()
            or not self._cloud_collection_enabled()
            or points_ready is not None
        ):
            return

        odom_pose = self._nearest_odom_pose(robot_index, msg.header.stamp)
        if odom_pose is None and self.require_odom_compensation:
            self._throttled_wait_for_odom(robot_index)
            return
        if odom_pose is None:
            odom_pose = (0.0, 0.0, 0.0)

        if self._submap_motion_exceeded(robot_index, odom_pose):
            self._reset_robot_submap(robot_index, frames)
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_motion_reset_log_ns[robot_index] >= 2_000_000_000:
                self._last_motion_reset_log_ns[robot_index] = now_ns
                self.get_logger().warn(
                    "Reset r%d startup ICP submap because its planar odometry "
                    "moved beyond %.3fm/%.2fdeg while frames were collected. "
                    "Keep both robots stationary until alignment locks."
                    % (
                        robot_index,
                        self.max_submap_motion_translation_m,
                        math.degrees(self.max_submap_motion_rotation_rad),
                    )
                )

        points = self._cloud_to_xyz_points(robot_index, msg)
        if points is None:
            return
        frames.append(points)
        self._submap_odom_poses[robot_index].append(odom_pose)
        self.get_logger().info(
            "Cached %s odom-aware cropped XYZ ICP frame %d/%d with %d points."
            % (robot_name, len(frames), self.frame_count, len(points))
        )
        if len(frames) < self.frame_count:
            return

        non_empty = [frame for frame in frames if len(frame) > 0]
        merged = np.vstack(non_empty) if non_empty else np.empty((0, 3))
        merged = self._prepare_icp_points(merged)
        self._submap_reference_odom[robot_index] = mean_planar_candidate(
            self._submap_odom_poses[robot_index]
        )
        self.get_logger().info(
            "Built %s odom-aware cropped XYZ ICP submap from %d frames with "
            "%d points; representative odom=(%.3f, %.3f, %.2fdeg)."
            % (
                robot_name,
                len(frames),
                len(merged),
                self._submap_reference_odom[robot_index][0],
                self._submap_reference_odom[robot_index][1],
                math.degrees(self._submap_reference_odom[robot_index][2]),
            )
        )
        if robot_index == 0:
            self.robot0_points = merged
        else:
            self.robot1_points = merged
        self._try_compute_alignment(msg.header.stamp)

    def _reset_cloud_samples(self) -> None:
        super()._reset_cloud_samples()
        self._submap_odom_poses[0].clear()
        self._submap_odom_poses[1].clear()
        self._submap_reference_odom = [None, None]

    @staticmethod
    def _planar_from_result(result, invert_result: bool) -> PlanarTransform:
        rotation = result[0]
        translation = result[1]
        if rotation.shape == (3, 3):
            if invert_result:
                rotation, translation = invert_transform(rotation, translation)
            _, _, yaw = rotation_rpy(rotation)
            return float(translation[0]), float(translation[1]), float(yaw)

        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        transform = (float(translation[0]), float(translation[1]), float(yaw))
        return invert_planar(transform) if invert_result else transform

    @staticmethod
    def _synthetic_planar_result(result, transform: PlanarTransform, invert: bool):
        # InitialXyIcpAlignment applies ``invert_result`` itself. Feed it the
        # pre-inverted equivalent so that its final publication equals the
        # odom-compensated transform.
        raw_transform = invert_planar(transform) if invert else transform
        x, y, yaw = raw_transform
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        rotation = np.asarray(
            [[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]], dtype=np.float64
        )
        translation = np.asarray([x, y], dtype=np.float64)
        return (
            rotation,
            translation,
            result[2],
            result[3],
            result[4],
            "%s/odom_compensated" % result[5],
        )

    def _set_alignment_from_result(self, result, stamp, label: str) -> str:
        if self.input_mode != "cloud_initial" or result is None:
            return super()._set_alignment_from_result(result, stamp, label)
        if result[0].shape == (3, 3) and not self._transform_allowed(result[0]):
            return super()._set_alignment_from_result(result, stamp, label)

        target_odom_from_target_base = self._submap_reference_odom[0]
        source_odom_from_source_base = self._submap_reference_odom[1]
        if (
            target_odom_from_target_base is None
            or source_odom_from_source_base is None
        ):
            self.pending_candidates.clear()
            self.get_logger().warn(
                "%s ICP result rejected because one or both time-matched odometry "
                "snapshots are unavailable; it will be retried." % label
            )
            return "rejected"

        target_base_from_source_base = self._planar_from_result(
            result, self.invert_result
        )
        target_odom_from_source_odom = world_from_source_odom(
            target_odom_from_target_base,
            target_base_from_source_base,
            source_odom_from_source_base,
        )
        self.get_logger().info(
            "%s frame composition: r0_odom_T_r0_base=(%.3f, %.3f, %.2fdeg) "
            "r0_base_T_r1_base=(%.3f, %.3f, %.2fdeg) "
            "r1_odom_T_r1_base=(%.3f, %.3f, %.2fdeg) => "
            "%s_T_r1_odom=(%.3f, %.3f, %.2fdeg)"
            % (
                label,
                target_odom_from_target_base[0],
                target_odom_from_target_base[1],
                math.degrees(target_odom_from_target_base[2]),
                target_base_from_source_base[0],
                target_base_from_source_base[1],
                math.degrees(target_base_from_source_base[2]),
                source_odom_from_source_base[0],
                source_odom_from_source_base[1],
                math.degrees(source_odom_from_source_base[2]),
                str(self.get_parameter("target_frame_id").value),
                target_odom_from_source_odom[0],
                target_odom_from_source_odom[1],
                math.degrees(target_odom_from_source_odom[2]),
            )
        )

        synthetic = self._synthetic_planar_result(
            result, target_odom_from_source_odom, self.invert_result
        )
        return InitialXyIcpAlignment._set_alignment_from_result(
            self, synthetic, stamp, label + " odom-compensated"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomAwareCroppedXyzAlignment()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
