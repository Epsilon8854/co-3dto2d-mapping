#!/usr/bin/env python3
"""Stable common-world record output for the two-robot live mapper.

This wrapper keeps the ground-fused odometry selection and temporal occupancy
fusion from the existing implementation, but publishes recorded robot odometry
and occupancy grids directly in the common/world frame. It also latches the
first accepted inter-robot alignment by default so a late noisy estimate cannot
repeatedly reset and visually blink the merged map.
"""

from __future__ import annotations

from copy import deepcopy
import math
from typing import Optional, Set, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry

from co_3dto2d_mapping.alignment_utils import angular_distance
from co_3dto2d_mapping.record_republisher_ground_fused import (
    GroundFusedTemporalRecordRepublisher,
)


PlanarAlignment = Tuple[float, float, float]


def _multiply_quaternion(left: Quaternion, right: Quaternion) -> Quaternion:
    output = Quaternion()
    output.x = (
        left.w * right.x
        + left.x * right.w
        + left.y * right.z
        - left.z * right.y
    )
    output.y = (
        left.w * right.y
        - left.x * right.z
        + left.y * right.w
        + left.z * right.x
    )
    output.z = (
        left.w * right.z
        + left.x * right.y
        - left.y * right.x
        + left.z * right.w
    )
    output.w = (
        left.w * right.w
        - left.x * right.x
        - left.y * right.y
        - left.z * right.z
    )
    norm = math.sqrt(
        output.x * output.x
        + output.y * output.y
        + output.z * output.z
        + output.w * output.w
    )
    if norm > 1e-12:
        output.x /= norm
        output.y /= norm
        output.z /= norm
        output.w /= norm
    else:
        output.x = 0.0
        output.y = 0.0
        output.z = 0.0
        output.w = 1.0
    return output


def transform_grid_to_common_frame(
    msg: OccupancyGrid,
    alignment: PlanarAlignment,
    common_frame_id: str,
    stamp=None,
) -> OccupancyGrid:
    """Rigidly move an occupancy-grid pose into ``common_frame_id``.

    The raster values do not need resampling: an OccupancyGrid already stores a
    full origin pose. Composing that origin with ``common <- robot/odom`` keeps
    all odometry-applied cells intact while making the published frame contract
    self-contained for RViz, map saving, and consumers that do not resolve TF.
    """

    output = deepcopy(msg)
    if stamp is not None:
        output.header.stamp = stamp
    output.header.frame_id = str(common_frame_id)

    align_x, align_y, align_yaw = alignment
    cosine = math.cos(align_yaw)
    sine = math.sin(align_yaw)
    origin = msg.info.origin
    output.info.origin.position.x = (
        align_x
        + cosine * float(origin.position.x)
        - sine * float(origin.position.y)
    )
    output.info.origin.position.y = (
        align_y
        + sine * float(origin.position.x)
        + cosine * float(origin.position.y)
    )
    output.info.origin.position.z = float(origin.position.z)

    yaw_quaternion = Quaternion()
    yaw_quaternion.z = math.sin(0.5 * align_yaw)
    yaw_quaternion.w = math.cos(0.5 * align_yaw)
    output.info.origin.orientation = _multiply_quaternion(
        yaw_quaternion,
        origin.orientation,
    )
    return output


class WorldFrameGroundFusedRecordRepublisher(
    GroundFusedTemporalRecordRepublisher
):
    """Republish selected odometry and maps in the shared world frame."""

    def __init__(self) -> None:
        self._world_odom_logged: Set[int] = set()
        self._world_map_logged: Set[int] = set()
        self._last_latched_alignment_warning_ns = 0
        super().__init__()

        self.declare_parameter("publish_world_odometry", True)
        self.declare_parameter("suppress_unaligned_world_odometry", True)
        self.declare_parameter("publish_world_maps", True)
        self.declare_parameter("suppress_unaligned_world_maps", True)
        self.declare_parameter("lock_world_alignment", True)
        self.declare_parameter("world_alignment_same_translation_m", 0.01)
        self.declare_parameter(
            "world_alignment_same_rotation_rad", math.radians(0.25)
        )
        self.publish_world_odometry = bool(
            self.get_parameter("publish_world_odometry").value
        )
        self.suppress_unaligned_world_odometry = bool(
            self.get_parameter("suppress_unaligned_world_odometry").value
        )
        self.publish_world_maps = bool(
            self.get_parameter("publish_world_maps").value
        )
        self.suppress_unaligned_world_maps = bool(
            self.get_parameter("suppress_unaligned_world_maps").value
        )
        self.lock_world_alignment = bool(
            self.get_parameter("lock_world_alignment").value
        )
        self.world_alignment_same_translation_m = max(
            0.0,
            float(
                self.get_parameter("world_alignment_same_translation_m").value
            ),
        )
        self.world_alignment_same_rotation_rad = max(
            0.0,
            float(
                self.get_parameter("world_alignment_same_rotation_rad").value
            ),
        )
        self.get_logger().info(
            "World-frame record output: odometry=%s maps=%s common_frame=%s "
            "suppress_unaligned_odom=%s suppress_unaligned_maps=%s "
            "lock_alignment=%s"
            % (
                "true" if self.publish_world_odometry else "false",
                "true" if self.publish_world_maps else "false",
                self.common_frame_id,
                "true" if self.suppress_unaligned_world_odometry else "false",
                "true" if self.suppress_unaligned_world_maps else "false",
                "true" if self.lock_world_alignment else "false",
            )
        )

    @staticmethod
    def _candidate_from_alignment(msg: TransformStamped) -> PlanarAlignment:
        rotation = msg.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        return (
            float(msg.transform.translation.x),
            float(msg.transform.translation.y),
            float(yaw),
        )

    def alignment_callback(self, msg: TransformStamped) -> None:
        candidate = self._candidate_from_alignment(msg)
        if self.lock_world_alignment and self.alignment is not None:
            translation_delta = math.hypot(
                candidate[0] - self.alignment[0],
                candidate[1] - self.alignment[1],
            )
            rotation_delta = angular_distance(candidate[2], self.alignment[2])
            if (
                translation_delta > self.world_alignment_same_translation_m
                or rotation_delta > self.world_alignment_same_rotation_rad
            ):
                now_ns = self.get_clock().now().nanoseconds
                if (
                    now_ns - self._last_latched_alignment_warning_ns
                    >= 2_000_000_000
                ):
                    self._last_latched_alignment_warning_ns = now_ns
                    self.get_logger().warn(
                        "Ignoring changed inter-robot alignment %.3fm/%.2fdeg "
                        "after the world transform was latched. Set "
                        "lock_world_alignment:=false to permit live updates."
                        % (translation_delta, math.degrees(rotation_delta))
                    )
            # Repeated or slightly different samples are intentionally ignored:
            # the stored TransformStamped is re-stamped by publish_transforms().
            return
        super().alignment_callback(msg)

    @staticmethod
    def _rotate_pose_covariance(covariance, yaw: float):
        values = np.asarray(covariance, dtype=np.float64)
        if values.size != 36 or not np.all(np.isfinite(values)):
            return list(covariance)
        matrix = values.reshape((6, 6))
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        rotation = np.asarray(
            [
                [cos_yaw, -sin_yaw, 0.0],
                [sin_yaw, cos_yaw, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        jacobian = np.zeros((6, 6), dtype=np.float64)
        jacobian[:3, :3] = rotation
        jacobian[3:, 3:] = rotation
        return (jacobian @ matrix @ jacobian.T).reshape(-1).tolist()

    def _world_odom(
        self, msg: Odometry, stamp, robot_id: int
    ) -> Optional[Odometry]:
        output = super().copy_odom(msg, stamp, robot_id)
        if not self.publish_world_odometry:
            return output

        if robot_id == 0:
            output.header.frame_id = self.common_frame_id
            alignment = (0.0, 0.0, 0.0)
        else:
            if self.alignment is None:
                if self.suppress_unaligned_world_odometry:
                    return None
                return output
            alignment = self.alignment
            output.header.frame_id = self.common_frame_id

        x_align, y_align, yaw_align = alignment
        pose = msg.pose.pose
        cos_yaw = math.cos(yaw_align)
        sin_yaw = math.sin(yaw_align)
        output.pose.pose.position.x = (
            x_align
            + cos_yaw * pose.position.x
            - sin_yaw * pose.position.y
        )
        output.pose.pose.position.y = (
            y_align
            + sin_yaw * pose.position.x
            + cos_yaw * pose.position.y
        )
        output.pose.pose.position.z = pose.position.z
        output.pose.pose.orientation = self.multiply_quaternion(
            self.quaternion_from_yaw(yaw_align), pose.orientation
        )
        output.pose.covariance = self._rotate_pose_covariance(
            msg.pose.covariance, yaw_align
        )

        if robot_id not in self._world_odom_logged:
            self._world_odom_logged.add(robot_id)
            self.get_logger().info(
                "Publishing %s/r%d/odom directly in common frame %s."
                % (self.output_prefix, robot_id, self.common_frame_id)
            )
        return output

    def _world_grid(
        self,
        msg: OccupancyGrid,
        stamp,
        robot_id: int,
    ) -> Optional[OccupancyGrid]:
        if not self.publish_world_maps:
            return self.copy_grid(msg, stamp, robot_id)

        if robot_id == 0:
            alignment = (0.0, 0.0, 0.0)
        elif self.alignment is None:
            if self.suppress_unaligned_world_maps:
                return None
            return self.copy_grid(msg, stamp, robot_id)
        else:
            alignment = self.alignment

        output = transform_grid_to_common_frame(
            msg,
            alignment,
            self.common_frame_id,
            stamp,
        )
        if robot_id not in self._world_map_logged:
            self._world_map_logged.add(robot_id)
            self.get_logger().info(
                "Publishing %s/r%d occupancy grids directly in common frame %s."
                % (self.output_prefix, robot_id, self.common_frame_id)
            )
        return output

    def publish_outputs(self) -> None:
        # Keep the most recent ground-fused pose selected before creating both
        # the world-frame odometry messages and odom->base TF transforms.
        self._refresh_odometry_selection()
        stamp = self.get_clock().now().to_msg()
        for robot_id, msg in self.latest_odom.items():
            output = self._world_odom(msg, stamp, robot_id)
            if output is not None:
                self.odom_pubs[robot_id].publish(output)

        for robot_id, msg in self.latest_local_maps.items():
            output = self._world_grid(msg, stamp, robot_id)
            if output is not None:
                self.local_map_pubs[robot_id].publish(output)
        for robot_id, msg in self.latest_global_maps.items():
            output = self._world_grid(msg, stamp, robot_id)
            if output is not None:
                self.global_map_pubs[robot_id].publish(output)
        for robot_id, msg in self.latest_slice_kept_points.items():
            self.slice_kept_points_pubs[robot_id].publish(
                self.copy_cloud(msg, stamp, robot_id)
            )
        for robot_id, msg in self.latest_slice_rejected_points.items():
            self.slice_rejected_points_pubs[robot_id].publish(
                self.copy_cloud(msg, stamp, robot_id)
            )

        if self.publish_tf:
            self.publish_transforms(stamp)

        merged = (
            self.build_merged_global(stamp)
            if self.publish_merged_global
            else None
        )
        if merged is not None and self.merged_global_pub is not None:
            self.merged_global_pub.publish(merged)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WorldFrameGroundFusedRecordRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
