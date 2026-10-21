#!/usr/bin/env python3
"""Gravity-referenced mapping cloud with optional observed-floor pose fusion.

The node synchronizes each raw cloud with the raw RTAB-Map odometry, estimates
an optional floor observation and publishes both outputs:

* a gravity-height-filtered cloud consumed by the occupancy mapper; and
* ``mapping/floor_based_odometry`` consumed by the same mapper.

This removes the former cloud-only callback and therefore eliminates duplicate
plane fitting for one scan. ``ground_plane_height_reference_mode`` selects an
observed floor plane or a virtual floor made from the IMU gravity direction and
a measured sensor height.

``ground_plane_pose_enabled`` is the projection ablation switch.  When false,
the filtered cloud is unchanged but raw odometry is passed through to the mapper.
"""

from __future__ import annotations

from collections import deque
from copy import deepcopy
import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

from co_3dto2d_mapping.gravity_plane_pose_fusion import GravityPlanePoseFusion
from co_3dto2d_mapping.gravity_plane_utils import (
    PlaneFitResult,
    blend_unit_vectors,
    estimate_gravity_constrained_plane,
    fixed_z_gravity_pose,
    pose_z_from_plane,
    quaternion_from_rpy,
    roll_pitch_from_up,
    yaw_from_quaternion,
)
from co_3dto2d_mapping.plane_height_filter import (
    filter_points_by_fixed_sensor_height,
    filter_points_by_plane_height,
)
from co_3dto2d_mapping.single_floor_plane import estimate_single_floor_plane


class GravityPlaneHeightCloudFusion(GravityPlanePoseFusion):
    def __init__(self) -> None:
        # The base class creates the raw-cloud/raw-odometry synchronizer and
        # registers this class's overridden _synchronized_callback.
        super().__init__()

        self.declare_parameter("ground_plane_height_filter_enabled", True)
        self.declare_parameter(
            "ground_plane_height_reference_mode", "estimated_plane"
        )
        self.declare_parameter("lidar_mount_height_above_floor_m", 0.30)
        self.declare_parameter(
            "ground_plane_filtered_cloud_topic",
            "mapping/plane_height_filtered",
        )
        self.declare_parameter("ground_plane_filter_min_height_m", 0.05)
        self.declare_parameter("ground_plane_filter_max_height_m", 1.00)
        self.declare_parameter("ground_plane_filtered_cloud_min_points", 20)
        self.declare_parameter("ground_plane_filtered_cloud_log_period_ms", 2000)
        self.declare_parameter("ground_plane_debug_cloud_enabled", False)
        self.declare_parameter("ground_plane_debug_paths_enabled", False)
        self.declare_parameter(
            "ground_plane_debug_cloud_topic", "mapping/debug/ground_plane_inliers"
        )
        self.declare_parameter(
            "ground_plane_debug_raw_path_topic",
            "mapping/debug/raw_odometry_path",
        )
        self.declare_parameter(
            "ground_plane_debug_fused_path_topic",
            "mapping/debug/floor_based_odometry_path",
        )
        self.declare_parameter("ground_plane_debug_path_max_poses", 10000)
        self.declare_parameter("ground_plane_debug_path_publish_period_ms", 250)

        self.declare_parameter("ground_plane_estimation_mode", "single_floor")
        self.declare_parameter("ground_plane_single_floor_bin_size_m", 0.025)
        self.declare_parameter(
            "ground_plane_single_floor_lowest_support_ratio", 0.55
        )
        self.declare_parameter(
            "ground_plane_single_floor_prior_tolerance_m", 0.20
        )

        value = lambda name: self.get_parameter(name).value
        self.height_filter_enabled = bool(
            value("ground_plane_height_filter_enabled")
        )
        self.height_reference_mode = str(
            value("ground_plane_height_reference_mode")
        )
        self.lidar_mount_height_m = float(
            value("lidar_mount_height_above_floor_m")
        )
        self.filtered_cloud_topic = str(
            value("ground_plane_filtered_cloud_topic")
        )
        self.filter_min_height_m = float(
            value("ground_plane_filter_min_height_m")
        )
        self.filter_max_height_m = float(
            value("ground_plane_filter_max_height_m")
        )
        self.filtered_cloud_min_points = int(
            value("ground_plane_filtered_cloud_min_points")
        )
        self.filtered_cloud_log_period_ms = int(
            value("ground_plane_filtered_cloud_log_period_ms")
        )
        self.debug_cloud_enabled = bool(value("ground_plane_debug_cloud_enabled"))
        self.debug_paths_enabled = bool(value("ground_plane_debug_paths_enabled"))
        self.debug_cloud_topic = str(value("ground_plane_debug_cloud_topic"))
        self.debug_raw_path_topic = str(value("ground_plane_debug_raw_path_topic"))
        self.debug_fused_path_topic = str(
            value("ground_plane_debug_fused_path_topic")
        )
        self.debug_path_max_poses = int(value("ground_plane_debug_path_max_poses"))
        self.debug_path_publish_period_ms = int(
            value("ground_plane_debug_path_publish_period_ms")
        )
        self.estimation_mode = str(value("ground_plane_estimation_mode"))
        self.single_floor_bin_size_m = float(
            value("ground_plane_single_floor_bin_size_m")
        )
        self.single_floor_lowest_support_ratio = float(
            value("ground_plane_single_floor_lowest_support_ratio")
        )
        self.single_floor_prior_tolerance_m = float(
            value("ground_plane_single_floor_prior_tolerance_m")
        )
        self._validate_wrapper_parameters()

        self._height_cloud_publisher = self.create_publisher(
            PointCloud2,
            self.filtered_cloud_topic,
            qos_profile_sensor_data,
        )
        self._debug_cloud_publisher = (
            self.create_publisher(
                PointCloud2, self.debug_cloud_topic, qos_profile_sensor_data
            )
            if self.debug_cloud_enabled
            else None
        )
        self._raw_path_publisher = (
            self.create_publisher(Path, self.debug_raw_path_topic, 10)
            if self.debug_paths_enabled
            else None
        )
        self._fused_path_publisher = (
            self.create_publisher(Path, self.debug_fused_path_topic, 10)
            if self.debug_paths_enabled
            else None
        )
        self._raw_path_poses = deque(maxlen=self.debug_path_max_poses)
        self._fused_path_poses = deque(maxlen=self.debug_path_max_poses)
        self._last_debug_path_stamp_ns: Optional[int] = None
        self._last_debug_path_publish_ns = 0
        self._height_filter_last_log_ns = 0
        self._invalid_odometry_warning_active = False

        self.get_logger().info(
            "Gravity-height mapping started. cloud=%s odom_in=%s cloud_out=%s "
            "odom_out=%s reference=%s lidar_mount_height=%.3fm pose_reference=%s estimator=%s "
            "pose_to_mapping=%s height=[%.3f, %.3f]m debug_cloud=%s paths=(%s, %s)"
            % (
                self.pointcloud_topic,
                self.planar_odometry_topic,
                self.filtered_cloud_topic,
                self.output_odometry_topic,
                self.height_reference_mode,
                self.lidar_mount_height_m,
                self.pose_reference_mode,
                self.estimation_mode,
                "true" if self.enabled else "false",
                self.filter_min_height_m,
                self.filter_max_height_m,
                self.debug_cloud_topic if self.debug_cloud_enabled else "disabled",
                self.debug_raw_path_topic,
                self.debug_fused_path_topic,
            )
        )

    def _validate_wrapper_parameters(self) -> None:
        if not self.filtered_cloud_topic:
            raise ValueError("ground_plane_filtered_cloud_topic must not be empty")
        if self.filter_min_height_m < 0.0:
            raise ValueError("ground_plane_filter_min_height_m must be non-negative")
        if self.filter_max_height_m <= self.filter_min_height_m:
            raise ValueError(
                "ground_plane_filter_max_height_m must exceed the minimum"
            )
        if self.height_reference_mode not in {
            "estimated_plane",
            "fixed_sensor_height",
        }:
            raise ValueError(
                "ground_plane_height_reference_mode must be estimated_plane or "
                "fixed_sensor_height"
            )
        if (
            not math.isfinite(self.lidar_mount_height_m)
            or self.lidar_mount_height_m <= 0.0
        ):
            raise ValueError(
                "lidar_mount_height_above_floor_m must be positive and finite"
            )
        if (
            self.height_reference_mode == "fixed_sensor_height"
            and self.enabled
            and self.pose_reference_mode != "fixed_height_imu_gravity"
        ):
            raise ValueError(
                "fixed_sensor_height with pose correction requires "
                "ground_plane_pose_reference_mode=fixed_height_imu_gravity"
            )
        if self.filtered_cloud_min_points < 1:
            raise ValueError(
                "ground_plane_filtered_cloud_min_points must be positive"
            )
        if self.filtered_cloud_log_period_ms < 1:
            raise ValueError(
                "ground_plane_filtered_cloud_log_period_ms must be positive"
            )
        if self.debug_cloud_enabled and not self.debug_cloud_topic:
            raise ValueError("ground_plane_debug_cloud_topic must not be empty")
        if not self.debug_raw_path_topic or not self.debug_fused_path_topic:
            raise ValueError("ground-plane debug path topics must not be empty")
        if self.debug_path_max_poses < 2:
            raise ValueError("ground_plane_debug_path_max_poses must be at least 2")
        if self.debug_path_publish_period_ms < 1:
            raise ValueError(
                "ground_plane_debug_path_publish_period_ms must be positive"
            )
        if self.estimation_mode not in {"single_floor", "ransac"}:
            raise ValueError(
                "ground_plane_estimation_mode must be single_floor or ransac"
            )
        if (
            not math.isfinite(self.single_floor_bin_size_m)
            or self.single_floor_bin_size_m <= 0.0
        ):
            raise ValueError(
                "ground_plane_single_floor_bin_size_m must be positive and finite"
            )
        if not 0.0 < self.single_floor_lowest_support_ratio <= 1.0:
            raise ValueError(
                "ground_plane_single_floor_lowest_support_ratio must be in (0, 1]"
            )
        if (
            not math.isfinite(self.single_floor_prior_tolerance_m)
            or self.single_floor_prior_tolerance_m < 0.0
        ):
            raise ValueError(
                "ground_plane_single_floor_prior_tolerance_m must be finite and "
                "non-negative"
            )

    def _accept_or_hold_plane(
        self,
        result: Optional[PlaneFitResult],
        stamp_ns: int,
        input_pose_z_m: float,
    ) -> Tuple[Optional[np.ndarray], Optional[float], str]:
        # rosbag replay can restart or seek to an earlier epoch. A negative
        # timestamp delta must never satisfy the parent's hold-time condition,
        # otherwise a stale floor from the previous replay can leak into the
        # new map. Reset before delegating to the common temporal gate.
        if (
            self._last_plane_stamp_ns is not None
            and stamp_ns > 0
            and stamp_ns < self._last_plane_stamp_ns
        ):
            self.get_logger().warn(
                "Resetting shared floor state after a non-monotonic cloud timestamp."
            )
            self._clear_plane_state()
        return super()._accept_or_hold_plane(
            result,
            stamp_ns,
            input_pose_z_m,
        )

    def _accept_or_hold_gravity(
        self, up: Optional[np.ndarray], stamp_ns: int
    ) -> Tuple[Optional[np.ndarray], str]:
        """Smooth IMU world-up without creating a cloud-plane observation."""

        if (
            self._last_plane_stamp_ns is not None
            and stamp_ns > 0
            and stamp_ns < self._last_plane_stamp_ns
        ):
            self._clear_plane_state()

        if up is not None:
            if self._filtered_normal is None:
                self._filtered_normal = np.asarray(up, dtype=np.float64)
                state = "gravity_initialized"
            else:
                self._filtered_normal = blend_unit_vectors(
                    self._filtered_normal, up, self.filter_gain
                )
                state = "gravity_accepted"
            self._last_plane_stamp_ns = stamp_ns
            return self._filtered_normal, state

        if (
            self._filtered_normal is not None
            and self._last_plane_stamp_ns is not None
            and stamp_ns - self._last_plane_stamp_ns
            <= int(self.hold_timeout_sec * 1e9)
        ):
            return self._filtered_normal, "gravity_held"
        return None, "gravity_unavailable"

    def _fit_current_floor(
        self,
        local_points: np.ndarray,
        up: Optional[np.ndarray],
    ) -> Tuple[Optional[PlaneFitResult], int]:
        if up is None or len(local_points) == 0:
            return None, 0
        candidates = self._candidate_points(local_points, up)
        candidate_count = int(len(candidates))
        if candidate_count < self.plane_config.min_inliers:
            return None, candidate_count

        if self.estimation_mode == "single_floor":
            result = estimate_single_floor_plane(
                candidates,
                up,
                self.plane_config,
                prior_height_m=self._filtered_height_m,
                prior_tolerance_m=self.single_floor_prior_tolerance_m,
                bin_size_m=self.single_floor_bin_size_m,
                lowest_support_ratio=self.single_floor_lowest_support_ratio,
            )
        else:
            result = estimate_gravity_constrained_plane(
                candidates,
                up,
                self.plane_config,
            )
        return result, candidate_count

    @staticmethod
    def _xyz_cloud(
        points_xyz: np.ndarray,
        stamp,
        frame_id: str,
    ) -> PointCloud2:
        points = np.ascontiguousarray(points_xyz, dtype=np.float32)
        message = PointCloud2()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.height = 1
        message.width = int(len(points))
        message.fields = [
            PointField(
                name="x",
                offset=0,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name="y",
                offset=4,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name="z",
                offset=8,
                datatype=PointField.FLOAT32,
                count=1,
            ),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = message.point_step * message.width
        message.data = points.tobytes(order="C")
        message.is_dense = True
        return message

    @staticmethod
    def _colored_xyz_cloud(
        points_xyz: np.ndarray,
        stamp,
        frame_id: str,
        rgb_value: int,
    ) -> PointCloud2:
        points = np.ascontiguousarray(points_xyz, dtype=np.float32)
        packed_rgb = np.full(len(points), rgb_value, dtype=np.uint32).view(np.float32)
        data = np.empty((len(points), 4), dtype=np.float32)
        if len(points):
            data[:, :3] = points
            data[:, 3] = packed_rgb

        message = PointCloud2()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.height = 1
        message.width = int(len(points))
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 16
        message.row_step = message.point_step * message.width
        message.data = data.tobytes(order="C")
        message.is_dense = True
        return message

    def _publish_ground_plane_debug_cloud(
        self,
        cloud: PointCloud2,
        local_points: Optional[np.ndarray],
        normal: Optional[np.ndarray],
        height_m: Optional[float],
        up: Optional[np.ndarray],
        state: str,
    ) -> None:
        publisher = self._debug_cloud_publisher
        if publisher is None:
            return

        if (
            self.pose_reference_mode == "fixed_height_imu_gravity"
            or self.height_reference_mode == "fixed_sensor_height"
        ):
            publisher.publish(
                self._colored_xyz_cloud(
                    np.empty((0, 3), dtype=np.float64),
                    cloud.header.stamp,
                    self.local_frame_id,
                    0x0000FF00,
                )
            )
            return

        selected = np.empty((0, 3), dtype=np.float64)
        if (
            local_points is not None
            and len(local_points)
            and normal is not None
            and height_m is not None
        ):
            candidate_up = up if up is not None else normal
            vertical = local_points @ candidate_up
            tangent = local_points - vertical[:, None] * candidate_up[None, :]
            radial = np.linalg.norm(tangent, axis=1)
            candidate_mask = (
                (radial >= self.candidate_min_range_m)
                & (radial <= self.candidate_max_range_m)
                & (vertical >= self.candidate_height_min_m)
                & (vertical <= self.candidate_height_max_m)
            )
            if self.candidate_center_exclusion_m > 0.0:
                candidate_mask &= radial >= self.candidate_center_exclusion_m
            residual = np.abs(local_points @ normal + height_m)
            selected = local_points[
                candidate_mask
                & (residual <= self.plane_config.distance_threshold_m)
            ]

        # Green marks a fit accepted for this scan; amber marks points shown
        # against a temporarily held plane after a rejected or failed fit.
        rgb_value = 0x00FFB000 if state.startswith("held_") else 0x0000FF00
        publisher.publish(
            self._colored_xyz_cloud(
                selected,
                cloud.header.stamp,
                self.local_frame_id,
                rgb_value,
            )
        )

    def _append_path_pose(self, history, odometry: Odometry) -> None:
        pose = PoseStamped()
        pose.header = deepcopy(odometry.header)
        pose.pose = deepcopy(odometry.pose.pose)
        history.append(pose)

    def _publish_debug_paths(
        self,
        raw_odometry: Odometry,
        fused_odometry: Odometry,
        stamp_ns: int,
    ) -> None:
        if (
            self._last_debug_path_stamp_ns is not None
            and stamp_ns < self._last_debug_path_stamp_ns
        ):
            self._raw_path_poses.clear()
            self._fused_path_poses.clear()
            self._last_debug_path_publish_ns = 0
        self._last_debug_path_stamp_ns = stamp_ns
        if self._raw_path_publisher is None or self._fused_path_publisher is None:
            return
        self._append_path_pose(self._raw_path_poses, raw_odometry)
        self._append_path_pose(self._fused_path_poses, fused_odometry)

        if (
            stamp_ns - self._last_debug_path_publish_ns
            < self.debug_path_publish_period_ms * 1_000_000
            and self._last_debug_path_publish_ns != 0
        ):
            return

        raw_path = Path()
        raw_path.header = deepcopy(raw_odometry.header)
        raw_path.poses = list(self._raw_path_poses)
        fused_path = Path()
        fused_path.header = deepcopy(fused_odometry.header)
        fused_path.poses = list(self._fused_path_poses)
        self._raw_path_publisher.publish(raw_path)
        self._fused_path_publisher.publish(fused_path)
        self._last_debug_path_publish_ns = stamp_ns

    def _publish_mapping_cloud(
        self,
        cloud: PointCloud2,
        local_points: Optional[np.ndarray],
        normal: Optional[np.ndarray],
        height_m: Optional[float],
        state: str,
        candidate_count: int,
        result: Optional[PlaneFitResult],
    ) -> None:
        if local_points is None or len(local_points) == 0:
            self._log_height_filter(
                state="cloud_unavailable",
                input_count=0,
                output_count=0,
                candidate_count=candidate_count,
                result=result,
            )
            return

        if not self.height_filter_enabled:
            selected = local_points
        elif normal is None or height_m is None:
            self._log_height_filter(
                state=state,
                input_count=len(local_points),
                output_count=0,
                candidate_count=candidate_count,
                result=result,
            )
            return
        else:
            if self.height_reference_mode == "fixed_sensor_height":
                filtered = filter_points_by_fixed_sensor_height(
                    local_points,
                    normal,
                    self.lidar_mount_height_m,
                    self.filter_min_height_m,
                    self.filter_max_height_m,
                )
            else:
                filtered = filter_points_by_plane_height(
                    local_points,
                    normal,
                    height_m,
                    self.filter_min_height_m,
                    self.filter_max_height_m,
                )
            selected = filtered.points
            if len(selected) < self.filtered_cloud_min_points:
                self._log_height_filter(
                    state="too_few_filtered_points",
                    input_count=len(local_points),
                    output_count=len(selected),
                    candidate_count=candidate_count,
                    result=result,
                )
                return

        self._height_cloud_publisher.publish(
            self._xyz_cloud(
                selected,
                cloud.header.stamp,
                self.local_frame_id,
            )
        )
        self._log_height_filter(
            state=state if self.height_filter_enabled else "filter_disabled",
            input_count=len(local_points),
            output_count=len(selected),
            candidate_count=candidate_count,
            result=result,
        )

    def _fused_mapping_odometry(
        self,
        cloud: PointCloud2,
        input_odometry: Odometry,
        normal: Optional[np.ndarray],
        height_m: Optional[float],
        result: Optional[PlaneFitResult],
    ) -> Tuple[Odometry, Optional[float], Optional[float], Optional[float]]:
        output = deepcopy(input_odometry)
        output.header.stamp = cloud.header.stamp
        if not self.enabled:
            return output, None, None, None

        if self.pose_reference_mode == "fixed_height_imu_gravity":
            pose_z = self.fixed_base_link_height_m
            output.pose.pose.position.z = pose_z
            if normal is None:
                return output, None, None, pose_z
            orientation = input_odometry.pose.pose.orientation
            yaw = yaw_from_quaternion(
                (orientation.x, orientation.y, orientation.z, orientation.w)
            )
            pose_z, quaternion = fixed_z_gravity_pose(
                fixed_base_link_height_m=self.fixed_base_link_height_m,
                up_vector=normal,
                icp_yaw_rad=yaw,
            )
            roll, pitch = roll_pitch_from_up(normal)
            output.pose.pose.orientation.x = float(quaternion[0])
            output.pose.pose.orientation.y = float(quaternion[1])
            output.pose.pose.orientation.z = float(quaternion[2])
            output.pose.pose.orientation.w = float(quaternion[3])
            output.pose.covariance[14] = 0.0
            output.pose.covariance[21] = (
                self.orientation_stddev_rad * self.orientation_stddev_rad
            )
            output.pose.covariance[28] = (
                self.orientation_stddev_rad * self.orientation_stddev_rad
            )
            return output, roll, pitch, pose_z

        if normal is None or height_m is None:
            return output, None, None, None

        roll, pitch = roll_pitch_from_up(normal)
        orientation = input_odometry.pose.pose.orientation
        yaw = yaw_from_quaternion(
            (orientation.x, orientation.y, orientation.z, orientation.w)
        )
        quaternion = quaternion_from_rpy(roll, pitch, yaw)
        input_z = float(input_odometry.pose.pose.position.z)
        if not math.isfinite(input_z):
            input_z = 0.0
        pose_z = pose_z_from_plane(
            height_m,
            mode=self.z_mode,
            reference_z_m=self.reference_z_m,
            z_offset_m=self.z_offset_m,
            initial_height_m=self._initial_plane_height_m,
            initial_pose_z_m=(
                input_z if self.z_mode == "passthrough" else self._initial_pose_z_m
            ),
        )

        output.pose.pose.position.x = input_odometry.pose.pose.position.x
        output.pose.pose.position.y = input_odometry.pose.pose.position.y
        output.pose.pose.position.z = pose_z
        output.pose.pose.orientation.x = float(quaternion[0])
        output.pose.pose.orientation.y = float(quaternion[1])
        output.pose.pose.orientation.z = float(quaternion[2])
        output.pose.pose.orientation.w = float(quaternion[3])

        if result is not None:
            z_variance = max(
                self.z_stddev_min_m * self.z_stddev_min_m,
                result.rmse_m * result.rmse_m,
            )
            orientation_variance = (
                self.orientation_stddev_rad * self.orientation_stddev_rad
            )
            output.pose.covariance[14] = z_variance
            output.pose.covariance[21] = orientation_variance
            output.pose.covariance[28] = orientation_variance
        return output, roll, pitch, pose_z

    def _synchronized_callback(
        self,
        cloud: PointCloud2,
        input_odometry: Odometry,
    ) -> None:
        # This is the only floor-fitting callback. Both mapping outputs below
        # carry cloud.header.stamp and use the same accepted/held plane state.
        up = self._up_from_imu(cloud.header.stamp)
        # Cloud-to-local conversion is independent of IMU availability. Keeping
        # it available lets a short IMU dropout use the previously held floor
        # state for filtering instead of dropping an otherwise valid scan.
        local_points = self._cloud_to_local(cloud)
        input_z = float(input_odometry.pose.pose.position.z)
        stamp_ns = self._stamp_ns(cloud.header.stamp)
        if self.pose_reference_mode == "fixed_height_imu_gravity":
            # No cloud plane is fitted in this mode. The configured LiDAR mount
            # height serves only the cloud-height filter; odometry z is fixed
            # by fixed_base_link_height_above_floor_m.
            result = None
            candidate_count = 0
            normal, state = self._accept_or_hold_gravity(up, stamp_ns)
            height_m = self.lidar_mount_height_m if normal is not None else None
        elif self.height_reference_mode == "fixed_sensor_height":
            # The virtual plane lies one measured sensor height below the
            # local-frame origin. It has no LiDAR inliers by design.
            result = None
            candidate_count = 0
            normal = up
            height_m = self.lidar_mount_height_m if up is not None else None
            state = (
                "fixed_sensor_height"
                if up is not None
                else "gravity_unavailable"
            )
        else:
            points_for_fit = (
                local_points
                if local_points is not None
                else np.empty((0, 3), dtype=np.float64)
            )
            result, candidate_count = self._fit_current_floor(points_for_fit, up)
            normal, height_m, state = self._accept_or_hold_plane(
                result,
                stamp_ns,
                input_z,
            )

        self._publish_ground_plane_debug_cloud(
            cloud,
            local_points,
            normal,
            height_m,
            up,
            state,
        )

        self._publish_mapping_cloud(
            cloud,
            local_points,
            normal,
            height_m,
            state,
            candidate_count,
            result,
        )

        position = input_odometry.pose.pose.position
        orientation = input_odometry.pose.pose.orientation
        pose_values = np.asarray(
            [
                position.x,
                position.y,
                position.z,
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ],
            dtype=np.float64,
        )
        try:
            yaw_from_quaternion(
                (orientation.x, orientation.y, orientation.z, orientation.w)
            )
            odometry_pose_is_valid = bool(np.all(np.isfinite(pose_values)))
        except ValueError:
            odometry_pose_is_valid = False
        if not odometry_pose_is_valid:
            if not self._invalid_odometry_warning_active:
                self.get_logger().warning(
                    "Skipping invalid planar odometry pose; ground-plane and "
                    "height-filtered clouds continue to publish."
                )
            self._invalid_odometry_warning_active = True
            return
        self._invalid_odometry_warning_active = False

        output, roll, pitch, pose_z = self._fused_mapping_odometry(
            cloud,
            input_odometry,
            normal,
            height_m,
            result,
        )
        self._publisher.publish(output)
        self._publish_debug_paths(input_odometry, output, stamp_ns)
        if self.height_reference_mode == "estimated_plane":
            self._log_result(
                state if self.enabled else "pose_integration_disabled",
                candidate_count,
                result,
                roll=roll,
                pitch=pitch,
                pose_z=pose_z,
            )

    def _log_height_filter(
        self,
        *,
        state: str,
        input_count: int,
        output_count: int,
        candidate_count: int,
        result: Optional[PlaneFitResult],
    ) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if (
            now_ns - self._height_filter_last_log_ns
            < self.filtered_cloud_log_period_ms * 1_000_000
        ):
            return
        self._height_filter_last_log_ns = now_ns
        if self.height_reference_mode == "fixed_sensor_height":
            self.get_logger().info(
                "Gravity-height cloud %s: raw=%d output=%d floor_band=[%.2f, %.2f]m "
                "lidar_mount_height=%.2fm"
                % (
                    state,
                    input_count,
                    output_count,
                    self.filter_min_height_m,
                    self.filter_max_height_m,
                    self.lidar_mount_height_m,
                )
            )
            return
        if result is None:
            self.get_logger().warn(
                "Plane-height cloud %s: raw=%d candidates=%d output=%d; "
                "waiting for a valid shared floor state."
                % (state, input_count, candidate_count, output_count)
            )
            return
        self.get_logger().info(
            "Plane-height cloud %s: raw=%d candidates=%d output=%d "
            "band=[%.2f, %.2f]m floor_inliers=%d ratio=%.2f rmse=%.3fm"
            % (
                state,
                input_count,
                candidate_count,
                output_count,
                self.filter_min_height_m,
                self.filter_max_height_m,
                result.inlier_count,
                result.inlier_ratio,
                result.rmse_m,
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GravityPlaneHeightCloudFusion()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
