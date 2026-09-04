#!/usr/bin/env python3
"""Single-floor pose fusion and plane-relative mapping cloud from one fit.

The node synchronizes each raw cloud with the raw RTAB-Map odometry, estimates
one floor observation, and uses that exact accepted/held state for both outputs:

* a plane-height-filtered cloud consumed by the occupancy mapper; and
* ``mapping/floor_odometry`` consumed by the same mapper.

This removes the former cloud-only callback and therefore eliminates duplicate
plane fitting for one scan.  The default estimator assumes one approximately
horizontal floor: the IMU-derived up vector fixes the normal and a robust 1-D
mode search estimates only the floor offset.  Set
``ground_plane_estimation_mode:=ransac`` for the previous constrained RANSAC.

``ground_plane_pose_enabled`` is the projection ablation switch.  When false,
the filtered cloud is unchanged but raw odometry is passed through to the mapper.
"""

from __future__ import annotations

from copy import deepcopy
import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

from co_3dto2d_mapping.gravity_plane_pose_fusion import GravityPlanePoseFusion
from co_3dto2d_mapping.gravity_plane_utils import (
    PlaneFitResult,
    estimate_gravity_constrained_plane,
    pose_z_from_plane,
    quaternion_from_rpy,
    roll_pitch_from_up,
    yaw_from_quaternion,
)
from co_3dto2d_mapping.plane_height_filter import filter_points_by_plane_height
from co_3dto2d_mapping.single_floor_plane import estimate_single_floor_plane


class GravityPlaneHeightCloudFusion(GravityPlanePoseFusion):
    def __init__(self) -> None:
        # The base class creates the raw-cloud/raw-odometry synchronizer and
        # registers this class's overridden _synchronized_callback.
        super().__init__()

        self.declare_parameter("ground_plane_height_filter_enabled", True)
        self.declare_parameter(
            "ground_plane_filtered_cloud_topic",
            "mapping/plane_height_filtered",
        )
        self.declare_parameter("ground_plane_filter_min_height_m", 0.05)
        self.declare_parameter("ground_plane_filter_max_height_m", 1.00)
        self.declare_parameter("ground_plane_filtered_cloud_min_points", 20)
        self.declare_parameter("ground_plane_filtered_cloud_log_period_ms", 2000)

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
        self._height_filter_last_log_ns = 0

        self.get_logger().info(
            "Single-floor shared-state mapping started. cloud=%s odom_in=%s "
            "cloud_out=%s odom_out=%s estimator=%s pose_to_mapping=%s "
            "height=[%.3f, %.3f]m"
            % (
                self.pointcloud_topic,
                self.planar_odometry_topic,
                self.filtered_cloud_topic,
                self.output_odometry_topic,
                self.estimation_mode,
                "true" if self.enabled else "false",
                self.filter_min_height_m,
                self.filter_max_height_m,
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
        if self.filtered_cloud_min_points < 1:
            raise ValueError(
                "ground_plane_filtered_cloud_min_points must be positive"
            )
        if self.filtered_cloud_log_period_ms < 1:
            raise ValueError(
                "ground_plane_filtered_cloud_log_period_ms must be positive"
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
        if not self.enabled or normal is None or height_m is None:
            return output, None, None, None

        roll, pitch = roll_pitch_from_up(normal)
        orientation = input_odometry.pose.pose.orientation
        yaw = yaw_from_quaternion(
            (orientation.x, orientation.y, orientation.z, orientation.w)
        )
        quaternion = quaternion_from_rpy(roll, pitch, yaw)
        input_z = float(input_odometry.pose.pose.position.z)
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
        points_for_fit = (
            local_points
            if local_points is not None
            else np.empty((0, 3), dtype=np.float64)
        )
        result, candidate_count = self._fit_current_floor(points_for_fit, up)

        input_z = float(input_odometry.pose.pose.position.z)
        stamp_ns = self._stamp_ns(cloud.header.stamp)
        normal, height_m, state = self._accept_or_hold_plane(
            result,
            stamp_ns,
            input_z,
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
        output, roll, pitch, pose_z = self._fused_mapping_odometry(
            cloud,
            input_odometry,
            normal,
            height_m,
            result,
        )
        self._publisher.publish(output)
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
