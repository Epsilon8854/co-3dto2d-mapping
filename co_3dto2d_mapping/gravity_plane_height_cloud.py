#!/usr/bin/env python3
"""Ground-plane pose fusion plus a plane-relative obstacle cloud.

The existing pose-fusion implementation keeps producing the final odometry:
planar x/y/yaw plus plane-derived z/roll/pitch. This wrapper additionally fits
that same IMU-constrained ground plane directly from each raw cloud (without
waiting for planar odometry) and publishes only points between a configurable
height above the plane. The default band is 0.05 m through 1.00 m.

Publishing the filtered cloud independently prevents a dependency cycle:

raw cloud + IMU -> plane-height cloud -> planar mapper -> planar odometry
                                            |                 |
                                            +---- pose fusion-+
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

from co_3dto2d_mapping.gravity_plane_pose_fusion import GravityPlanePoseFusion
from co_3dto2d_mapping.gravity_plane_utils import (
    PlaneFitResult,
    blend_unit_vectors,
    estimate_gravity_constrained_plane,
    vector_angle,
)
from co_3dto2d_mapping.plane_height_filter import filter_points_by_plane_height


class GravityPlaneHeightCloudFusion(GravityPlanePoseFusion):
    def __init__(self) -> None:
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

        self._height_cloud_publisher = self.create_publisher(
            PointCloud2,
            self.filtered_cloud_topic,
            qos_profile_sensor_data,
        )
        # The base class already subscribes to this cloud through a message
        # filter paired with planar odometry. This additional subscription is
        # intentional: plane-height filtering must run before planar odometry
        # exists so the mapper can consume the resulting cloud.
        self._height_cloud_subscription = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self._height_cloud_callback,
            qos_profile_sensor_data,
        )

        self._height_filter_normal: Optional[np.ndarray] = None
        self._height_filter_offset_m: Optional[float] = None
        self._height_filter_last_valid_stamp_ns: Optional[int] = None
        self._height_filter_last_log_ns = 0

        self.get_logger().info(
            "Plane-relative cloud filter started. input=%s output=%s "
            "height=[%.3f, %.3f]m enabled=%s"
            % (
                self.pointcloud_topic,
                self.filtered_cloud_topic,
                self.filter_min_height_m,
                self.filter_max_height_m,
                "true" if self.height_filter_enabled else "false",
            )
        )

    def _reset_height_filter_state(self) -> None:
        self._height_filter_normal = None
        self._height_filter_offset_m = None
        self._height_filter_last_valid_stamp_ns = None

    def _select_height_filter_plane(
        self,
        result: Optional[PlaneFitResult],
        stamp_ns: int,
    ) -> Tuple[Optional[np.ndarray], Optional[float], str]:
        last_stamp = self._height_filter_last_valid_stamp_ns
        if last_stamp is not None and stamp_ns < last_stamp:
            self._reset_height_filter_state()
            last_stamp = None
        if (
            last_stamp is not None
            and self.state_reset_timeout_sec > 0.0
            and stamp_ns - last_stamp
            > int(self.state_reset_timeout_sec * 1e9)
        ):
            self._reset_height_filter_state()
            last_stamp = None

        if result is not None:
            if (
                self._height_filter_normal is not None
                and self._height_filter_offset_m is not None
            ):
                height_jump = abs(
                    float(result.offset) - self._height_filter_offset_m
                )
                tilt_jump = vector_angle(
                    result.normal, self._height_filter_normal
                )
                if (
                    height_jump <= self.max_height_jump_m
                    and tilt_jump <= self.max_tilt_jump_rad
                ):
                    self._height_filter_normal = blend_unit_vectors(
                        self._height_filter_normal,
                        result.normal,
                        self.filter_gain,
                    )
                    self._height_filter_offset_m = (
                        (1.0 - self.filter_gain)
                        * self._height_filter_offset_m
                        + self.filter_gain * float(result.offset)
                    )
                    self._height_filter_last_valid_stamp_ns = stamp_ns
                    return (
                        self._height_filter_normal,
                        self._height_filter_offset_m,
                        "accepted",
                    )
                return self._held_height_filter_plane(stamp_ns, "jump_rejected")

            self._height_filter_normal = np.asarray(
                result.normal, dtype=np.float64
            )
            self._height_filter_offset_m = float(result.offset)
            self._height_filter_last_valid_stamp_ns = stamp_ns
            return (
                self._height_filter_normal,
                self._height_filter_offset_m,
                "initialized",
            )

        return self._held_height_filter_plane(stamp_ns, "fit_failed")

    def _held_height_filter_plane(
        self, stamp_ns: int, reason: str
    ) -> Tuple[Optional[np.ndarray], Optional[float], str]:
        if (
            self._height_filter_normal is not None
            and self._height_filter_offset_m is not None
            and self._height_filter_last_valid_stamp_ns is not None
            and stamp_ns - self._height_filter_last_valid_stamp_ns
            <= int(self.hold_timeout_sec * 1e9)
        ):
            return (
                self._height_filter_normal,
                self._height_filter_offset_m,
                "held_" + reason,
            )
        return None, None, reason

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

    def _height_cloud_callback(self, cloud: PointCloud2) -> None:
        local_points = self._cloud_to_local(cloud)
        if local_points is None or len(local_points) == 0:
            return

        if not self.height_filter_enabled:
            self._height_cloud_publisher.publish(
                self._xyz_cloud(
                    local_points,
                    cloud.header.stamp,
                    self.local_frame_id,
                )
            )
            return

        up = self._up_from_imu(cloud.header.stamp)
        candidates = (
            self._candidate_points(local_points, up)
            if up is not None
            else np.empty((0, 3), dtype=np.float64)
        )
        result = None
        if up is not None and len(candidates) >= self.plane_config.min_inliers:
            result = estimate_gravity_constrained_plane(
                candidates,
                up,
                self.plane_config,
            )

        stamp_ns = self._stamp_ns(cloud.header.stamp)
        normal, offset_m, state = self._select_height_filter_plane(
            result,
            stamp_ns,
        )
        if normal is None or offset_m is None:
            self._log_height_filter(
                state=state,
                input_count=len(local_points),
                output_count=0,
                candidate_count=len(candidates),
                result=result,
            )
            return

        filtered = filter_points_by_plane_height(
            local_points,
            normal,
            offset_m,
            self.filter_min_height_m,
            self.filter_max_height_m,
        )
        if len(filtered.points) < self.filtered_cloud_min_points:
            self._log_height_filter(
                state="too_few_filtered_points",
                input_count=len(local_points),
                output_count=len(filtered.points),
                candidate_count=len(candidates),
                result=result,
            )
            return

        self._height_cloud_publisher.publish(
            self._xyz_cloud(
                filtered.points,
                cloud.header.stamp,
                self.local_frame_id,
            )
        )
        self._log_height_filter(
            state=state,
            input_count=len(local_points),
            output_count=len(filtered.points),
            candidate_count=len(candidates),
            result=result,
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
                "waiting for a valid ground plane."
                % (state, input_count, candidate_count, output_count)
            )
            return
        self.get_logger().info(
            "Plane-height cloud %s: raw=%d candidates=%d output=%d "
            "band=[%.2f, %.2f]m plane_inliers=%d ratio=%.2f rmse=%.3fm"
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
