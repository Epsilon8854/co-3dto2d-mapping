#!/usr/bin/env python3
"""Fuse planar mapping pose with IMU-constrained ground-plane attitude/height.

The incoming planar odometry supplies x, y and yaw. A ground plane is estimated
from the current PointCloud2 under an IMU-derived gravity prior; its normal and
signed distance supply roll, pitch and z. When no reliable plane is available,
the node republishes the planar odometry unchanged instead of injecting a bad
3-D correction.
"""

from __future__ import annotations

from copy import deepcopy
import math
import threading
from typing import Optional, Tuple

import message_filters
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, PointCloud2, PointField
from tf2_ros import Buffer, TransformListener

from co_3dto2d_mapping.gravity_plane_utils import (
    PlaneFitConfig,
    PlaneFitResult,
    blend_unit_vectors,
    estimate_gravity_constrained_plane,
    normalize_vector,
    pose_z_from_plane,
    quaternion_from_rpy,
    quaternion_rotate,
    roll_pitch_from_up,
    rotation_matrix_from_quaternion,
    up_from_world_orientation,
    vector_angle,
    yaw_from_quaternion,
)


class GravityPlanePoseFusion(Node):
    def __init__(self) -> None:
        super().__init__("gravity_plane_pose_fusion")
        self._declare_parameters()
        self._read_parameters()
        self._validate_parameters()

        self._publisher = self.create_publisher(
            Odometry, self.output_odometry_topic, 10
        )
        self._imu_lock = threading.Lock()
        self._latest_imu: Optional[Imu] = None
        self._imu_subscription = self.create_subscription(
            Imu,
            self.imu_topic,
            self._imu_callback,
            qos_profile_sensor_data,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        cloud_subscriber = message_filters.Subscriber(
            self,
            PointCloud2,
            self.pointcloud_topic,
            qos_profile=qos_profile_sensor_data,
        )
        odometry_qos = QoSProfile(depth=max(10, self.sync_queue_size))
        odometry_qos.reliability = ReliabilityPolicy.RELIABLE
        odom_subscriber = message_filters.Subscriber(
            self,
            Odometry,
            self.planar_odometry_topic,
            qos_profile=odometry_qos,
        )
        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            [cloud_subscriber, odom_subscriber],
            queue_size=self.sync_queue_size,
            slop=self.sync_slop_sec,
            allow_headerless=False,
        )
        self._synchronizer.registerCallback(self._synchronized_callback)
        self._cloud_subscriber = cloud_subscriber
        self._odom_subscriber = odom_subscriber

        self._filtered_normal: Optional[np.ndarray] = None
        self._filtered_height_m: Optional[float] = None
        self._last_plane_stamp_ns: Optional[int] = None
        self._initial_plane_height_m: Optional[float] = None
        self._initial_pose_z_m: Optional[float] = None
        self._last_log_ns = 0
        self._warned_frames = set()

        self.get_logger().info(
            "Gravity-plane pose fusion started. planar=%s cloud=%s imu=%s output=%s "
            "enabled=%s gravity=%s z_mode=%s"
            % (
                self.planar_odometry_topic,
                self.pointcloud_topic,
                self.imu_topic,
                self.output_odometry_topic,
                "true" if self.enabled else "false",
                self.gravity_source,
                self.z_mode,
            )
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("ground_plane_pose_enabled", True)
        self.declare_parameter("ground_plane_pointcloud_topic", "/livox/lidar")
        self.declare_parameter("ground_plane_imu_topic", "/livox/imu_filtered")
        self.declare_parameter(
            "ground_plane_planar_odometry_topic", "toy/planar_odometry"
        )
        self.declare_parameter(
            "ground_plane_output_odometry_topic", "toy/corrected_odometry"
        )
        self.declare_parameter("ground_plane_local_frame_id", "base_link")
        self.declare_parameter(
            "ground_plane_gravity_source", "orientation_then_acceleration"
        )
        self.declare_parameter("ground_plane_imu_timeout_sec", 0.30)
        self.declare_parameter("ground_plane_acceleration_min_mps2", 6.0)
        self.declare_parameter("ground_plane_acceleration_max_mps2", 13.0)
        self.declare_parameter("ground_plane_acceleration_blend", 0.15)
        self.declare_parameter("ground_plane_sync_queue_size", 30)
        self.declare_parameter("ground_plane_sync_slop_sec", 0.08)

        self.declare_parameter("ground_plane_candidate_min_range_m", 0.30)
        self.declare_parameter("ground_plane_candidate_max_range_m", 6.0)
        self.declare_parameter("ground_plane_candidate_height_min_m", -2.5)
        self.declare_parameter("ground_plane_candidate_height_max_m", -0.05)
        self.declare_parameter("ground_plane_candidate_center_exclusion_m", 0.25)
        self.declare_parameter("ground_plane_max_points", 4000)

        self.declare_parameter("ground_plane_ransac_iterations", 120)
        self.declare_parameter("ground_plane_distance_threshold_m", 0.04)
        self.declare_parameter("ground_plane_max_normal_deviation_deg", 18.0)
        self.declare_parameter("ground_plane_min_inliers", 80)
        self.declare_parameter("ground_plane_min_inlier_ratio", 0.08)
        self.declare_parameter("ground_plane_min_height_m", 0.05)
        self.declare_parameter("ground_plane_max_height_m", 2.5)
        self.declare_parameter("ground_plane_lowest_score_weight", 0.03)
        self.declare_parameter("ground_plane_random_seed", 7)

        self.declare_parameter("ground_plane_filter_gain", 0.25)
        self.declare_parameter("ground_plane_max_height_jump_m", 0.20)
        self.declare_parameter("ground_plane_max_tilt_jump_deg", 10.0)
        self.declare_parameter("ground_plane_hold_timeout_sec", 0.75)
        self.declare_parameter("ground_plane_state_reset_timeout_sec", 3.0)
        self.declare_parameter("ground_plane_z_mode", "height_above_plane")
        self.declare_parameter("ground_plane_reference_z_m", 0.0)
        self.declare_parameter("ground_plane_z_offset_m", 0.0)
        self.declare_parameter("ground_plane_z_stddev_min_m", 0.02)
        self.declare_parameter("ground_plane_orientation_stddev_rad", 0.03)
        self.declare_parameter("ground_plane_log_period_ms", 2000)

    def _read_parameters(self) -> None:
        value = lambda name: self.get_parameter(name).value
        self.enabled = bool(value("ground_plane_pose_enabled"))
        self.pointcloud_topic = str(value("ground_plane_pointcloud_topic"))
        self.imu_topic = str(value("ground_plane_imu_topic"))
        self.planar_odometry_topic = str(
            value("ground_plane_planar_odometry_topic")
        )
        self.output_odometry_topic = str(
            value("ground_plane_output_odometry_topic")
        )
        self.local_frame_id = str(value("ground_plane_local_frame_id"))
        self.gravity_source = str(value("ground_plane_gravity_source"))
        self.imu_timeout_sec = float(value("ground_plane_imu_timeout_sec"))
        self.acceleration_min_mps2 = float(
            value("ground_plane_acceleration_min_mps2")
        )
        self.acceleration_max_mps2 = float(
            value("ground_plane_acceleration_max_mps2")
        )
        self.acceleration_blend = float(
            value("ground_plane_acceleration_blend")
        )
        self.sync_queue_size = int(value("ground_plane_sync_queue_size"))
        self.sync_slop_sec = float(value("ground_plane_sync_slop_sec"))

        self.candidate_min_range_m = float(
            value("ground_plane_candidate_min_range_m")
        )
        self.candidate_max_range_m = float(
            value("ground_plane_candidate_max_range_m")
        )
        self.candidate_height_min_m = float(
            value("ground_plane_candidate_height_min_m")
        )
        self.candidate_height_max_m = float(
            value("ground_plane_candidate_height_max_m")
        )
        self.candidate_center_exclusion_m = float(
            value("ground_plane_candidate_center_exclusion_m")
        )
        self.max_points = int(value("ground_plane_max_points"))

        self.plane_config = PlaneFitConfig(
            ransac_iterations=int(value("ground_plane_ransac_iterations")),
            distance_threshold_m=float(
                value("ground_plane_distance_threshold_m")
            ),
            max_normal_deviation_rad=math.radians(
                float(value("ground_plane_max_normal_deviation_deg"))
            ),
            min_inliers=int(value("ground_plane_min_inliers")),
            min_inlier_ratio=float(value("ground_plane_min_inlier_ratio")),
            min_height_m=float(value("ground_plane_min_height_m")),
            max_height_m=float(value("ground_plane_max_height_m")),
            max_points=self.max_points,
            lowest_plane_score_weight=float(
                value("ground_plane_lowest_score_weight")
            ),
            random_seed=int(value("ground_plane_random_seed")),
        )

        self.filter_gain = float(value("ground_plane_filter_gain"))
        self.max_height_jump_m = float(
            value("ground_plane_max_height_jump_m")
        )
        self.max_tilt_jump_rad = math.radians(
            float(value("ground_plane_max_tilt_jump_deg"))
        )
        self.hold_timeout_sec = float(
            value("ground_plane_hold_timeout_sec")
        )
        self.state_reset_timeout_sec = float(
            value("ground_plane_state_reset_timeout_sec")
        )
        self.z_mode = str(value("ground_plane_z_mode"))
        self.reference_z_m = float(value("ground_plane_reference_z_m"))
        self.z_offset_m = float(value("ground_plane_z_offset_m"))
        self.z_stddev_min_m = float(value("ground_plane_z_stddev_min_m"))
        self.orientation_stddev_rad = float(
            value("ground_plane_orientation_stddev_rad")
        )
        self.log_period_ms = int(value("ground_plane_log_period_ms"))

    def _validate_parameters(self) -> None:
        if not self.pointcloud_topic or not self.imu_topic:
            raise ValueError("ground-plane input topics must not be empty")
        if not self.planar_odometry_topic or not self.output_odometry_topic:
            raise ValueError("ground-plane odometry topics must not be empty")
        if self.planar_odometry_topic == self.output_odometry_topic:
            raise ValueError("planar and output odometry topics must differ")
        if self.gravity_source not in {
            "orientation",
            "acceleration",
            "orientation_then_acceleration",
            "blend",
        }:
            raise ValueError(
                "ground_plane_gravity_source must be orientation, acceleration, "
                "orientation_then_acceleration, or blend"
            )
        if self.imu_timeout_sec < 0.0:
            raise ValueError("ground_plane_imu_timeout_sec must be non-negative")
        if not 0.0 <= self.acceleration_blend <= 1.0:
            raise ValueError("ground_plane_acceleration_blend must be in [0, 1]")
        if (
            self.acceleration_min_mps2 < 0.0
            or self.acceleration_max_mps2 <= self.acceleration_min_mps2
        ):
            raise ValueError("ground-plane acceleration magnitude bounds are invalid")
        if self.sync_queue_size < 2 or self.sync_slop_sec < 0.0:
            raise ValueError("ground-plane synchronization parameters are invalid")
        if (
            self.candidate_min_range_m < 0.0
            or self.candidate_max_range_m <= self.candidate_min_range_m
            or self.candidate_height_max_m <= self.candidate_height_min_m
            or self.candidate_center_exclusion_m < 0.0
        ):
            raise ValueError("ground-plane candidate bounds are invalid")
        if self.plane_config.ransac_iterations < 1:
            raise ValueError("ground_plane_ransac_iterations must be positive")
        if self.plane_config.distance_threshold_m <= 0.0:
            raise ValueError("ground_plane_distance_threshold_m must be positive")
        if not 0.0 <= self.plane_config.max_normal_deviation_rad <= math.pi / 2.0:
            raise ValueError("ground-plane normal deviation must be in [0, 90] deg")
        if self.plane_config.min_inliers < 3 or self.max_points < self.plane_config.min_inliers:
            raise ValueError("ground-plane point-count parameters are invalid")
        if not 0.0 < self.plane_config.min_inlier_ratio <= 1.0:
            raise ValueError("ground_plane_min_inlier_ratio must be in (0, 1]")
        if (
            self.plane_config.min_height_m < 0.0
            or self.plane_config.max_height_m <= self.plane_config.min_height_m
        ):
            raise ValueError("ground-plane height bounds are invalid")
        if not 0.0 <= self.filter_gain <= 1.0:
            raise ValueError("ground_plane_filter_gain must be in [0, 1]")
        if (
            self.max_height_jump_m < 0.0
            or self.max_tilt_jump_rad < 0.0
            or self.hold_timeout_sec < 0.0
            or self.state_reset_timeout_sec < self.hold_timeout_sec
        ):
            raise ValueError("ground-plane temporal parameters are invalid")
        if self.z_mode not in {
            "height_above_plane",
            "relative_to_initial",
            "passthrough",
        }:
            raise ValueError("ground_plane_z_mode is invalid")
        if self.z_stddev_min_m < 0.0 or self.orientation_stddev_rad < 0.0:
            raise ValueError("ground-plane covariance parameters are invalid")

    @staticmethod
    def _stamp_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _imu_callback(self, msg: Imu) -> None:
        with self._imu_lock:
            self._latest_imu = msg

    def _transform_direction(
        self, vector: np.ndarray, source_frame: str
    ) -> Optional[np.ndarray]:
        direction = normalize_vector(vector)
        if direction is None:
            return None
        if not source_frame or source_frame == self.local_frame_id:
            return direction
        try:
            transform = self._tf_buffer.lookup_transform(
                self.local_frame_id, source_frame, Time()
            )
        except Exception as exc:
            key = (self.local_frame_id, source_frame)
            if key not in self._warned_frames:
                self._warned_frames.add(key)
                self.get_logger().warn(
                    "Waiting for transform %s <- %s for gravity/plane fusion: %s"
                    % (self.local_frame_id, source_frame, exc)
                )
            return None
        rotation = transform.transform.rotation
        transformed = quaternion_rotate(
            (rotation.x, rotation.y, rotation.z, rotation.w), direction
        )
        return normalize_vector(transformed)

    def _up_from_imu(self, cloud_stamp) -> Optional[np.ndarray]:
        with self._imu_lock:
            imu = self._latest_imu
        if imu is None:
            return None

        cloud_stamp_ns = self._stamp_ns(cloud_stamp)
        imu_stamp_ns = self._stamp_ns(imu.header.stamp)
        if (
            self.imu_timeout_sec > 0.0
            and cloud_stamp_ns > 0
            and imu_stamp_ns > 0
            and abs(cloud_stamp_ns - imu_stamp_ns)
            > int(self.imu_timeout_sec * 1e9)
        ):
            return None

        orientation_up = None
        if imu.orientation_covariance[0] >= 0.0:
            try:
                orientation_up = up_from_world_orientation(
                    (
                        imu.orientation.x,
                        imu.orientation.y,
                        imu.orientation.z,
                        imu.orientation.w,
                    )
                )
                orientation_up = self._transform_direction(
                    orientation_up, imu.header.frame_id
                )
            except ValueError:
                orientation_up = None

        acceleration_up = None
        if imu.linear_acceleration_covariance[0] >= 0.0:
            acceleration = np.asarray(
                [
                    imu.linear_acceleration.x,
                    imu.linear_acceleration.y,
                    imu.linear_acceleration.z,
                ],
                dtype=np.float64,
            )
            magnitude = float(np.linalg.norm(acceleration))
            if self.acceleration_min_mps2 <= magnitude <= self.acceleration_max_mps2:
                acceleration_up = self._transform_direction(
                    acceleration, imu.header.frame_id
                )

        if self.gravity_source == "orientation":
            return orientation_up
        if self.gravity_source == "acceleration":
            return acceleration_up
        if self.gravity_source == "orientation_then_acceleration":
            return orientation_up if orientation_up is not None else acceleration_up
        if orientation_up is None:
            return acceleration_up
        if acceleration_up is None:
            return orientation_up
        return blend_unit_vectors(
            orientation_up, acceleration_up, self.acceleration_blend
        )

    @staticmethod
    def _point_field(msg: PointCloud2, name: str) -> PointField:
        for field in msg.fields:
            if field.name == name:
                return field
        raise ValueError("PointCloud2 is missing field %r" % name)

    @staticmethod
    def _field_format(field: PointField, big_endian: bool) -> str:
        byte_order = ">" if big_endian else "<"
        if field.datatype == PointField.FLOAT32:
            return byte_order + "f4"
        if field.datatype == PointField.FLOAT64:
            return byte_order + "f8"
        raise ValueError("PointCloud2 xyz fields must be FLOAT32 or FLOAT64")

    def _cloud_xyz(self, msg: PointCloud2) -> np.ndarray:
        x_field = self._point_field(msg, "x")
        y_field = self._point_field(msg, "y")
        z_field = self._point_field(msg, "z")
        dtype = np.dtype(
            {
                "names": ["x", "y", "z"],
                "formats": [
                    self._field_format(x_field, msg.is_bigendian),
                    self._field_format(y_field, msg.is_bigendian),
                    self._field_format(z_field, msg.is_bigendian),
                ],
                "offsets": [x_field.offset, y_field.offset, z_field.offset],
                "itemsize": int(msg.point_step),
            }
        )
        rows = []
        data = memoryview(msg.data)
        width = int(msg.width)
        height = int(msg.height)
        for row in range(height):
            offset = row * int(msg.row_step)
            rows.append(np.frombuffer(data, dtype=dtype, count=width, offset=offset))
        if not rows:
            return np.empty((0, 3), dtype=np.float64)
        array = np.concatenate(rows) if len(rows) > 1 else rows[0]
        points = np.column_stack((array["x"], array["y"], array["z"]))
        points = np.asarray(points, dtype=np.float64)
        return points[np.all(np.isfinite(points), axis=1)]

    def _cloud_to_local(self, msg: PointCloud2) -> Optional[np.ndarray]:
        try:
            points = self._cloud_xyz(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return None
        if not msg.header.frame_id or msg.header.frame_id == self.local_frame_id:
            return points
        try:
            transform = self._tf_buffer.lookup_transform(
                self.local_frame_id, msg.header.frame_id, Time()
            )
        except Exception as exc:
            key = (self.local_frame_id, msg.header.frame_id)
            if key not in self._warned_frames:
                self._warned_frames.add(key)
                self.get_logger().warn(
                    "Waiting for transform %s <- %s before ground-plane fitting: %s"
                    % (self.local_frame_id, msg.header.frame_id, exc)
                )
            return None
        rotation = transform.transform.rotation
        matrix = rotation_matrix_from_quaternion(
            (rotation.x, rotation.y, rotation.z, rotation.w)
        )
        translation = np.asarray(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float64,
        )
        return points @ matrix.T + translation

    def _candidate_points(
        self, local_points: np.ndarray, up: np.ndarray
    ) -> np.ndarray:
        if len(local_points) == 0:
            return local_points
        vertical = local_points @ up
        tangent = local_points - vertical[:, None] * up[None, :]
        radial = np.linalg.norm(tangent, axis=1)
        mask = (
            (radial >= self.candidate_min_range_m)
            & (radial <= self.candidate_max_range_m)
            & (vertical >= self.candidate_height_min_m)
            & (vertical <= self.candidate_height_max_m)
        )
        if self.candidate_center_exclusion_m > 0.0:
            mask &= radial >= self.candidate_center_exclusion_m
        return local_points[mask]

    def _clear_plane_state(self) -> None:
        self._filtered_normal = None
        self._filtered_height_m = None
        self._last_plane_stamp_ns = None
        self._initial_plane_height_m = None
        self._initial_pose_z_m = None

    def _accept_or_hold_plane(
        self,
        result: Optional[PlaneFitResult],
        stamp_ns: int,
        input_pose_z_m: float,
    ) -> Tuple[Optional[np.ndarray], Optional[float], str]:
        if (
            self._last_plane_stamp_ns is not None
            and self.state_reset_timeout_sec > 0.0
            and stamp_ns - self._last_plane_stamp_ns
            > int(self.state_reset_timeout_sec * 1e9)
        ):
            self._clear_plane_state()

        if result is not None:
            if self._filtered_normal is not None and self._filtered_height_m is not None:
                height_jump = abs(result.height_m - self._filtered_height_m)
                tilt_jump = vector_angle(result.normal, self._filtered_normal)
                if (
                    height_jump <= self.max_height_jump_m
                    and tilt_jump <= self.max_tilt_jump_rad
                ):
                    self._filtered_normal = blend_unit_vectors(
                        self._filtered_normal, result.normal, self.filter_gain
                    )
                    self._filtered_height_m = (
                        (1.0 - self.filter_gain) * self._filtered_height_m
                        + self.filter_gain * result.height_m
                    )
                    self._last_plane_stamp_ns = stamp_ns
                    return self._filtered_normal, self._filtered_height_m, "accepted"
                return self._held_plane(stamp_ns, "jump_rejected")

            self._filtered_normal = np.asarray(result.normal, dtype=np.float64)
            self._filtered_height_m = float(result.height_m)
            self._last_plane_stamp_ns = stamp_ns
            self._initial_plane_height_m = float(result.height_m)
            self._initial_pose_z_m = float(input_pose_z_m)
            return self._filtered_normal, self._filtered_height_m, "initialized"

        return self._held_plane(stamp_ns, "fit_failed")

    def _held_plane(
        self, stamp_ns: int, reason: str
    ) -> Tuple[Optional[np.ndarray], Optional[float], str]:
        if (
            self._filtered_normal is not None
            and self._filtered_height_m is not None
            and self._last_plane_stamp_ns is not None
            and stamp_ns - self._last_plane_stamp_ns
            <= int(self.hold_timeout_sec * 1e9)
        ):
            return self._filtered_normal, self._filtered_height_m, "held_" + reason
        return None, None, reason

    def _log_result(
        self,
        state: str,
        candidate_count: int,
        result: Optional[PlaneFitResult],
        roll: Optional[float] = None,
        pitch: Optional[float] = None,
        pose_z: Optional[float] = None,
    ) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_log_ns < self.log_period_ms * 1_000_000:
            return
        self._last_log_ns = now_ns
        if result is None:
            self.get_logger().warn(
                "Ground-plane pose %s: candidates=%d; planar z/roll/pitch fallback may be used."
                % (state, candidate_count)
            )
            return
        self.get_logger().info(
            "Ground-plane pose %s: candidates=%d inliers=%d ratio=%.2f rmse=%.3fm "
            "normal_delta=%.2fdeg pose=(z=%.3f roll=%.2fdeg pitch=%.2fdeg)"
            % (
                state,
                candidate_count,
                result.inlier_count,
                result.inlier_ratio,
                result.rmse_m,
                math.degrees(result.normal_deviation_rad),
                pose_z if pose_z is not None else math.nan,
                math.degrees(roll) if roll is not None else math.nan,
                math.degrees(pitch) if pitch is not None else math.nan,
            )
        )

    def _synchronized_callback(
        self, cloud: PointCloud2, planar_odometry: Odometry
    ) -> None:
        output = deepcopy(planar_odometry)
        output.header.stamp = cloud.header.stamp
        if not self.enabled:
            self._publisher.publish(output)
            return

        up = self._up_from_imu(cloud.header.stamp)
        local_points = self._cloud_to_local(cloud) if up is not None else None
        candidates = (
            self._candidate_points(local_points, up)
            if local_points is not None and up is not None
            else np.empty((0, 3), dtype=np.float64)
        )
        result = None
        if up is not None and len(candidates) >= self.plane_config.min_inliers:
            result = estimate_gravity_constrained_plane(
                candidates, up, self.plane_config
            )

        stamp_ns = self._stamp_ns(cloud.header.stamp)
        input_z = float(planar_odometry.pose.pose.position.z)
        normal, height_m, state = self._accept_or_hold_plane(
            result, stamp_ns, input_z
        )
        if normal is None or height_m is None:
            self._log_result(state, len(candidates), result)
            self._publisher.publish(output)
            return

        roll, pitch = roll_pitch_from_up(normal)
        orientation = planar_odometry.pose.pose.orientation
        yaw = yaw_from_quaternion(
            (orientation.x, orientation.y, orientation.z, orientation.w)
        )
        quaternion = quaternion_from_rpy(roll, pitch, yaw)
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

        output.pose.pose.position.x = planar_odometry.pose.pose.position.x
        output.pose.pose.position.y = planar_odometry.pose.pose.position.y
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

        self._log_result(
            state,
            len(candidates),
            result,
            roll=roll,
            pitch=pitch,
            pose_z=pose_z,
        )
        self._publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GravityPlanePoseFusion()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
