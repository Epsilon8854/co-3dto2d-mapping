#!/usr/bin/env python3
"""Register two live robots from cropped RTAB-Map input clouds in full XYZ.

The accepted 6-DoF ICP result is projected to x/y/yaw because the downstream
merged occupancy-map interface is planar.  Legacy global-occupancy mode still
delegates to the original constrained 2D implementation.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2, PointField

from co_3dto2d_mapping.alignment_utils import angular_distance
from co_3dto2d_mapping.constrained_initial_xy_icp_alignment import (
    HeadingConstrainedInitialXyIcpAlignment,
    _env_bool,
    _env_float,
)
from co_3dto2d_mapping.heading_constraint import (
    initial_yaw_candidates,
    registration_rank,
    yaw_within_prior,
)
from co_3dto2d_mapping.initial_xy_icp_alignment import InitialXyIcpAlignment
from co_3dto2d_mapping.pointcloud_registration import (
    estimate_rigid_transform,
    invert_transform,
    rotation_angle,
    rotation_rpy,
    rotation_tilt,
    rotation_yaw,
    transform_points,
    voxel_downsample,
    yaw_rotation_matrix,
)


class CroppedXyzInitialIcpAlignment(HeadingConstrainedInitialXyIcpAlignment):
    """Use XYZ correspondences for ``cloud_initial`` startup alignment."""

    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter("robot0_local_frame_id", "")
        self.declare_parameter("robot1_local_frame_id", "")
        self.declare_parameter("use_z_filter", True)
        self.declare_parameter("slice_z_in_cloud_frame", True)
        self.declare_parameter("range_min_m", 0.0)
        self.declare_parameter("range_max_m", 0.0)
        self.declare_parameter(
            "enforce_tilt_prior",
            _env_bool("CO3DTO2D_ENFORCE_TILT_PRIOR", True),
        )
        self.declare_parameter(
            "max_tilt_deviation_rad",
            math.radians(_env_float("CO3DTO2D_MAX_RELATIVE_TILT_DEG", 15.0)),
        )

        frame0 = str(self.get_parameter("robot0_local_frame_id").value)
        frame1 = str(self.get_parameter("robot1_local_frame_id").value)
        self.robot_local_frame_ids = (
            frame0 or self.local_frame_id,
            frame1 or self.local_frame_id,
        )
        self.use_z_filter = bool(self.get_parameter("use_z_filter").value)
        self.slice_z_in_cloud_frame = bool(
            self.get_parameter("slice_z_in_cloud_frame").value
        )
        self.range_min_m = max(0.0, float(self.get_parameter("range_min_m").value))
        self.range_max_m = max(0.0, float(self.get_parameter("range_max_m").value))
        self.enforce_tilt_prior = bool(
            self.get_parameter("enforce_tilt_prior").value
        )
        self.max_tilt_deviation_rad = float(
            self.get_parameter("max_tilt_deviation_rad").value
        )
        if self.input_mode == "cloud_initial" and not all(self.robot_local_frame_ids):
            raise ValueError("both robot local frame IDs are required for XYZ alignment")
        if self.use_z_filter and self.z_min > self.z_max:
            raise ValueError("z_min must be <= z_max")
        if self.range_max_m > 0.0 and self.range_max_m < self.range_min_m:
            raise ValueError("range_max_m must be zero or >= range_min_m")
        if not 0.0 <= self.max_tilt_deviation_rad <= math.pi:
            raise ValueError("max_tilt_deviation_rad must be between 0 and pi")

        self._last_prior_rejection: Optional[str] = None
        self.get_logger().info(
            "Cropped XYZ startup ICP: mode=%s local_frames=(%s, %s) "
            "transform_to_local=%s z_filter=%s z=[%.2f, %.2f] invert_z=%s "
            "z_frame=%s range=[%.2f, %.2f] center_box=%.2f voxel=%.3f "
            "max_points=%d tilt_prior=%s max_tilt=%.1fdeg"
            % (
                self.input_mode,
                *self.robot_local_frame_ids,
                "true" if self.transform_cloud_to_local_frame else "false",
                "true" if self.use_z_filter else "false",
                self.z_min,
                self.z_max,
                "true" if self.invert_z_slice else "false",
                "cloud" if self.slice_z_in_cloud_frame else "local",
                self.range_min_m,
                self.range_max_m,
                self.center_box_half_extent_m,
                self.voxel_size,
                self.max_points,
                "true" if self.enforce_tilt_prior else "false",
                math.degrees(self.max_tilt_deviation_rad),
            )
        )

    def _cloud_collection_enabled(self) -> bool:
        if self.alignment_msg is None:
            return True
        if self.lock_after_first_alignment:
            return False
        return (
            not self.last_recompute_ns
            or self.get_clock().now().nanoseconds - self.last_recompute_ns
            >= int(self.recompute_period_sec * 1e9)
        )

    def _robot0_callback(self, msg: PointCloud2) -> None:
        self._cache_robot_cloud(0, "robot0", self.robot0_frames, msg)

    def _robot1_callback(self, msg: PointCloud2) -> None:
        self._cache_robot_cloud(1, "robot1", self.robot1_frames, msg)

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
        submap = self._cache_initial_xyz_frame(robot_index, robot_name, frames, msg)
        if robot_index == 0:
            self.robot0_points = submap
        else:
            self.robot1_points = submap
        self._try_compute_alignment(msg.header.stamp)

    def _cache_initial_xyz_frame(
        self,
        robot_index: int,
        robot_name: str,
        frames: List[np.ndarray],
        msg: PointCloud2,
    ) -> Optional[np.ndarray]:
        points = self._cloud_to_xyz_points(robot_index, msg)
        if points is None:
            return None
        frames.append(points)
        self.get_logger().info(
            "Cached %s cropped XYZ ICP frame %d/%d with %d points."
            % (robot_name, len(frames), self.frame_count, len(points))
        )
        if len(frames) < self.frame_count:
            return None
        non_empty = [frame for frame in frames if len(frame) > 0]
        merged = np.vstack(non_empty) if non_empty else np.empty((0, 3))
        merged = self._prepare_icp_points(merged)
        self.get_logger().info(
            "Built %s cropped XYZ ICP submap from %d frames with %d points."
            % (robot_name, len(frames), len(merged))
        )
        return merged

    def _lookup_robot_cloud_transform(
        self, robot_index: int, msg: PointCloud2
    ) -> Tuple[bool, Optional[TransformStamped]]:
        local_frame = self.robot_local_frame_ids[robot_index]
        if (
            not self.transform_cloud_to_local_frame
            or not msg.header.frame_id
            or msg.header.frame_id == local_frame
        ):
            return True, None
        try:
            return True, self.tf_buffer.lookup_transform(
                local_frame, msg.header.frame_id, Time()
            )
        except Exception as exc:
            key = (local_frame, msg.header.frame_id)
            if key not in self.missing_transform_warnings:
                self.missing_transform_warnings.add(key)
                self.get_logger().warn(
                    "Waiting for transform %s <- %s before caching 3D ICP "
                    "frames: %s" % (local_frame, msg.header.frame_id, exc)
                )
            return False, None

    @staticmethod
    def _transform_components(
        transform: TransformStamped,
    ) -> Tuple[np.ndarray, np.ndarray]:
        q = np.asarray(
            [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ],
            dtype=np.float64,
        )
        norm = float(np.linalg.norm(q))
        if norm <= 1e-12:
            rotation = np.eye(3)
        else:
            x, y, z, w = q / norm
            rotation = np.asarray(
                [
                    [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                    [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                    [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
                ]
            )
        translation = np.asarray(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float64,
        )
        return rotation, translation

    @staticmethod
    def _field_dtype(msg: PointCloud2, name: str) -> Tuple[int, np.dtype]:
        field = next((field for field in msg.fields if field.name == name), None)
        if field is None:
            raise ValueError("PointCloud2 is missing field '%s'" % name)
        order = ">" if msg.is_bigendian else "<"
        if field.datatype == PointField.FLOAT32:
            return int(field.offset), np.dtype(order + "f4")
        if field.datatype == PointField.FLOAT64:
            return int(field.offset), np.dtype(order + "f8")
        raise ValueError("PointCloud2 field '%s' must be FLOAT32 or FLOAT64" % name)

    @classmethod
    def _read_cloud_field(cls, msg: PointCloud2, name: str) -> np.ndarray:
        offset, dtype = cls._field_dtype(msg, name)
        width, height, step = int(msg.width), int(msg.height), int(msg.point_step)
        row_step = int(msg.row_step) or width * step
        if width <= 0 or height <= 0 or step <= 0:
            return np.empty(0)
        data = bytes(msg.data)
        rows = []
        for row in range(height):
            row_start = row * row_step
            row_end = min(len(data), row_start + row_step)
            start = row_start + offset
            if start + dtype.itemsize > row_end:
                break
            available = 1 + (row_end - start - dtype.itemsize) // step
            count = min(width, max(0, available))
            if count:
                values = np.ndarray(
                    (count,), dtype=dtype, buffer=data, offset=start, strides=(step,)
                )
                rows.append(np.asarray(values, dtype=np.float64))
        return np.concatenate(rows) if rows else np.empty(0)

    def _cloud_to_xyz_points(
        self, robot_index: int, msg: PointCloud2
    ) -> Optional[np.ndarray]:
        transform_ok, cloud_to_local = self._lookup_robot_cloud_transform(
            robot_index, msg
        )
        if not transform_ok:
            return None
        try:
            fields = [self._read_cloud_field(msg, name) for name in ("x", "y", "z")]
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return np.empty((0, 3))
        count = min(map(len, fields), default=0)
        if count == 0:
            return np.empty((0, 3))
        cloud = np.column_stack([field[:count] for field in fields])
        cloud = cloud[np.all(np.isfinite(cloud), axis=1)]
        local = cloud
        if cloud_to_local is not None and len(local):
            rotation, translation = self._transform_components(cloud_to_local)
            local = transform_points(local, rotation, translation)
        if not len(local):
            return local

        keep = np.ones(len(local), dtype=bool)
        if self.use_z_filter:
            z = cloud[:, 2] if self.slice_z_in_cloud_frame else local[:, 2]
            inside = (z >= self.z_min) & (z <= self.z_max)
            keep &= ~inside if self.invert_z_slice else inside
        ranges = np.hypot(local[:, 0], local[:, 1])
        if self.range_min_m > 0.0:
            keep &= ranges >= self.range_min_m
        if self.range_max_m > 0.0:
            keep &= ranges <= self.range_max_m
        if self.center_box_half_extent_m > 0.0:
            half = self.center_box_half_extent_m
            keep &= ~((np.abs(local[:, 0]) <= half) & (np.abs(local[:, 1]) <= half))
        return self._prepare_icp_points(local[keep])

    def _prepare_icp_points(self, points: np.ndarray) -> np.ndarray:
        prepared = voxel_downsample(points, self.voxel_size)
        if len(prepared) > self.max_points:
            indices = np.linspace(0, len(prepared) - 1, self.max_points, dtype=np.int64)
            prepared = prepared[indices]
        return prepared

    def _published_yaw(self, rotation: np.ndarray) -> float:
        published = rotation.T if self.invert_result else rotation
        return rotation_yaw(published)

    def _published_tilt(self, rotation: np.ndarray) -> float:
        published = rotation.T if self.invert_result else rotation
        return rotation_tilt(published)

    def _transform_allowed(self, rotation: np.ndarray) -> bool:
        heading_allowed = (
            not self.enforce_heading_prior
            or yaw_within_prior(
                self._published_yaw(rotation),
                self.expected_yaw_rad,
                self.max_yaw_deviation_rad,
            )
        )
        tilt_allowed = (
            not self.enforce_tilt_prior
            or self._published_tilt(rotation) <= self.max_tilt_deviation_rad + 1e-9
        )
        return heading_allowed and tilt_allowed

    def _remember_prior_rejection(self, rotation: np.ndarray, reason: str) -> None:
        yaw, tilt = self._published_yaw(rotation), self._published_tilt(rotation)
        self._last_prior_rejection = (
            "%s: yaw=%.1fdeg expected=%.1fdeg yaw_deviation=%.1fdeg "
            "yaw_limit=%.1fdeg tilt=%.1fdeg tilt_limit=%.1fdeg"
            % (
                reason,
                math.degrees(yaw),
                math.degrees(self.expected_yaw_rad),
                math.degrees(angular_distance(yaw, self.expected_yaw_rad)),
                math.degrees(self.max_yaw_deviation_rad),
                math.degrees(tilt),
                math.degrees(self.max_tilt_deviation_rad),
            )
        )

    def _run_xyz_icp_once(
        self,
        target: np.ndarray,
        source: np.ndarray,
        initial_rotation: np.ndarray,
        initial_translation: np.ndarray,
        label: str,
    ):
        tree = cKDTree(target)
        rotation = initial_rotation.copy()
        translation = initial_translation.copy()
        last_rmse = None
        if not self._transform_allowed(rotation):
            self._remember_prior_rejection(rotation, "initial transform rejected")
            return None

        for _ in range(self.max_iterations):
            transformed = transform_points(source, rotation, translation)
            distances, indices = tree.query(transformed, k=1)
            mask = distances <= self.max_correspondence_distance
            if int(np.count_nonzero(mask)) < self.min_correspondences:
                return None
            delta_rotation, delta_translation = estimate_rigid_transform(
                transformed[mask], target[indices[mask]]
            )
            delta_angle = rotation_angle(delta_rotation)
            proposed_rotation = delta_rotation @ rotation
            proposed_translation = delta_rotation @ translation + delta_translation
            if (
                self.max_iteration_rotation_rad > 0.0
                and delta_angle > self.max_iteration_rotation_rad
            ):
                self._remember_prior_rejection(
                    proposed_rotation, "single ICP rotation step rejected"
                )
                return None
            if not self._transform_allowed(proposed_rotation):
                self._remember_prior_rejection(
                    proposed_rotation, "ICP left the allowed heading/tilt region"
                )
                return None
            rotation, translation = proposed_rotation, proposed_translation
            rmse = float(np.sqrt(np.mean(np.square(distances[mask]))))
            if (
                last_rmse is not None
                and abs(last_rmse - rmse) < 1e-5
                and np.linalg.norm(delta_translation) < self.convergence_translation_m
                and delta_angle < self.convergence_rotation_rad
            ):
                break
            last_rmse = rmse

        distances, _ = tree.query(transform_points(source, rotation, translation), k=1)
        mask = distances <= self.max_correspondence_distance
        correspondences = int(np.count_nonzero(mask))
        if correspondences < self.min_correspondences:
            return None
        return (
            rotation,
            translation,
            float(np.sqrt(np.mean(np.square(distances[mask])))),
            float(correspondences) / float(len(source)),
            correspondences,
            label,
        )

    def _run_icp(self, target: np.ndarray, source: np.ndarray):
        if (
            target.ndim == 2
            and source.ndim == 2
            and target.shape[1:] == source.shape[1:] == (2,)
        ):
            return super()._run_icp(target, source)
        self._last_prior_rejection = None
        if (
            target.ndim != 2
            or source.ndim != 2
            or target.shape[1:] != (3,)
            or source.shape[1:] != (3,)
            or len(target) < self.min_correspondences
            or len(source) < self.min_correspondences
        ):
            return None

        candidates: List[Tuple[tuple, tuple]] = []
        target_centroid, source_centroid = np.mean(target, axis=0), np.mean(source, axis=0)
        for published_yaw in initial_yaw_candidates(
            self.expected_yaw_rad,
            self.max_yaw_deviation_rad,
            self.initial_yaw_offsets_rad,
        ):
            raw_yaw = -published_yaw if self.invert_result else published_yaw
            initial_rotation = yaw_rotation_matrix(raw_yaw, 3)
            translations = [("origin", np.zeros(3))]
            if self.initialize_from_centroids:
                centroid = target_centroid - source_centroid @ initial_rotation.T
                if np.linalg.norm(centroid) > 1e-9:
                    translations.append(("centroid", centroid))
            for translation_label, initial_translation in translations:
                label = "yaw=%.1fdeg/%s" % (
                    math.degrees(published_yaw),
                    translation_label,
                )
                result = self._run_xyz_icp_once(
                    target, source, initial_rotation, initial_translation, label
                )
                if result is None:
                    continue
                rank = registration_rank(
                    fitness=result[3],
                    rmse=result[2],
                    correspondences=result[4],
                    yaw=self._published_yaw(result[0]),
                    expected_yaw=self.expected_yaw_rad,
                    max_deviation=self.max_yaw_deviation_rad,
                    heading_prior_weight=self.heading_prior_weight,
                    enforce_prior=self.enforce_heading_prior,
                )
                if rank is not None:
                    candidates.append((result, rank))
        return max(candidates, key=lambda item: item[1])[0] if candidates else None

    def _set_alignment_from_result(self, result, stamp, label: str) -> str:
        if self.input_mode == "global_occupancy":
            return super()._set_alignment_from_result(result, stamp, label)
        if result is None and self._last_prior_rejection:
            self.pending_candidates.clear()
            self.get_logger().warn(
                "%s 3D ICP rejected by geometric prior; %s. It will be retried."
                % (label, self._last_prior_rejection)
            )
            return "rejected"
        if result is not None and result[0].shape == (3, 3):
            if not self._transform_allowed(result[0]):
                self.pending_candidates.clear()
                self._remember_prior_rejection(result[0], "final safety check rejected")
                self.get_logger().warn(
                    "%s 3D ICP rejected by geometric prior; %s. It will be retried."
                    % (label, self._last_prior_rejection)
                )
                return "rejected"
            raw_roll, raw_pitch, raw_yaw = rotation_rpy(result[0])
            self.get_logger().info(
                "%s 3D ICP raw candidate: x=%.3f y=%.3f z=%.3f "
                "roll=%.2fdeg pitch=%.2fdeg yaw=%.2fdeg fitness=%.3f "
                "rmse=%.3f correspondences=%d init=%s"
                % (
                    label,
                    *result[1],
                    math.degrees(raw_roll),
                    math.degrees(raw_pitch),
                    math.degrees(raw_yaw),
                    result[3],
                    result[2],
                    result[4],
                    result[5],
                )
            )
        status = InitialXyIcpAlignment._set_alignment_from_result(
            self, result, stamp, label
        )
        if status == "accepted" and result is not None and result[0].shape == (3, 3):
            rotation, translation = result[0], result[1]
            if self.invert_result:
                rotation, translation = invert_transform(rotation, translation)
            _, _, yaw = rotation_rpy(rotation)
            self.get_logger().info(
                "%s accepted full XYZ registration; published_planar="
                "(x=%.3f y=%.3f z=0 yaw=%.2fdeg)."
                % (label, translation[0], translation[1], math.degrees(yaw))
            )
        return status

    def _try_compute_alignment(self, stamp) -> None:
        if self.failed or (
            self.alignment_msg is not None and self.lock_after_first_alignment
        ):
            return
        if self.robot0_points is None or self.robot1_points is None:
            return
        refreshing = self.alignment_msg is not None
        result = self._run_icp(self.robot0_points, self.robot1_points)
        status = self._set_alignment_from_result(
            result,
            stamp,
            "Refreshed cropped-cloud 3D" if refreshing else "Initial cropped-cloud 3D",
        )
        if status == "accepted":
            self.last_recompute_ns = self.get_clock().now().nanoseconds
            if not self.lock_after_first_alignment:
                self._reset_cloud_samples()
            return
        if status == "pending" or self.retry_on_failure:
            self._reset_cloud_samples()
            self.get_logger().info(
                "Collecting a fresh pair of cropped XYZ submaps for the next ICP attempt."
            )
        else:
            self.failed = True
            self.get_logger().error(
                "Initial cropped-cloud 3D ICP failed and retry_on_failure is false."
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CroppedXyzInitialIcpAlignment()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
