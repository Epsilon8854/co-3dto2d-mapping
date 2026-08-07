#!/usr/bin/env python3

import math
import struct
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformListener


class InitialXyIcpAlignment(Node):
    def __init__(self) -> None:
        super().__init__("initial_xy_icp_alignment")
        self.declare_parameter("input_mode", "cloud_initial")
        self.declare_parameter("robot0_cloud_topic", "/r0/livox/lidar")
        self.declare_parameter("robot1_cloud_topic", "/r1/livox/lidar")
        self.declare_parameter("robot0_map_topic", "/r0/toy/global_occupancy")
        self.declare_parameter("robot1_map_topic", "/r1/toy/global_occupancy")
        self.declare_parameter("alignment_topic", "/toy/initial_xy_alignment")
        self.declare_parameter("target_frame_id", "odom")
        self.declare_parameter("source_frame_id", "r1/odom")
        self.declare_parameter("local_frame_id", "base_link")
        self.declare_parameter("transform_cloud_to_local_frame", True)
        self.declare_parameter("z_min", 0.4)
        self.declare_parameter("z_max", 1.2)
        self.declare_parameter("invert_z_slice", True)
        self.declare_parameter("frame_count", 5)
        self.declare_parameter("invert_result", False)
        self.declare_parameter("center_box_half_extent_m", 0.0)
        self.declare_parameter("voxel_size", 0.10)
        self.declare_parameter("max_points", 30000)
        self.declare_parameter("max_correspondence_distance", 0.75)
        self.declare_parameter("min_correspondences", 100)
        self.declare_parameter("min_fitness", 0.05)
        self.declare_parameter("max_rmse", 0.40)
        self.declare_parameter("max_iterations", 80)
        self.declare_parameter("recompute_period_sec", 5.0)
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("convergence_translation_m", 1e-4)
        self.declare_parameter("convergence_rotation_rad", 1e-4)
        self.declare_parameter("publish_period_sec", 1.0)

        self.input_mode = str(self.get_parameter("input_mode").value)
        self.z_min = float(self.get_parameter("z_min").value)
        self.z_max = float(self.get_parameter("z_max").value)
        self.invert_z_slice = bool(self.get_parameter("invert_z_slice").value)
        self.frame_count = max(1, int(self.get_parameter("frame_count").value))
        self.invert_result = bool(self.get_parameter("invert_result").value)
        self.local_frame_id = str(self.get_parameter("local_frame_id").value)
        self.transform_cloud_to_local_frame = bool(
            self.get_parameter("transform_cloud_to_local_frame").value
        )
        self.center_box_half_extent_m = max(
            0.0, float(self.get_parameter("center_box_half_extent_m").value)
        )
        self.voxel_size = max(0.0, float(self.get_parameter("voxel_size").value))
        self.max_points = max(100, int(self.get_parameter("max_points").value))
        self.max_correspondence_distance = float(
            self.get_parameter("max_correspondence_distance").value
        )
        self.min_correspondences = max(
            3, int(self.get_parameter("min_correspondences").value)
        )
        self.min_fitness = float(self.get_parameter("min_fitness").value)
        self.max_rmse = float(self.get_parameter("max_rmse").value)
        self.max_iterations = max(1, int(self.get_parameter("max_iterations").value))
        self.recompute_period_sec = max(
            0.1, float(self.get_parameter("recompute_period_sec").value)
        )
        self.occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self.convergence_translation_m = float(
            self.get_parameter("convergence_translation_m").value
        )
        self.convergence_rotation_rad = float(
            self.get_parameter("convergence_rotation_rad").value
        )

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            TransformStamped,
            str(self.get_parameter("alignment_topic").value),
            qos,
        )

        self.robot0_frames: List[np.ndarray] = []
        self.robot1_frames: List[np.ndarray] = []
        self.robot0_points: Optional[np.ndarray] = None
        self.robot1_points: Optional[np.ndarray] = None
        self.robot0_map: Optional[OccupancyGrid] = None
        self.robot1_map: Optional[OccupancyGrid] = None
        self.alignment_msg: Optional[TransformStamped] = None
        self.failed = False
        self.last_recompute_ns = 0
        self.missing_transform_warnings = set()
        self.tf_buffer = None
        self.tf_listener = None
        if self.transform_cloud_to_local_frame:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        if self.input_mode == "global_occupancy":
            self.create_subscription(
                OccupancyGrid,
                str(self.get_parameter("robot0_map_topic").value),
                self._robot0_map_callback,
                10,
            )
            self.create_subscription(
                OccupancyGrid,
                str(self.get_parameter("robot1_map_topic").value),
                self._robot1_map_callback,
                10,
            )
        elif self.input_mode == "cloud_initial":
            self.create_subscription(
                PointCloud2,
                str(self.get_parameter("robot0_cloud_topic").value),
                self._robot0_callback,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                PointCloud2,
                str(self.get_parameter("robot1_cloud_topic").value),
                self._robot1_callback,
                qos_profile_sensor_data,
            )
        else:
            raise ValueError("input_mode must be 'cloud_initial' or 'global_occupancy'")
        self.timer = self.create_timer(
            max(0.1, float(self.get_parameter("publish_period_sec").value)),
            self._publish_alignment,
        )
        self.get_logger().info(
            "Waiting for XY ICP alignment input_mode=%s clouds=(%s, %s) maps=(%s, %s), frame_count=%d, "
            "invert_result=%s, local_frame=%s, transform_cloud_to_local=%s, "
            "center_box_half_extent_m=%.2f, voxel_size=%.3f, recompute_period=%.2fs"
            % (
                self.input_mode,
                str(self.get_parameter("robot0_cloud_topic").value),
                str(self.get_parameter("robot1_cloud_topic").value),
                str(self.get_parameter("robot0_map_topic").value),
                str(self.get_parameter("robot1_map_topic").value),
                self.frame_count,
                "true" if self.invert_result else "false",
                self.local_frame_id,
                "true" if self.transform_cloud_to_local_frame else "false",
                self.center_box_half_extent_m,
                self.voxel_size,
                self.recompute_period_sec,
            )
        )

    def _robot0_callback(self, msg: PointCloud2) -> None:
        if self.robot0_points is not None:
            return
        self.robot0_points = self._cache_initial_frame(
            "robot0", self.robot0_frames, msg
        )
        self._try_compute_alignment(msg.header.stamp)

    def _robot1_callback(self, msg: PointCloud2) -> None:
        if self.robot1_points is not None:
            return
        self.robot1_points = self._cache_initial_frame(
            "robot1", self.robot1_frames, msg
        )
        self._try_compute_alignment(msg.header.stamp)

    def _robot0_map_callback(self, msg: OccupancyGrid) -> None:
        self.robot0_map = msg

    def _robot1_map_callback(self, msg: OccupancyGrid) -> None:
        self.robot1_map = msg

    def _cache_initial_frame(
        self, robot_name: str, frames: List[np.ndarray], msg: PointCloud2
    ) -> Optional[np.ndarray]:
        points = self._cloud_to_xy_points(msg)
        if points is None:
            return None
        frames.append(points)
        self.get_logger().info(
            "Cached %s initial XY ICP frame %d/%d with %d points."
            % (robot_name, len(frames), self.frame_count, len(points))
        )
        if len(frames) < self.frame_count:
            return None

        non_empty_frames = [frame for frame in frames if len(frame) > 0]
        if non_empty_frames:
            merged = np.vstack(non_empty_frames)
        else:
            merged = np.empty((0, 2), dtype=np.float64)
        merged = self._prepare_icp_points(merged)
        self.get_logger().info(
            "Built %s initial XY ICP submap from %d frames with %d points."
            % (robot_name, len(frames), len(merged))
        )
        return merged

    def _lookup_cloud_to_local_transform(
        self, msg: PointCloud2
    ) -> Tuple[bool, Optional[TransformStamped]]:
        if (
            not self.transform_cloud_to_local_frame
            or not msg.header.frame_id
            or msg.header.frame_id == self.local_frame_id
        ):
            return True, None
        try:
            return True, self.tf_buffer.lookup_transform(
                self.local_frame_id, msg.header.frame_id, Time()
            )
        except Exception as exc:
            key = (self.local_frame_id, msg.header.frame_id)
            if key not in self.missing_transform_warnings:
                self.missing_transform_warnings.add(key)
                self.get_logger().warn(
                    "Waiting for transform %s <- %s before caching initial ICP frames: %s"
                    % (self.local_frame_id, msg.header.frame_id, exc)
                )
            return False, None

    def _transform_point(
        self, transform: TransformStamped, x: float, y: float, z: float
    ) -> Tuple[float, float, float]:
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm > 1e-12:
            qx /= norm
            qy /= norm
            qz /= norm
            qw /= norm

        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        rx = (1.0 - 2.0 * (yy + zz)) * x + 2.0 * (xy - wz) * y + 2.0 * (xz + wy) * z
        ry = 2.0 * (xy + wz) * x + (1.0 - 2.0 * (xx + zz)) * y + 2.0 * (yz - wx) * z
        rz = 2.0 * (xz - wy) * x + 2.0 * (yz + wx) * y + (1.0 - 2.0 * (xx + yy)) * z

        return (
            rx + transform.transform.translation.x,
            ry + transform.transform.translation.y,
            rz + transform.transform.translation.z,
        )

    def _field_unpacker(self, msg: PointCloud2, name: str) -> Tuple[int, str]:
        fields = {field.name: field for field in msg.fields}
        if name not in fields:
            raise ValueError("PointCloud2 is missing field '%s'" % name)
        field = fields[name]
        if field.datatype == PointField.FLOAT32:
            return field.offset, ("<f" if not msg.is_bigendian else ">f")
        if field.datatype == PointField.FLOAT64:
            return field.offset, ("<d" if not msg.is_bigendian else ">d")
        raise ValueError("PointCloud2 field '%s' must be FLOAT32 or FLOAT64" % name)

    def _cloud_to_xy_points(self, msg: PointCloud2) -> Optional[np.ndarray]:
        transform_ok, cloud_to_local_transform = self._lookup_cloud_to_local_transform(msg)
        if not transform_ok:
            return None
        x_offset, x_fmt = self._field_unpacker(msg, "x")
        y_offset, y_fmt = self._field_unpacker(msg, "y")
        z_offset, z_fmt = self._field_unpacker(msg, "z")
        point_step = int(msg.point_step)
        input_points = int(msg.width) * int(msg.height)
        data = bytes(msg.data)
        points = []

        for index in range(input_points):
            start = index * point_step
            point_bytes = data[start : start + point_step]
            if len(point_bytes) != point_step:
                continue
            x = struct.unpack_from(x_fmt, point_bytes, x_offset)[0]
            y = struct.unpack_from(y_fmt, point_bytes, y_offset)[0]
            z = struct.unpack_from(z_fmt, point_bytes, z_offset)[0]
            if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
                continue
            if cloud_to_local_transform is not None:
                x, y, z = self._transform_point(cloud_to_local_transform, x, y, z)
            inside_slice = self.z_min <= z <= self.z_max
            keep_point = not inside_slice if self.invert_z_slice else inside_slice
            if (
                keep_point
                and self.center_box_half_extent_m > 0.0
                and abs(x) <= self.center_box_half_extent_m
                and abs(y) <= self.center_box_half_extent_m
            ):
                keep_point = False
            if keep_point:
                points.append((x, y))

        if not points:
            return np.empty((0, 2), dtype=np.float64)
        xy = np.asarray(points, dtype=np.float64)
        return self._prepare_icp_points(xy)

    def _prepare_icp_points(self, xy: np.ndarray) -> np.ndarray:
        xy = self._voxel_downsample(xy, self.voxel_size)
        if len(xy) > self.max_points:
            stride = max(1, len(xy) // self.max_points)
            xy = xy[::stride][: self.max_points]
        return xy

    def _grid_to_xy_points(self, msg: OccupancyGrid) -> np.ndarray:
        width = int(msg.info.width)
        height = int(msg.info.height)
        if width <= 0 or height <= 0 or len(msg.data) != width * height:
            return np.empty((0, 2), dtype=np.float64)

        data = np.asarray(msg.data, dtype=np.int16).reshape((height, width))
        rows, cols = np.nonzero(data > self.occupied_threshold)
        if len(rows) == 0:
            return np.empty((0, 2), dtype=np.float64)

        resolution = float(msg.info.resolution)
        local_x = (cols.astype(np.float64) + 0.5) * resolution
        local_y = (rows.astype(np.float64) + 0.5) * resolution
        origin = msg.info.origin
        yaw = self._yaw_from_quaternion(
            origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w,
        )
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        x = origin.position.x + cos_yaw * local_x - sin_yaw * local_y
        y = origin.position.y + sin_yaw * local_x + cos_yaw * local_y
        return self._prepare_icp_points(np.column_stack((x, y)))

    def _voxel_downsample(self, points: np.ndarray, voxel_size: float) -> np.ndarray:
        if voxel_size <= 0.0 or len(points) == 0:
            return points
        keys = np.floor(points / voxel_size).astype(np.int64)
        unique_keys, inverse = np.unique(keys, axis=0, return_inverse=True)
        sums = np.zeros((len(unique_keys), 2), dtype=np.float64)
        counts = np.zeros(len(unique_keys), dtype=np.float64)
        np.add.at(sums, inverse, points)
        np.add.at(counts, inverse, 1.0)
        return sums / counts[:, None]

    def _estimate_rigid_2d(
        self, source: np.ndarray, target: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        source_mean = np.mean(source, axis=0)
        target_mean = np.mean(target, axis=0)
        source_centered = source - source_mean
        target_centered = target - target_mean
        u, _, vt = np.linalg.svd(source_centered.T @ target_centered)
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0.0:
            vt[-1, :] *= -1.0
            rotation = vt.T @ u.T
        translation = target_mean - rotation @ source_mean
        return rotation, translation

    def _run_icp(self, target: np.ndarray, source: np.ndarray):
        if len(target) < self.min_correspondences or len(source) < self.min_correspondences:
            return None

        tree = cKDTree(target)
        total_rotation = np.eye(2, dtype=np.float64)
        total_translation = np.zeros(2, dtype=np.float64)
        last_rmse = None
        best = None

        for _ in range(self.max_iterations):
            transformed = source @ total_rotation.T + total_translation
            distances, indices = tree.query(transformed, k=1)
            mask = distances <= self.max_correspondence_distance
            correspondences = int(np.count_nonzero(mask))
            if correspondences < self.min_correspondences:
                return None

            matched_source = transformed[mask]
            matched_target = target[indices[mask]]
            delta_rotation, delta_translation = self._estimate_rigid_2d(
                matched_source, matched_target
            )
            total_rotation = delta_rotation @ total_rotation
            total_translation = delta_rotation @ total_translation + delta_translation

            rmse = float(np.sqrt(np.mean(np.square(distances[mask]))))
            fitness = float(correspondences) / float(len(source))
            yaw_delta = math.atan2(delta_rotation[1, 0], delta_rotation[0, 0])
            best = (total_rotation.copy(), total_translation.copy(), rmse, fitness, correspondences)
            if (
                last_rmse is not None
                and abs(last_rmse - rmse) < 1e-5
                and np.linalg.norm(delta_translation) < self.convergence_translation_m
                and abs(yaw_delta) < self.convergence_rotation_rad
            ):
                break
            last_rmse = rmse

        return best

    def _set_alignment_from_result(
        self, result, stamp, label: str, fail_on_reject: bool
    ) -> None:
        if result is None:
            if fail_on_reject:
                self.failed = True
                log = self.get_logger().error
            else:
                log = self.get_logger().warn
            log(
                "%s XY ICP failed: not enough correspondences within %.3fm."
                % (label, self.max_correspondence_distance)
            )
            return

        raw_rotation, raw_translation, rmse, fitness, correspondences = result
        raw_yaw = math.atan2(raw_rotation[1, 0], raw_rotation[0, 0])
        if fitness < self.min_fitness or rmse > self.max_rmse:
            if fail_on_reject:
                self.failed = True
                log = self.get_logger().error
            else:
                log = self.get_logger().warn
            log(
                "%s XY ICP rejected: fitness=%.3f rmse=%.3f correspondences=%d "
                "thresholds fitness>=%.3f rmse<=%.3f"
                % (
                    label,
                    fitness,
                    rmse,
                    correspondences,
                    self.min_fitness,
                    self.max_rmse,
                )
            )
            return

        rotation = raw_rotation
        translation = raw_translation
        if self.invert_result:
            rotation = raw_rotation.T
            translation = -(rotation @ raw_translation)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])

        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = str(self.get_parameter("target_frame_id").value)
        msg.child_frame_id = str(self.get_parameter("source_frame_id").value)
        msg.transform.translation.x = float(translation[0])
        msg.transform.translation.y = float(translation[1])
        msg.transform.translation.z = 0.0
        msg.transform.rotation.z = math.sin(0.5 * yaw)
        msg.transform.rotation.w = math.cos(0.5 * yaw)
        self.alignment_msg = msg
        self.publisher.publish(msg)
        self.get_logger().info(
            "%s XY ICP alignment accepted: raw=(x=%.3f y=%.3f yaw=%.3fdeg) "
            "published=(x=%.3f y=%.3f yaw=%.3fdeg, inverted=%s) "
            "fitness=%.3f rmse=%.3f correspondences=%d"
            % (
                label,
                raw_translation[0],
                raw_translation[1],
                math.degrees(raw_yaw),
                translation[0],
                translation[1],
                math.degrees(yaw),
                "true" if self.invert_result else "false",
                fitness,
                rmse,
                correspondences,
            )
        )

    def _try_compute_alignment(self, stamp) -> None:
        if self.alignment_msg is not None or self.failed:
            return
        if self.robot0_points is None or self.robot1_points is None:
            return

        result = self._run_icp(self.robot0_points, self.robot1_points)
        self._set_alignment_from_result(result, stamp, "Initial", True)

    def _try_compute_periodic_map_alignment(self) -> None:
        if self.robot0_map is None or self.robot1_map is None:
            return

        now = self.get_clock().now()
        if self.last_recompute_ns and (
            now.nanoseconds - self.last_recompute_ns
            < int(self.recompute_period_sec * 1e9)
        ):
            return

        self.last_recompute_ns = now.nanoseconds
        robot0_points = self._grid_to_xy_points(self.robot0_map)
        robot1_points = self._grid_to_xy_points(self.robot1_map)
        if len(robot0_points) < self.min_correspondences or len(robot1_points) < self.min_correspondences:
            self.get_logger().warn(
                "Periodic map XY ICP skipped: map samples are too small r0=%d r1=%d min=%d"
                % (len(robot0_points), len(robot1_points), self.min_correspondences)
            )
            return

        result = self._run_icp(robot0_points, robot1_points)
        self._set_alignment_from_result(
            result, now.to_msg(), "Periodic map", False
        )

    def _publish_alignment(self) -> None:
        if self.input_mode == "global_occupancy":
            self._try_compute_periodic_map_alignment()
        if self.alignment_msg is not None:
            self.alignment_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(self.alignment_msg)

    @staticmethod
    def _yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
        return math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InitialXyIcpAlignment()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
