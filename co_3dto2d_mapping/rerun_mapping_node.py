#!/usr/bin/env python3
"""Rerun visualization for the plane-relative, ground-fused mapping pipeline.

The visualization consumes the final per-robot odometry from ``/toy_record``
and uses the full quaternion to place the local plane-height-filtered cloud in
3-D. This avoids the old behavior where only x/y/yaw were visualized and the
legacy slice clouds could appear detached from the ground-fused pose.
"""

from __future__ import annotations

import math
import shutil
import struct
import subprocess
import sys
import time
from collections import defaultdict
from typing import Dict, Iterable, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


def quaternion_yaw(quaternion) -> float:
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


def quaternion_rotation_matrix(quaternion) -> np.ndarray:
    """Return the active 3-D rotation represented by a ROS quaternion."""

    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 1e-12:
        return np.eye(3, dtype=np.float32)
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    return np.asarray(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float32,
    )


def apply_pose_transform(points, position, quaternion) -> np.ndarray:
    """Transform body-frame points with the full odometry pose."""

    points = np.asarray(points, dtype=np.float32).reshape((-1, 3))
    if len(points) == 0:
        return np.empty((0, 3), dtype=np.float32)
    translation = np.asarray(position, dtype=np.float32).reshape(3)
    rotation = quaternion_rotation_matrix(quaternion)
    return points @ rotation.T + translation


def apply_planar_transform(points, x, y, yaw) -> np.ndarray:
    points = np.asarray(points, dtype=np.float32).reshape((-1, 3))
    if len(points) == 0:
        return np.empty((0, 3), dtype=np.float32)
    result = points.copy()
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    result[:, 0] = x + cos_yaw * points[:, 0] - sin_yaw * points[:, 1]
    result[:, 1] = y + sin_yaw * points[:, 0] + cos_yaw * points[:, 1]
    return result


def occupied_cell_centers(msg, occupied_threshold=50):
    width = int(msg.info.width)
    height = int(msg.info.height)
    resolution = float(msg.info.resolution)
    if (
        width <= 0
        or height <= 0
        or resolution <= 0.0
        or len(msg.data) != width * height
    ):
        return np.empty((0, 3), dtype=np.float32)

    data = np.asarray(msg.data, dtype=np.int16)
    indices = np.flatnonzero(data > occupied_threshold)
    if len(indices) == 0:
        return np.empty((0, 3), dtype=np.float32)

    rows = indices // width
    cols = indices % width
    local_x = (cols.astype(np.float64) + 0.5) * resolution
    local_y = (rows.astype(np.float64) + 0.5) * resolution
    yaw = quaternion_yaw(msg.info.origin.orientation)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    origin = msg.info.origin.position
    world_x = origin.x + cos_yaw * local_x - sin_yaw * local_y
    world_y = origin.y + sin_yaw * local_x + cos_yaw * local_y
    world_z = np.full_like(world_x, origin.z, dtype=np.float64)
    return np.column_stack((world_x, world_y, world_z)).astype(np.float32)


def pointcloud_xyz(msg):
    field_by_name = {field.name: field for field in msg.fields}
    if not all(name in field_by_name for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float32)

    formats = {
        PointField.FLOAT32: "f",
        PointField.FLOAT64: "d",
    }
    fields = [field_by_name[name] for name in ("x", "y", "z")]
    if any(field.datatype not in formats for field in fields):
        return np.empty((0, 3), dtype=np.float32)

    endian = ">" if msg.is_bigendian else "<"
    raw = bytes(msg.data)
    row_step = int(msg.row_step) or int(msg.point_step) * int(msg.width)
    points = []
    for row in range(int(msg.height)):
        row_offset = row * row_step
        for col in range(int(msg.width)):
            point_offset = row_offset + col * int(msg.point_step)
            xyz = []
            valid = True
            for field in fields:
                fmt = endian + formats[field.datatype]
                size = struct.calcsize(fmt)
                offset = point_offset + int(field.offset)
                if offset + size > len(raw):
                    valid = False
                    break
                xyz.append(struct.unpack_from(fmt, raw, offset)[0])
            if valid and all(math.isfinite(value) for value in xyz):
                points.append(xyz)
    return np.asarray(points, dtype=np.float32).reshape((-1, 3))


def _load_rerun():
    try:
        import rerun as rr
    except Exception as exc:
        version = f"{sys.version_info.major}.{sys.version_info.minor}"
        raise RuntimeError(
            "rerun could not be imported under Python "
            f"{version}; install a compatible rerun-sdk build"
        ) from exc
    return rr


class RerunMappingNode(Node):
    def __init__(self):
        super().__init__("co_3dto2d_rerun")
        self.declare_parameter("spawn_viewer", True)
        self.declare_parameter("rerun_port", 9876)
        self.declare_parameter("occupancy_point_radius", 0.045)
        self.declare_parameter("slice_point_radius", 0.025)
        self.declare_parameter("plane_height_point_radius", 0.025)
        self.declare_parameter("odometry_point_radius", 0.04)
        self.declare_parameter("robot_axis_length_m", 0.60)
        self.declare_parameter("max_trajectory_points", 5000)
        self.declare_parameter("visualize_plane_height_cloud", True)
        self.declare_parameter("visualize_legacy_slice_points", False)
        self.declare_parameter(
            "plane_height_cloud_topic_format",
            "/r{robot_id}/mapping/plane_height_filtered",
        )

        self.rr = _load_rerun()
        self.spawn_viewer = bool(self.get_parameter("spawn_viewer").value)
        self.rerun_port = int(self.get_parameter("rerun_port").value)
        self.occupancy_point_radius = float(
            self.get_parameter("occupancy_point_radius").value
        )
        self.slice_point_radius = float(
            self.get_parameter("slice_point_radius").value
        )
        self.plane_height_point_radius = float(
            self.get_parameter("plane_height_point_radius").value
        )
        self.odometry_point_radius = float(
            self.get_parameter("odometry_point_radius").value
        )
        self.robot_axis_length_m = float(
            self.get_parameter("robot_axis_length_m").value
        )
        self.max_trajectory_points = int(
            self.get_parameter("max_trajectory_points").value
        )
        self.visualize_plane_height_cloud = bool(
            self.get_parameter("visualize_plane_height_cloud").value
        )
        self.visualize_legacy_slice_points = bool(
            self.get_parameter("visualize_legacy_slice_points").value
        )
        self.plane_height_cloud_topic_format = str(
            self.get_parameter("plane_height_cloud_topic_format").value
        )

        for name, value in (
            ("occupancy_point_radius", self.occupancy_point_radius),
            ("slice_point_radius", self.slice_point_radius),
            ("plane_height_point_radius", self.plane_height_point_radius),
            ("odometry_point_radius", self.odometry_point_radius),
            ("robot_axis_length_m", self.robot_axis_length_m),
        ):
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
        if self.max_trajectory_points < 2:
            raise ValueError("max_trajectory_points must be at least two")
        if "{robot_id}" not in self.plane_height_cloud_topic_format:
            raise ValueError(
                "plane_height_cloud_topic_format must contain {robot_id}"
            )

        self.alignment = (0.0, 0.0, 0.0)
        self.alignment_ready = False
        self.local_trajectories = defaultdict(list)
        self.latest_robot_pose: Dict[int, Tuple[np.ndarray, object]] = {}
        self._missing_pose_warned = set()
        self.subscriptions = []

        self._setup_rerun()
        self._setup_subscriptions()
        self.get_logger().info(
            "Standalone Rerun visualization started. plane_cloud=%s "
            "legacy_slices=%s"
            % (
                "true" if self.visualize_plane_height_cloud else "false",
                "true" if self.visualize_legacy_slice_points else "false",
            )
        )

    def _setup_rerun(self):
        self.rr.init("co_3dto2d_mapping", spawn=False)
        if self.spawn_viewer:
            rerun_binary = shutil.which("rerun")
            if rerun_binary is None:
                raise RuntimeError("rerun CLI was not found in PATH")
            subprocess.Popen(
                [
                    rerun_binary,
                    "--detach-process",
                    "--port",
                    str(self.rerun_port),
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            time.sleep(1.0)

        if hasattr(self.rr, "connect_grpc"):
            self.rr.connect_grpc(
                f"rerun+http://127.0.0.1:{self.rerun_port}/proxy"
            )
        elif hasattr(self.rr, "connect_tcp"):
            self.rr.connect_tcp(f"127.0.0.1:{self.rerun_port}")
        else:
            self.rr.connect(f"127.0.0.1:{self.rerun_port}")
        self.rr.log(
            "mapping",
            self.rr.ViewCoordinates.RIGHT_HAND_Z_UP,
            static=True,
        )

    def _setup_subscriptions(self):
        occupancy_topics = (
            (0, "r0", "/toy_record/r0/global_occupancy"),
            (1, "r1", "/toy_record/r1/global_occupancy"),
            (-1, "merged", "/toy_record/merged_global_occupancy"),
        )
        for robot_id, label, topic in occupancy_topics:
            self.subscriptions.append(
                self.create_subscription(
                    OccupancyGrid,
                    topic,
                    lambda msg, rid=robot_id, name=label: self._occupancy_callback(
                        msg, rid, name
                    ),
                    10,
                )
            )

        for robot_id in (0, 1):
            self.subscriptions.append(
                self.create_subscription(
                    Odometry,
                    f"/toy_record/r{robot_id}/odom",
                    lambda msg, rid=robot_id: self._odometry_callback(msg, rid),
                    50,
                )
            )
            if self.visualize_plane_height_cloud:
                topic = self.plane_height_cloud_topic_format.format(
                    robot_id=robot_id
                )
                self.subscriptions.append(
                    self.create_subscription(
                        PointCloud2,
                        topic,
                        lambda msg, rid=robot_id: self._plane_height_callback(
                            msg, rid
                        ),
                        qos_profile_sensor_data,
                    )
                )
            if self.visualize_legacy_slice_points:
                for kind in ("kept", "rejected"):
                    topic = f"/toy_record/r{robot_id}/slice_{kind}_points"
                    self.subscriptions.append(
                        self.create_subscription(
                            PointCloud2,
                            topic,
                            lambda msg, rid=robot_id, name=kind: self._slice_callback(
                                msg, rid, name
                            ),
                            10,
                        )
                    )

        self.subscriptions.append(
            self.create_subscription(
                TransformStamped,
                "/toy/initial_xy_alignment",
                self._alignment_callback,
                10,
            )
        )

    def _apply_robot_alignment(self, points, robot_id):
        if robot_id == 1 and self.alignment_ready:
            return apply_planar_transform(points, *self.alignment)
        return np.asarray(points, dtype=np.float32).reshape((-1, 3))

    def _set_time(self, stamp):
        seconds = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if seconds > 0.0 and hasattr(self.rr, "set_time_seconds"):
            self.rr.set_time_seconds("ros_time", seconds)

    def _occupancy_callback(self, msg, robot_id, label):
        points = occupied_cell_centers(msg)
        points = self._apply_robot_alignment(points, robot_id)
        path = f"mapping/occupancy/{label}"
        if len(points) == 0:
            self.rr.log(path, self.rr.Clear(recursive=True), static=False)
            return
        self._set_time(msg.header.stamp)
        colors = {
            "r0": [45, 160, 255, 210],
            "r1": [255, 130, 35, 210],
            "merged": [235, 235, 235, 230],
        }
        self.rr.log(
            path,
            self.rr.Points3D(
                points,
                radii=[self.occupancy_point_radius] * len(points),
                colors=[colors[label]] * len(points),
            ),
            static=False,
        )

    def _plane_height_callback(self, msg, robot_id):
        local_points = pointcloud_xyz(msg)
        path = f"mapping/plane_height_cloud/r{robot_id}"
        if len(local_points) == 0:
            self.rr.log(path, self.rr.Clear(recursive=True), static=False)
            return

        pose = self.latest_robot_pose.get(robot_id)
        if pose is None:
            if robot_id not in self._missing_pose_warned:
                self._missing_pose_warned.add(robot_id)
                self.get_logger().warn(
                    "Waiting for /toy_record/r%d/odom before visualizing the "
                    "local plane-height cloud." % robot_id
                )
            return

        position, orientation = pose
        points = apply_pose_transform(local_points, position, orientation)
        points = self._apply_robot_alignment(points, robot_id)
        color = [45, 160, 255, 225] if robot_id == 0 else [255, 130, 35, 225]
        self._set_time(msg.header.stamp)
        self.rr.log(
            path,
            self.rr.Points3D(
                points,
                radii=[self.plane_height_point_radius] * len(points),
                colors=[color] * len(points),
            ),
            static=False,
        )

    def _slice_callback(self, msg, robot_id, kind):
        points = pointcloud_xyz(msg)
        points = self._apply_robot_alignment(points, robot_id)
        path = f"mapping/legacy_slices/r{robot_id}/{kind}"
        if len(points) == 0:
            self.rr.log(path, self.rr.Clear(recursive=True), static=False)
            return
        self._set_time(msg.header.stamp)
        color = [40, 120, 255, 180] if kind == "kept" else [255, 40, 40, 150]
        self.rr.log(
            path,
            self.rr.Points3D(
                points,
                radii=[self.slice_point_radius] * len(points),
                colors=[color] * len(points),
            ),
            static=False,
        )

    def _robot_axes(self, position, orientation, robot_id) -> Iterable[np.ndarray]:
        origin = np.asarray(position, dtype=np.float32).reshape(3)
        rotation = quaternion_rotation_matrix(orientation)
        strips = []
        for axis in range(3):
            endpoint = origin + self.robot_axis_length_m * rotation[:, axis]
            strip = np.vstack((origin, endpoint)).astype(np.float32)
            strips.append(self._apply_robot_alignment(strip, robot_id))
        return strips

    def _odometry_callback(self, msg, robot_id):
        pose = msg.pose.pose
        position = np.asarray(
            [pose.position.x, pose.position.y, pose.position.z],
            dtype=np.float32,
        )
        self.latest_robot_pose[robot_id] = (position, pose.orientation)
        self._missing_pose_warned.discard(robot_id)

        trajectory_storage = self.local_trajectories[robot_id]
        trajectory_storage.append(position.tolist())
        if len(trajectory_storage) > self.max_trajectory_points:
            del trajectory_storage[: len(trajectory_storage) - self.max_trajectory_points]

        local_trajectory = np.asarray(trajectory_storage, dtype=np.float32)
        trajectory = self._apply_robot_alignment(local_trajectory, robot_id)
        current = trajectory[-1:]
        color = [45, 160, 255, 255] if robot_id == 0 else [255, 130, 35, 255]
        self._set_time(msg.header.stamp)
        root = f"mapping/odometry/r{robot_id}"
        self.rr.log(
            f"{root}/current",
            self.rr.Points3D(
                current,
                radii=[self.odometry_point_radius],
                colors=[color],
            ),
            static=False,
        )
        if len(trajectory) >= 2:
            self.rr.log(
                f"{root}/trajectory",
                self.rr.LineStrips3D([trajectory], colors=[color]),
                static=False,
            )

        axes = list(self._robot_axes(position, pose.orientation, robot_id))
        self.rr.log(
            f"{root}/axes",
            self.rr.LineStrips3D(
                axes,
                colors=[
                    [255, 70, 70, 255],
                    [70, 255, 90, 255],
                    [70, 130, 255, 255],
                ],
            ),
            static=False,
        )

    def _alignment_callback(self, msg):
        translation = msg.transform.translation
        self.alignment = (
            float(translation.x),
            float(translation.y),
            quaternion_yaw(msg.transform.rotation),
        )
        self.alignment_ready = True


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RerunMappingNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
