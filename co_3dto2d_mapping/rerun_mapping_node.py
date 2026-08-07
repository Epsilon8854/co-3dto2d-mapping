#!/usr/bin/env python3
import math
import shutil
import struct
import subprocess
import sys
import time
from collections import defaultdict

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


def quaternion_yaw(quaternion):
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


def apply_planar_transform(points, x, y, yaw):
    points = np.asarray(points, dtype=np.float32)
    if points.size == 0:
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
        self.declare_parameter("odometry_point_radius", 0.04)

        self.rr = _load_rerun()
        self.spawn_viewer = bool(self.get_parameter("spawn_viewer").value)
        self.rerun_port = int(self.get_parameter("rerun_port").value)
        self.occupancy_point_radius = float(
            self.get_parameter("occupancy_point_radius").value
        )
        self.slice_point_radius = float(
            self.get_parameter("slice_point_radius").value
        )
        self.odometry_point_radius = float(
            self.get_parameter("odometry_point_radius").value
        )
        self.alignment = (0.0, 0.0, 0.0)
        self.alignment_ready = False
        self.raw_trajectories = defaultdict(list)
        self.subscriptions = []

        self._setup_rerun()
        self._setup_subscriptions()
        self.get_logger().info("Standalone mapping Rerun visualization started.")

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
                    Odometry,
                    f"/toy_record/r{robot_id}/odom",
                    lambda msg, rid=robot_id: self._odometry_callback(msg, rid),
                    50,
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
        return points

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

    def _slice_callback(self, msg, robot_id, kind):
        points = pointcloud_xyz(msg)
        points = self._apply_robot_alignment(points, robot_id)
        path = f"mapping/slices/r{robot_id}/{kind}"
        if len(points) == 0:
            self.rr.log(path, self.rr.Clear(recursive=True), static=False)
            return
        self._set_time(msg.header.stamp)
        color = [40, 120, 255, 220] if kind == "kept" else [255, 40, 40, 180]
        self.rr.log(
            path,
            self.rr.Points3D(
                points,
                radii=[self.slice_point_radius] * len(points),
                colors=[color] * len(points),
            ),
            static=False,
        )

    def _odometry_callback(self, msg, robot_id):
        position = np.asarray(
            [[
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ]],
            dtype=np.float32,
        )
        self.raw_trajectories[robot_id].append(position[0].tolist())
        raw_trajectory = np.asarray(
            self.raw_trajectories[robot_id], dtype=np.float32
        )
        trajectory = self._apply_robot_alignment(raw_trajectory, robot_id)
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
