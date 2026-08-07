#!/usr/bin/env python3

import math
from copy import deepcopy
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster


KnownCell = Tuple[float, float, int]
PlanarAlignment = Optional[Tuple[float, float, float]]


def quaternion_yaw(q: Quaternion) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def known_cell_centers(
    msg: OccupancyGrid, occupied_threshold: int
) -> List[KnownCell]:
    width = int(msg.info.width)
    height = int(msg.info.height)
    resolution = float(msg.info.resolution)
    if width <= 0 or height <= 0 or resolution <= 0.0:
        return []
    if len(msg.data) != width * height:
        return []

    data = np.asarray(msg.data, dtype=np.int16).reshape((height, width))
    rows, cols = np.nonzero(data >= 0)
    if len(rows) == 0:
        return []

    local_x = (cols.astype(np.float64) + 0.5) * resolution
    local_y = (rows.astype(np.float64) + 0.5) * resolution
    origin = msg.info.origin
    yaw = quaternion_yaw(origin.orientation)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    world_x = origin.position.x + cos_yaw * local_x - sin_yaw * local_y
    world_y = origin.position.y + sin_yaw * local_x + cos_yaw * local_y
    values = np.where(data[rows, cols] > occupied_threshold, 100, 0)
    return [
        (float(x), float(y), int(value))
        for x, y, value in zip(world_x, world_y, values)
    ]


def apply_planar_alignment(
    point: Tuple[float, float], alignment: Tuple[float, float, float]
) -> Tuple[float, float]:
    x_align, y_align, yaw_align = alignment
    cos_yaw = math.cos(yaw_align)
    sin_yaw = math.sin(yaw_align)
    return (
        x_align + cos_yaw * point[0] - sin_yaw * point[1],
        y_align + sin_yaw * point[0] + cos_yaw * point[1],
    )


def merge_global_grids(
    global_maps: Dict[int, OccupancyGrid],
    alignment: PlanarAlignment,
    common_frame_id: str,
    merged_padding_m: float,
    occupied_threshold: int,
    stamp,
) -> Optional[OccupancyGrid]:
    known_cells: List[KnownCell] = []
    resolution: Optional[float] = None
    for robot_id, msg in global_maps.items():
        cells = known_cell_centers(msg, occupied_threshold)
        if not cells:
            continue
        if robot_id != 0:
            if alignment is None:
                continue
            cells = [
                (*apply_planar_alignment((x, y), alignment), value)
                for x, y, value in cells
            ]

        grid_resolution = float(msg.info.resolution)
        if resolution is None:
            resolution = grid_resolution
        elif not math.isclose(grid_resolution, resolution):
            continue
        known_cells.extend(cells)

    if not known_cells or resolution is None or resolution <= 0.0:
        return None

    points = np.asarray([(x, y) for x, y, _ in known_cells], dtype=np.float64)
    padding = max(0.0, float(merged_padding_m))
    min_x = math.floor((float(np.min(points[:, 0])) - padding) / resolution) * resolution
    min_y = math.floor((float(np.min(points[:, 1])) - padding) / resolution) * resolution
    max_x = math.ceil((float(np.max(points[:, 0])) + padding) / resolution) * resolution
    max_y = math.ceil((float(np.max(points[:, 1])) + padding) / resolution) * resolution
    width = max(1, int(math.ceil((max_x - min_x) / resolution)))
    height = max(1, int(math.ceil((max_y - min_y) / resolution)))

    merged = OccupancyGrid()
    merged.header.stamp = stamp
    merged.header.frame_id = common_frame_id
    merged.info.resolution = resolution
    merged.info.width = width
    merged.info.height = height
    merged.info.origin.position.x = min_x
    merged.info.origin.position.y = min_y
    merged.info.origin.position.z = 0.0
    merged.info.origin.orientation.w = 1.0
    data = np.full(width * height, -1, dtype=np.int8)

    for output_value in (0, 100):
        selected = np.asarray(
            [(x, y) for x, y, value in known_cells if value == output_value],
            dtype=np.float64,
        )
        if selected.size == 0:
            continue
        cols = np.floor((selected[:, 0] - min_x) / resolution).astype(np.int64)
        rows = np.floor((selected[:, 1] - min_y) / resolution).astype(np.int64)
        valid = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
        indices = rows[valid] * width + cols[valid]
        data[indices] = output_value

    merged.data = data.tolist()
    return merged


class ToyRecordRepublisher(Node):
    def __init__(self) -> None:
        super().__init__("toy_record_republisher")
        self.declare_parameter("target_frame_id", "odom")
        self.declare_parameter("common_frame_id", "map")
        self.declare_parameter("alignment_topic", "/toy/initial_xy_alignment")
        self.declare_parameter("publish_period_ms", 200)
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("merged_padding_m", 1.0)
        self.declare_parameter("robot_ids", [0, 1])
        self.declare_parameter("output_prefix", "/toy_record")
        self.declare_parameter("robot_odom_frame_format", "r{robot_id}/odom")
        self.declare_parameter("robot_base_frame_format", "r{robot_id}/base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_merged_global", False)

        self.target_frame_id = str(self.get_parameter("target_frame_id").value)
        self.common_frame_id = str(self.get_parameter("common_frame_id").value)
        self.alignment_topic = str(self.get_parameter("alignment_topic").value)
        self.publish_period_ms = max(
            1, int(self.get_parameter("publish_period_ms").value)
        )
        self.occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self.merged_padding_m = max(
            0.0, float(self.get_parameter("merged_padding_m").value)
        )
        self.robot_ids = [int(robot_id) for robot_id in self.get_parameter("robot_ids").value]
        self.output_prefix = str(self.get_parameter("output_prefix").value).rstrip("/")
        self.robot_odom_frame_format = str(
            self.get_parameter("robot_odom_frame_format").value
        )
        self.robot_base_frame_format = str(
            self.get_parameter("robot_base_frame_format").value
        )
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.publish_merged_global = bool(
            self.get_parameter("publish_merged_global").value
        )

        self.alignment: Optional[Tuple[float, float, float]] = None
        self.alignment_transform: Optional[TransformStamped] = None
        self.last_logged_alignment: Optional[Tuple[float, float, float]] = None
        self.latest_odom: Dict[int, Odometry] = {}
        self.latest_local_maps: Dict[int, OccupancyGrid] = {}
        self.latest_global_maps: Dict[int, OccupancyGrid] = {}
        self.latest_slice_kept_points: Dict[int, PointCloud2] = {}
        self.latest_slice_rejected_points: Dict[int, PointCloud2] = {}

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            TransformStamped,
            self.alignment_topic,
            self.alignment_callback,
            transient_qos,
        )

        self.odom_pubs = {}
        self.local_map_pubs = {}
        self.global_map_pubs = {}
        self.slice_kept_points_pubs = {}
        self.slice_rejected_points_pubs = {}
        for robot_id in self.robot_ids:
            self.create_subscription(
                Odometry,
                f"/r{robot_id}/odom",
                lambda msg, rid=robot_id: self.odom_callback(msg, rid),
                10,
            )
            self.create_subscription(
                OccupancyGrid,
                f"/r{robot_id}/toy/local_occupancy",
                lambda msg, rid=robot_id: self.local_map_callback(msg, rid),
                10,
            )
            self.create_subscription(
                OccupancyGrid,
                f"/r{robot_id}/toy/global_occupancy",
                lambda msg, rid=robot_id: self.global_map_callback(msg, rid),
                10,
            )
            self.create_subscription(
                PointCloud2,
                f"/r{robot_id}/toy/slice_kept_points",
                lambda msg, rid=robot_id: self.slice_kept_points_callback(msg, rid),
                10,
            )
            self.create_subscription(
                PointCloud2,
                f"/r{robot_id}/toy/slice_rejected_points",
                lambda msg, rid=robot_id: self.slice_rejected_points_callback(msg, rid),
                10,
            )
            self.odom_pubs[robot_id] = self.create_publisher(
                Odometry, f"{self.output_prefix}/r{robot_id}/odom", 10
            )
            self.local_map_pubs[robot_id] = self.create_publisher(
                OccupancyGrid, f"{self.output_prefix}/r{robot_id}/local_occupancy", 10
            )
            self.global_map_pubs[robot_id] = self.create_publisher(
                OccupancyGrid, f"{self.output_prefix}/r{robot_id}/global_occupancy", 10
            )
            self.slice_kept_points_pubs[robot_id] = self.create_publisher(
                PointCloud2, f"{self.output_prefix}/r{robot_id}/slice_kept_points", 10
            )
            self.slice_rejected_points_pubs[robot_id] = self.create_publisher(
                PointCloud2, f"{self.output_prefix}/r{robot_id}/slice_rejected_points", 10
            )

        self.merged_global_pub = None
        if self.publish_merged_global:
            self.merged_global_pub = self.create_publisher(
                OccupancyGrid, f"{self.output_prefix}/merged_global_occupancy", 10
            )
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.timer = self.create_timer(
            self.publish_period_ms / 1000.0, self.publish_outputs
        )
        self.get_logger().info(
            "Toy record republisher started. output_prefix=%s common_frame=%s period=%dms publish_tf=%s publish_merged_global=%s"
            % (
                self.output_prefix,
                self.common_frame_id,
                self.publish_period_ms,
                "true" if self.publish_tf else "false",
                "true" if self.publish_merged_global else "false",
            )
        )

    def alignment_callback(self, msg: TransformStamped) -> None:
        yaw = self.yaw_from_quaternion(msg.transform.rotation)
        self.alignment = (
            float(msg.transform.translation.x),
            float(msg.transform.translation.y),
            yaw,
        )
        self.alignment_transform = deepcopy(msg)
        if self.should_log_alignment(self.alignment):
            self.last_logged_alignment = self.alignment
            self.get_logger().info(
                "Received record alignment: x=%.3f y=%.3f yaw=%.3fdeg"
                % (
                    self.alignment[0],
                    self.alignment[1],
                    math.degrees(self.alignment[2]),
                )
            )

    def odom_callback(self, msg: Odometry, robot_id: int) -> None:
        self.latest_odom[robot_id] = msg

    def local_map_callback(self, msg: OccupancyGrid, robot_id: int) -> None:
        self.latest_local_maps[robot_id] = msg

    def global_map_callback(self, msg: OccupancyGrid, robot_id: int) -> None:
        self.latest_global_maps[robot_id] = msg

    def slice_kept_points_callback(self, msg: PointCloud2, robot_id: int) -> None:
        self.latest_slice_kept_points[robot_id] = msg

    def slice_rejected_points_callback(self, msg: PointCloud2, robot_id: int) -> None:
        self.latest_slice_rejected_points[robot_id] = msg

    def publish_outputs(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for robot_id, msg in self.latest_odom.items():
            output = self.copy_odom(msg, stamp, robot_id)
            self.odom_pubs[robot_id].publish(output)

        for robot_id, msg in self.latest_local_maps.items():
            self.local_map_pubs[robot_id].publish(self.copy_grid(msg, stamp, robot_id))
        for robot_id, msg in self.latest_global_maps.items():
            self.global_map_pubs[robot_id].publish(self.copy_grid(msg, stamp, robot_id))
        for robot_id, msg in self.latest_slice_kept_points.items():
            self.slice_kept_points_pubs[robot_id].publish(self.copy_cloud(msg, stamp, robot_id))
        for robot_id, msg in self.latest_slice_rejected_points.items():
            self.slice_rejected_points_pubs[robot_id].publish(
                self.copy_cloud(msg, stamp, robot_id)
            )

        if self.publish_tf:
            self.publish_transforms(stamp)

        merged = self.build_merged_global(stamp) if self.publish_merged_global else None
        if merged is not None and self.merged_global_pub is not None:
            self.merged_global_pub.publish(merged)

    def copy_odom(self, msg: Odometry, stamp, robot_id: int) -> Odometry:
        output = Odometry()
        output.header = deepcopy(msg.header)
        output.header.stamp = stamp
        output.header.frame_id = self.odom_frame(robot_id)
        output.child_frame_id = self.base_frame(robot_id)
        output.pose = deepcopy(msg.pose)
        output.twist = deepcopy(msg.twist)
        return output

    def transform_odom(self, msg: Odometry, stamp, robot_id: int) -> Odometry:
        x_align, y_align, yaw_align = self.alignment
        cos_yaw = math.cos(yaw_align)
        sin_yaw = math.sin(yaw_align)
        pose = msg.pose.pose

        output = self.copy_odom(msg, stamp, robot_id)
        output.pose.pose.position.x = (
            x_align + cos_yaw * pose.position.x - sin_yaw * pose.position.y
        )
        output.pose.pose.position.y = (
            y_align + sin_yaw * pose.position.x + cos_yaw * pose.position.y
        )
        output.pose.pose.position.z = pose.position.z
        output.pose.pose.orientation = self.multiply_quaternion(
            self.quaternion_from_yaw(yaw_align), pose.orientation
        )
        return output

    def copy_grid(self, msg: OccupancyGrid, stamp, robot_id: int) -> OccupancyGrid:
        output = OccupancyGrid()
        output.header = deepcopy(msg.header)
        output.header.stamp = stamp
        if not output.header.frame_id or output.header.frame_id == self.target_frame_id:
            output.header.frame_id = self.odom_frame(robot_id)
        output.info = deepcopy(msg.info)
        output.data = list(msg.data)
        return output

    def copy_cloud(self, msg: PointCloud2, stamp, robot_id: int) -> PointCloud2:
        output = deepcopy(msg)
        output.header.stamp = stamp
        if not output.header.frame_id or output.header.frame_id == self.target_frame_id:
            output.header.frame_id = self.odom_frame(robot_id)
        return output

    def build_merged_global(self, stamp) -> Optional[OccupancyGrid]:
        return merge_global_grids(
            self.latest_global_maps,
            self.alignment,
            self.common_frame_id,
            self.merged_padding_m,
            self.occupied_threshold,
            stamp,
        )

    def publish_transforms(self, stamp) -> None:
        transforms = [
            self.identity_transform(stamp, self.common_frame_id, self.odom_frame(0))
        ]
        if self.alignment_transform is not None:
            alignment = deepcopy(self.alignment_transform)
            alignment.header.stamp = stamp
            alignment.header.frame_id = self.common_frame_id
            alignment.child_frame_id = self.odom_frame(1)
            transforms.append(alignment)

        for robot_id, msg in self.latest_odom.items():
            transforms.append(self.odom_to_base_transform(msg, stamp, robot_id))

        for transform in transforms:
            self.tf_broadcaster.sendTransform(transform)

    def identity_transform(self, stamp, parent: str, child: str) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.rotation.w = 1.0
        return transform

    def odom_to_base_transform(
        self, msg: Odometry, stamp, robot_id: int
    ) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame(robot_id)
        transform.child_frame_id = self.base_frame(robot_id)
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = deepcopy(msg.pose.pose.orientation)
        return transform

    def apply_alignment(self, point: Tuple[float, float]) -> Tuple[float, float]:
        x_align, y_align, yaw_align = self.alignment
        cos_yaw = math.cos(yaw_align)
        sin_yaw = math.sin(yaw_align)
        return (
            x_align + cos_yaw * point[0] - sin_yaw * point[1],
            y_align + sin_yaw * point[0] + cos_yaw * point[1],
        )

    def odom_frame(self, robot_id: int) -> str:
        return self.robot_odom_frame_format.format(robot_id=robot_id)

    def base_frame(self, robot_id: int) -> str:
        return self.robot_base_frame_format.format(robot_id=robot_id)

    def should_log_alignment(self, alignment: Tuple[float, float, float]) -> bool:
        if self.last_logged_alignment is None:
            return True
        dx = alignment[0] - self.last_logged_alignment[0]
        dy = alignment[1] - self.last_logged_alignment[1]
        dyaw = math.atan2(
            math.sin(alignment[2] - self.last_logged_alignment[2]),
            math.cos(alignment[2] - self.last_logged_alignment[2]),
        )
        return math.hypot(dx, dy) > 0.01 or abs(dyaw) > math.radians(0.1)

    @staticmethod
    def yaw_from_quaternion(q: Quaternion) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    @staticmethod
    def quaternion_from_yaw(yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(0.5 * yaw)
        q.w = math.cos(0.5 * yaw)
        return q

    @staticmethod
    def multiply_quaternion(a: Quaternion, b: Quaternion) -> Quaternion:
        q = Quaternion()
        q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y
        q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x
        q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
        q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
        return q


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ToyRecordRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
