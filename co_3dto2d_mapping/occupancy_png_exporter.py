#!/usr/bin/env python3
"""Continuously save the latest two-live occupancy maps as PNG files."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, List, Optional, Set

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from co_3dto2d_mapping.occupancy_png import (
    OccupancyGridGeometry,
    OccupancyGridEncodingError,
    occupancy_grid_png,
    occupancy_grid_png_with_trajectories,
)
from co_3dto2d_mapping.occupancy_trajectory import (
    MAP_OUTPUTS,
    MapOutput,
    TrajectoryStore,
)


class OccupancyPngExporter(Node):
    """Save latest occupancy maps and matching two-robot trajectories as PNGs."""

    def __init__(self) -> None:
        super().__init__("occupancy_png_exporter")
        self.declare_parameter("output_directory", "output")
        self.declare_parameter("robot0_map_topic", "/r0/toy/global_occupancy")
        self.declare_parameter("robot1_map_topic", "/r1/toy/global_occupancy")
        self.declare_parameter("merged_map_topic", "/toy_record/merged_global_occupancy")
        self.declare_parameter("robot0_odom_topic", "/r0/odom")
        self.declare_parameter("robot1_odom_topic", "/r1/odom")
        self.declare_parameter("merged_alignment_topic", "/toy/startup_xy_alignment")
        self.declare_parameter("max_trajectory_points", 5000)
        self.declare_parameter("trajectory_export_period_sec", 1.0)

        self.output_directory = Path(
            str(self.get_parameter("output_directory").value)
        ).expanduser()
        self.output_directory.mkdir(parents=True, exist_ok=True)
        self._saved_filenames: Set[str] = set()
        self._latest_maps: Dict[str, OccupancyGrid] = {}
        self._trajectory_outputs_dirty = False
        self._trajectories = TrajectoryStore(
            max_points=int(self.get_parameter("max_trajectory_points").value)
        )
        self._trajectory_export_period_sec = float(
            self.get_parameter("trajectory_export_period_sec").value
        )
        if self._trajectory_export_period_sec <= 0.0:
            raise OccupancyGridEncodingError(
                reason="trajectory_export_period_sec must be positive"
            )

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self._subscriptions = []
        for output in MAP_OUTPUTS:
            topic = str(self.get_parameter(output.parameter_name).value)
            self._subscriptions.append(
                self.create_subscription(
                    OccupancyGrid,
                    topic,
                    lambda message, target=output: self._save_map(message, target),
                    qos,
                )
            )
        self._subscriptions.append(
            self.create_subscription(
                Odometry,
                str(self.get_parameter("robot0_odom_topic").value),
                self._robot0_odom_callback,
                qos,
            )
        )
        self._subscriptions.append(
            self.create_subscription(
                Odometry,
                str(self.get_parameter("robot1_odom_topic").value),
                self._robot1_odom_callback,
                qos,
            )
        )
        alignment_qos = QoSProfile(depth=1)
        alignment_qos.reliability = ReliabilityPolicy.RELIABLE
        alignment_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._subscriptions.append(
            self.create_subscription(
                TransformStamped,
                str(self.get_parameter("merged_alignment_topic").value),
                self._alignment_callback,
                alignment_qos,
            )
        )
        self._trajectory_timer = self.create_timer(
            self._trajectory_export_period_sec,
            self._save_pending_trajectory_maps,
        )
        self.get_logger().info(
            "Saving occupancy maps and robot trajectories to %s."
            % self.output_directory
        )

    def _write_image(self, filename: str, image: bytes) -> Path:
        target = self.output_directory / filename
        temporary = target.with_suffix(".tmp")
        temporary.write_bytes(image)
        temporary.replace(target)
        return target

    def _save_map(self, message: OccupancyGrid, output: MapOutput) -> None:
        width, height = int(message.info.width), int(message.info.height)
        try:
            image = occupancy_grid_png(
                width=width,
                height=height,
                occupancy=message.data,
            )
            target = self._write_image(output.filename, image)
        except (OSError, OccupancyGridEncodingError) as exc:
            self.get_logger().error(
                "Could not save occupancy PNG %s: %s" % (output.filename, exc)
            )
            return
        self._latest_maps[output.key] = message
        self._save_trajectory_map(message, output)
        if output.filename not in self._saved_filenames:
            self._saved_filenames.add(output.filename)
            self.get_logger().info("Saved occupancy PNG: %s" % target)

    def _robot0_odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        point = (float(position.x), float(position.y))
        if self._trajectories.add_robot0(point):
            self._trajectory_outputs_dirty = True

    def _robot1_odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        point = (float(position.x), float(position.y))
        if self._trajectories.add_robot1(point):
            self._trajectory_outputs_dirty = True

    def _alignment_callback(self, message: TransformStamped) -> None:
        rotation = message.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        translation = message.transform.translation
        alignment = (
            float(translation.x),
            float(translation.y),
            yaw,
        )
        self._trajectories.set_robot1_alignment(alignment)
        self._trajectory_outputs_dirty = True

    @staticmethod
    def _grid_geometry(message: OccupancyGrid) -> OccupancyGridGeometry:
        origin = message.info.origin
        rotation = origin.orientation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        return OccupancyGridGeometry(
            width=int(message.info.width),
            height=int(message.info.height),
            resolution=float(message.info.resolution),
            origin_x=float(origin.position.x),
            origin_y=float(origin.position.y),
            origin_yaw=yaw,
        )

    def _save_trajectory_map(self, message: OccupancyGrid, output: MapOutput) -> None:
        geometry = self._grid_geometry(message)
        overlays = self._trajectories.overlays_for(output, geometry)
        try:
            image = occupancy_grid_png_with_trajectories(
                width=geometry.width,
                height=geometry.height,
                occupancy=message.data,
                trajectories=overlays,
            )
            target = self._write_image(output.trajectory_filename, image)
        except (OSError, OccupancyGridEncodingError) as exc:
            self.get_logger().error(
                "Could not save trajectory occupancy PNG %s: %s"
                % (output.trajectory_filename, exc)
            )
            return
        if output.trajectory_filename not in self._saved_filenames:
            self._saved_filenames.add(output.trajectory_filename)
            self.get_logger().info("Saved trajectory occupancy PNG: %s" % target)

    def _save_pending_trajectory_maps(self) -> None:
        if not self._trajectory_outputs_dirty:
            return
        for output in MAP_OUTPUTS:
            message = self._latest_maps.get(output.key)
            if message is not None:
                self._save_trajectory_map(message, output)
        self._trajectory_outputs_dirty = False


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = OccupancyPngExporter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
