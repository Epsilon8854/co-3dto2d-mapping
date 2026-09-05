#!/usr/bin/env python3
"""Continuously save the latest two-live occupancy maps as PNG files."""

from __future__ import annotations

from pathlib import Path
from typing import Final, List, Optional, Set, Tuple

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from co_3dto2d_mapping.occupancy_png import (
    OccupancyGridEncodingError,
    occupancy_grid_png,
)


_MAP_OUTPUTS: Final[Tuple[Tuple[str, str], ...]] = (
    ("robot0_map_topic", "r0_global_occupancy.png"),
    ("robot1_map_topic", "r1_global_occupancy.png"),
    ("merged_map_topic", "merged_global_occupancy.png"),
)


class OccupancyPngExporter(Node):
    """Overwrite each output with the latest map received on its transient topic."""

    def __init__(self) -> None:
        super().__init__("occupancy_png_exporter")
        self.declare_parameter("output_directory", "output")
        self.declare_parameter("robot0_map_topic", "/r0/toy/global_occupancy")
        self.declare_parameter("robot1_map_topic", "/r1/toy/global_occupancy")
        self.declare_parameter("merged_map_topic", "/toy_record/merged_global_occupancy")

        self.output_directory = Path(
            str(self.get_parameter("output_directory").value)
        ).expanduser()
        self.output_directory.mkdir(parents=True, exist_ok=True)
        self._saved_filenames: Set[str] = set()

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self._subscriptions = []
        for parameter_name, filename in _MAP_OUTPUTS:
            topic = str(self.get_parameter(parameter_name).value)
            self._subscriptions.append(
                self.create_subscription(
                    OccupancyGrid,
                    topic,
                    lambda message, target=filename: self._save_map(message, target),
                    qos,
                )
            )
        self.get_logger().info(
            "Saving latest occupancy maps to %s." % self.output_directory
        )

    def _save_map(self, message: OccupancyGrid, filename: str) -> None:
        width, height = int(message.info.width), int(message.info.height)
        try:
            image = occupancy_grid_png(
                width=width,
                height=height,
                occupancy=message.data,
            )
            target = self.output_directory / filename
            temporary = target.with_suffix(".tmp")
            temporary.write_bytes(image)
            temporary.replace(target)
        except (OSError, OccupancyGridEncodingError) as exc:
            self.get_logger().error(
                "Could not save occupancy PNG %s: %s" % (filename, exc)
            )
            return
        if filename not in self._saved_filenames:
            self._saved_filenames.add(filename)
            self.get_logger().info("Saved occupancy PNG: %s" % target)


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
