#!/usr/bin/env python3
"""Save the final Experiment 2 global occupancy with raw and corrected paths.

The exporter is deliberately a replay-side utility. It receives the final map
and trajectories while a bag is replayed, then writes one north-up PNG when it
is interrupted by the replay runner.
"""

from __future__ import annotations

import csv
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from co_3dto2d_mapping.occupancy_png import (
    OccupancyGridGeometry,
    TrajectoryOverlay,
    occupancy_grid_png_with_trajectories,
    trajectory_pixels,
)


WorldPoint = Tuple[float, float]
TimedWorldPoint = Tuple[int, float, float]
RAW_CYAN = (0, 220, 255)
CORRECTED_ORANGE = (255, 153, 0)
OUTPUT_FILENAME = "topdown_global_map_raw_cyan_corrected_orange.png"


def _stamp_ns(stamp: object) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _append_unique(points: List[TimedWorldPoint], point: TimedWorldPoint) -> None:
    if not points or points[-1] != point:
        points.append(point)


def _geometry(message: OccupancyGrid) -> OccupancyGridGeometry:
    orientation = message.info.origin.orientation
    yaw = __import__("math").atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )
    return OccupancyGridGeometry(
        width=int(message.info.width),
        height=int(message.info.height),
        resolution=float(message.info.resolution),
        origin_x=float(message.info.origin.position.x),
        origin_y=float(message.info.origin.position.y),
        origin_yaw=yaw,
    )


class Experiment02TopDownExporter(Node):
    def __init__(self) -> None:
        super().__init__("experiment_02_topdown_exporter")
        self.declare_parameter("output_directory", "topdown")
        self.declare_parameter("global_map_topic", "/r0/mapping/global_occupancy")
        self.declare_parameter("raw_path_topic", "/r0/mapping/debug/raw_odometry_path")
        self.declare_parameter("icp_odometry_topic", "/r0/odom")
        self.declare_parameter("corrected_odometry_topic", "/r0/mapping/occupancy_mapper_odometry")
        self._output_directory = Path(
            str(self.get_parameter("output_directory").value)
        ).expanduser()
        self._latest_map: Optional[OccupancyGrid] = None
        self._raw_points: List[TimedWorldPoint] = []
        self._icp_points: List[TimedWorldPoint] = []
        self._corrected_points: List[TimedWorldPoint] = []
        self._subscriptions = [
            self.create_subscription(
                OccupancyGrid,
                str(self.get_parameter("global_map_topic").value),
                self._map_callback,
                10,
            ),
            self.create_subscription(
                RosPath,
                str(self.get_parameter("raw_path_topic").value),
                self._raw_path_callback,
                10,
            ),
            self.create_subscription(
                Odometry,
                str(self.get_parameter("icp_odometry_topic").value),
                self._icp_odometry_callback,
                100,
            ),
            self.create_subscription(
                Odometry,
                str(self.get_parameter("corrected_odometry_topic").value),
                self._corrected_odometry_callback,
                100,
            ),
        ]
        self.get_logger().info("Capturing final map and trajectories in %s" % self._output_directory)

    def _map_callback(self, message: OccupancyGrid) -> None:
        self._latest_map = message

    def _raw_path_callback(self, message: RosPath) -> None:
        self._raw_points = [
            (
                _stamp_ns(pose.header.stamp),
                float(pose.pose.position.x),
                float(pose.pose.position.y),
            )
            for pose in message.poses
        ]

    def _corrected_odometry_callback(self, message: Odometry) -> None:
        self._append_odometry_point(self._corrected_points, message)

    def _icp_odometry_callback(self, message: Odometry) -> None:
        self._append_odometry_point(self._icp_points, message)

    @staticmethod
    def _append_odometry_point(
        points: List[TimedWorldPoint], message: Odometry
    ) -> None:
        position = message.pose.pose.position
        _append_unique(
            points,
            (_stamp_ns(message.header.stamp), float(position.x), float(position.y)),
        )

    def _write_trajectory_csv(
        self, filename: str, points: List[TimedWorldPoint]
    ) -> None:
        with (self._output_directory / filename).open(
            "w", newline="", encoding="utf-8"
        ) as stream:
            writer = csv.writer(stream)
            writer.writerow(("stamp_ns", "x_m", "y_m"))
            writer.writerows(points)

    def save(self) -> Optional[Path]:
        if self._latest_map is None:
            print("No global occupancy map was received; no PNG written.", flush=True)
            return None
        geometry = _geometry(self._latest_map)
        raw_overlay_points = self._raw_points or self._icp_points
        overlays = (
            TrajectoryOverlay(
                points=trajectory_pixels(
                    geometry=geometry,
                    points=[(x, y) for _, x, y in raw_overlay_points],
                ),
                color=RAW_CYAN,
            ),
            TrajectoryOverlay(
                points=trajectory_pixels(
                    geometry=geometry,
                    points=[(x, y) for _, x, y in self._corrected_points],
                ),
                color=CORRECTED_ORANGE,
            ),
        )
        image = occupancy_grid_png_with_trajectories(
            width=geometry.width,
            height=geometry.height,
            occupancy=self._latest_map.data,
            trajectories=overlays,
        )
        self._output_directory.mkdir(parents=True, exist_ok=True)
        self._write_trajectory_csv("raw_odometry_xy.csv", raw_overlay_points)
        self._write_trajectory_csv("icp_odometry_xy.csv", self._icp_points)
        self._write_trajectory_csv("corrected_odometry_xy.csv", self._corrected_points)
        target = self._output_directory / OUTPUT_FILENAME
        target.write_bytes(image)
        (self._output_directory / "legend.yaml").write_text(
            "map: final /r0/mapping/global_occupancy\n"
            "raw_odometry: cyan (#00dcff), /r0/mapping/debug/raw_odometry_path (ground-node synchronized)\n"
            "icp_odometry: /r0/odom (direct RTAB-Map ICP output)\n"
            "corrected_odometry: orange (#ff9900), /r0/mapping/occupancy_mapper_odometry\n"
            f"raw_path_points: {len(raw_overlay_points)}\n"
            f"icp_odometry_points: {len(self._icp_points)}\n"
            f"corrected_path_points: {len(self._corrected_points)}\n",
            encoding="utf-8",
        )
        # SIGINT/SIGTERM closes the ROS context before this finally block.
        # Use stdout here so a successful export never emits a false rosout error.
        print("Saved top-down overlay: %s" % target, flush=True)
        return target


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = Experiment02TopDownExporter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
