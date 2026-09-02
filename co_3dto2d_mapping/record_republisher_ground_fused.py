#!/usr/bin/env python3
"""Temporal record republisher that prefers the final ground-fused odometry."""

from __future__ import annotations

from typing import Set

import rclpy
from nav_msgs.msg import Odometry

from co_3dto2d_mapping.record_republisher_temporal import (
    TemporalToyRecordRepublisher,
)


class GroundFusedTemporalRecordRepublisher(TemporalToyRecordRepublisher):
    def __init__(self) -> None:
        # Raw /rN/odom subscriptions are constructed by the base class and call
        # this class's overridden odom_callback. Until a fused message arrives,
        # raw odometry remains a safe fallback.
        self._robots_with_ground_fused_odom: Set[int] = set()
        super().__init__()
        self.declare_parameter("prefer_ground_fused_odometry", True)
        self.declare_parameter(
            "ground_fused_odometry_topic_format",
            "/r{robot_id}/toy/corrected_odometry",
        )
        self.prefer_ground_fused_odometry = bool(
            self.get_parameter("prefer_ground_fused_odometry").value
        )
        self.ground_fused_odometry_topic_format = str(
            self.get_parameter("ground_fused_odometry_topic_format").value
        )

        self._ground_fused_subscriptions = []
        if self.prefer_ground_fused_odometry:
            for robot_id in self.robot_ids:
                topic = self.ground_fused_odometry_topic_format.format(
                    robot_id=robot_id
                )
                subscription = self.create_subscription(
                    Odometry,
                    topic,
                    lambda msg, rid=robot_id: self.ground_fused_odom_callback(
                        msg, rid
                    ),
                    10,
                )
                self._ground_fused_subscriptions.append(subscription)
            self.get_logger().info(
                "Record republisher prefers ground-fused odometry format %s; "
                "raw /rN/odom remains the startup fallback."
                % self.ground_fused_odometry_topic_format
            )

    def odom_callback(self, msg: Odometry, robot_id: int) -> None:
        if robot_id in self._robots_with_ground_fused_odom:
            return
        super().odom_callback(msg, robot_id)

    def ground_fused_odom_callback(self, msg: Odometry, robot_id: int) -> None:
        first = robot_id not in self._robots_with_ground_fused_odom
        self._robots_with_ground_fused_odom.add(robot_id)
        self.latest_odom[robot_id] = msg
        if first:
            self.get_logger().info(
                "Using ground-fused odometry for r%d from %s"
                % (
                    robot_id,
                    self.ground_fused_odometry_topic_format.format(
                        robot_id=robot_id
                    ),
                )
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundFusedTemporalRecordRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
