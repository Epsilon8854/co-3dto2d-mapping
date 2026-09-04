#!/usr/bin/env python3
"""Temporal record republisher that prefers fresh ground-fused odometry."""

from __future__ import annotations

from typing import Dict, Optional, Set

import rclpy
from nav_msgs.msg import Odometry

from co_3dto2d_mapping.record_republisher_temporal import (
    TemporalToyRecordRepublisher,
)


class GroundFusedTemporalRecordRepublisher(TemporalToyRecordRepublisher):
    def __init__(self) -> None:
        # Raw /rN/odom subscriptions are constructed by the base class and call
        # this class's overridden odom_callback. Callbacks do not run until spin,
        # so the selection state can be prepared before super().__init__().
        self._latest_raw_odom: Dict[int, Odometry] = {}
        self._latest_ground_fused_odom: Dict[int, Odometry] = {}
        self._ground_fused_receipt_ns: Dict[int, int] = {}
        self._robots_using_ground_fused_odom: Set[int] = set()
        super().__init__()

        self.declare_parameter("prefer_ground_fused_odometry", True)
        self.declare_parameter(
            "ground_fused_odometry_topic_format",
            "/r{robot_id}/toy/corrected_odometry",
        )
        # This timeout uses receipt wall/ROS time. Three seconds is long enough
        # for deliberately slowed combined-bag replay while still preventing a
        # dead fusion node from freezing TF indefinitely.
        self.declare_parameter("ground_fused_odometry_timeout_sec", 3.0)
        self.prefer_ground_fused_odometry = bool(
            self.get_parameter("prefer_ground_fused_odometry").value
        )
        self.ground_fused_odometry_topic_format = str(
            self.get_parameter("ground_fused_odometry_topic_format").value
        )
        self.ground_fused_odometry_timeout_sec = float(
            self.get_parameter("ground_fused_odometry_timeout_sec").value
        )
        if self.ground_fused_odometry_timeout_sec < 0.0:
            raise ValueError("ground_fused_odometry_timeout_sec must be non-negative")

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
                "Record republisher prefers fresh ground-fused odometry format %s "
                "with %.2fs timeout; raw /rN/odom is the startup and dropout fallback."
                % (
                    self.ground_fused_odometry_topic_format,
                    self.ground_fused_odometry_timeout_sec,
                )
            )

    def _ground_fused_is_fresh(
        self, robot_id: int, now_ns: Optional[int] = None
    ) -> bool:
        if not self.prefer_ground_fused_odometry:
            return False
        receipt_ns = self._ground_fused_receipt_ns.get(robot_id)
        if receipt_ns is None:
            return False
        if self.ground_fused_odometry_timeout_sec == 0.0:
            return True
        if now_ns is None:
            now_ns = self.get_clock().now().nanoseconds
        return now_ns - receipt_ns <= int(
            self.ground_fused_odometry_timeout_sec * 1e9
        )

    def odom_callback(self, msg: Odometry, robot_id: int) -> None:
        self._latest_raw_odom[robot_id] = msg
        if not self._ground_fused_is_fresh(robot_id):
            self.latest_odom[robot_id] = msg

    def ground_fused_odom_callback(self, msg: Odometry, robot_id: int) -> None:
        first = robot_id not in self._robots_using_ground_fused_odom
        self._latest_ground_fused_odom[robot_id] = msg
        self._ground_fused_receipt_ns[robot_id] = self.get_clock().now().nanoseconds
        self._robots_using_ground_fused_odom.add(robot_id)
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

    def _refresh_odometry_selection(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        for robot_id in self.robot_ids:
            if self._ground_fused_is_fresh(robot_id, now_ns):
                fused = self._latest_ground_fused_odom.get(robot_id)
                if fused is not None:
                    self.latest_odom[robot_id] = fused
                    self._robots_using_ground_fused_odom.add(robot_id)
                continue

            raw = self._latest_raw_odom.get(robot_id)
            if raw is not None:
                self.latest_odom[robot_id] = raw
            if robot_id in self._robots_using_ground_fused_odom:
                self._robots_using_ground_fused_odom.remove(robot_id)
                self.get_logger().warn(
                    "Ground-fused odometry for r%d timed out; falling back to /r%d/odom."
                    % (robot_id, robot_id)
                )

    def publish_outputs(self) -> None:
        self._refresh_odometry_selection()
        super().publish_outputs()


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
