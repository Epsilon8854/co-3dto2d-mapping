#!/usr/bin/env python3
"""Latch a startup alignment onto the normal inter-robot alignment topic."""

from __future__ import annotations

from copy import deepcopy

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class StartupAlignmentRelay(Node):
    def __init__(self) -> None:
        super().__init__("startup_alignment_relay")
        self.declare_parameter("input_topic", "/toy/startup_xy_alignment")
        self.declare_parameter("output_topic", "/toy/initial_xy_alignment")
        self.declare_parameter("lock_after_first", True)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.lock_after_first = bool(self.get_parameter("lock_after_first").value)
        if not self.input_topic.startswith("/") or not self.output_topic.startswith("/"):
            raise ValueError("relay topics must be absolute")
        if self.input_topic == self.output_topic:
            raise ValueError("input_topic and output_topic must differ")

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(TransformStamped, self.output_topic, qos)
        self.subscription = self.create_subscription(
            TransformStamped,
            self.input_topic,
            self._callback,
            qos,
        )
        self.latched = None
        self.get_logger().info(
            "Relaying startup alignment %s -> %s lock_after_first=%s"
            % (
                self.input_topic,
                self.output_topic,
                "true" if self.lock_after_first else "false",
            )
        )

    def _callback(self, msg: TransformStamped) -> None:
        if self.latched is not None and self.lock_after_first:
            return
        self.latched = deepcopy(msg)
        self.publisher.publish(self.latched)
        self.get_logger().info(
            "Published startup 2D alignment on the normal fusion topic %s."
            % self.output_topic
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StartupAlignmentRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
