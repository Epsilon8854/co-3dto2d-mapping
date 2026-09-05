#!/usr/bin/env python3
"""Wait for one transient-local startup alignment and exit deterministically."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class AlignmentStartupGate(Node):
    def __init__(self) -> None:
        super().__init__("alignment_startup_gate")
        self.declare_parameter("alignment_topic", "/toy/startup_xy_alignment")
        # Generic input names are used by the LiDAR-slice map path.  The cloud
        # names remain as a backwards-compatible fallback for older launch files.
        self.declare_parameter("robot0_input_topic", "")
        self.declare_parameter("robot1_input_topic", "")
        self.declare_parameter("robot0_cloud_topic", "/r0/mapping/lidar")
        self.declare_parameter("robot1_cloud_topic", "/r1/mapping/lidar")
        self.declare_parameter("timeout_sec", 0.0)
        self.declare_parameter("status_period_sec", 2.0)

        value = lambda name: self.get_parameter(name).value
        self.alignment_topic = str(value("alignment_topic"))
        generic_topics = (
            str(value("robot0_input_topic")),
            str(value("robot1_input_topic")),
        )
        legacy_topics = (
            str(value("robot0_cloud_topic")),
            str(value("robot1_cloud_topic")),
        )
        self.input_topics = tuple(
            generic or legacy for generic, legacy in zip(generic_topics, legacy_topics)
        )
        # Preserve this attribute for source-level compatibility with existing
        # tests and external launch wrappers.
        self.cloud_topics = self.input_topics
        self.timeout_sec = float(value("timeout_sec"))
        self.status_period_sec = float(value("status_period_sec"))
        self._validate_parameters()

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.subscription = self.create_subscription(
            TransformStamped,
            self.alignment_topic,
            self._alignment_callback,
            qos,
        )

        self.received = False
        self.timed_out = False
        self.started_monotonic = time.monotonic()
        self.last_status_monotonic = self.started_monotonic - self.status_period_sec
        self.get_logger().info(
            "Waiting for accepted startup alignment on %s; timeout=%s, "
            "input_topics=(%s, %s)."
            % (
                self.alignment_topic,
                self._timeout_label(),
                *self.input_topics,
            )
        )

    def _validate_parameters(self) -> None:
        if not self.alignment_topic.startswith("/"):
            raise ValueError("alignment_topic must be absolute")
        if not all(topic.startswith("/") for topic in self.input_topics):
            raise ValueError("robot input topics must be absolute")
        if not math.isfinite(self.timeout_sec) or self.timeout_sec < 0.0:
            raise ValueError("timeout_sec must be non-negative and finite")
        if (
            not math.isfinite(self.status_period_sec)
            or self.status_period_sec <= 0.0
        ):
            raise ValueError("status_period_sec must be positive and finite")

    def _alignment_callback(self, message: TransformStamped) -> None:
        if self.received:
            return
        self.received = True
        self.get_logger().info(
            "Startup alignment received: %s <- %s, x=%.3f y=%.3f yaw=%.2fdeg."
            % (
                message.header.frame_id or "<empty>",
                message.child_frame_id or "<empty>",
                message.transform.translation.x,
                message.transform.translation.y,
                math.degrees(
                    2.0
                    * math.atan2(
                        message.transform.rotation.z,
                        message.transform.rotation.w,
                    )
                ),
            )
        )

    def _timeout_label(self) -> str:
        return "disabled" if self.timeout_sec == 0.0 else "%.1fs" % self.timeout_sec

    def poll(self) -> None:
        if self.received or self.timed_out:
            return
        now = time.monotonic()
        elapsed = now - self.started_monotonic
        if self.timeout_sec > 0.0 and elapsed >= self.timeout_sec:
            self.timed_out = True
            self.get_logger().error(
                "Startup alignment timed out after %.1fs. alignment_publishers=%d, "
                "input_publishers=(%s:%d, %s:%d)."
                % (
                    elapsed,
                    self.count_publishers(self.alignment_topic),
                    self.input_topics[0],
                    self.count_publishers(self.cloud_topics[0]),
                    self.input_topics[1],
                    self.count_publishers(self.cloud_topics[1]),
                )
            )
            return
        if now - self.last_status_monotonic < self.status_period_sec:
            return
        self.last_status_monotonic = now
        self.get_logger().info(
            "Still waiting for startup alignment: elapsed=%.1fs timeout=%s, "
            "alignment_publishers=%d, input_publishers=(%s:%d, %s:%d)."
            % (
                elapsed,
                self._timeout_label(),
                self.count_publishers(self.alignment_topic),
                self.input_topics[0],
                self.count_publishers(self.cloud_topics[0]),
                self.input_topics[1],
                self.count_publishers(self.cloud_topics[1]),
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AlignmentStartupGate()
    exit_code = 1
    try:
        while rclpy.ok() and not node.received and not node.timed_out:
            rclpy.spin_once(node, timeout_sec=0.2)
            node.poll()
        exit_code = 0 if node.received else 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
