#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class PointCloudFrameRepublisher(Node):
    """Republish PointCloud2 while replacing only the header frame_id."""

    def __init__(self) -> None:
        super().__init__("pointcloud_frame_republisher")
        self.declare_parameter("input_topic", "/livox/lidar")
        self.declare_parameter("output_topic", "/mapping/lidar")
        self.declare_parameter("output_frame_id", "")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.output_frame_id = str(self.get_parameter("output_frame_id").value)

        if input_topic == output_topic:
            raise ValueError(
                "input_topic and output_topic must differ to avoid a republish loop"
            )

        self.publisher = self.create_publisher(
            PointCloud2, output_topic, qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self.callback, qos_profile_sensor_data
        )
        self.get_logger().info(
            "Republishing PointCloud2 %s -> %s output_frame_id=%s"
            % (input_topic, output_topic, self.output_frame_id or "<preserve>")
        )

    def callback(self, msg: PointCloud2) -> None:
        # The Livox points are already expressed in the sensor-local coordinates.
        # For multi-robot TF isolation we only give that same sensor frame a unique
        # robot prefix; no point coordinate transform is performed here.
        if self.output_frame_id:
            msg.header.frame_id = self.output_frame_id
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointCloudFrameRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
