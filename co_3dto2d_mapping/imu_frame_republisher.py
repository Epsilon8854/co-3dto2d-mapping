#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuFrameRepublisher(Node):
    def __init__(self):
        super().__init__("imu_frame_republisher")
        self.declare_parameter("input_topic", "/livox/imu_filtered_raw_frame")
        self.declare_parameter("output_topic", "/livox/imu_filtered")
        self.declare_parameter("output_frame_id", "")

        self.output_frame_id = str(self.get_parameter("output_frame_id").value)
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self.publisher = self.create_publisher(Imu, output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            Imu, input_topic, self.callback, qos_profile_sensor_data
        )
        self.get_logger().info(
            "Republishing IMU frame %s -> %s output_frame_id=%s"
            % (input_topic, output_topic, self.output_frame_id or "<preserve>")
        )

    def callback(self, msg):
        out = Imu()
        out.header = msg.header
        if self.output_frame_id:
            out.header.frame_id = self.output_frame_id
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuFrameRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
