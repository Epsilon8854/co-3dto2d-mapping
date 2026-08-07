#!/usr/bin/env python3

import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


class PointCloudRearSectorFilter(Node):
    def __init__(self):
        super().__init__("pointcloud_rear_sector_filter")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("input_topic", "/livox/lidar_raw"),
                ("output_topic", "/livox/lidar"),
                ("enabled", True),
                ("rear_filter_angle_deg", 120.0),
                ("rear_axis", "-x"),
                ("min_xy_range_m", 0.0),
                ("log_period", 100),
                ("output_frame_id", ""),
            ],
        )

        self.enabled = bool(self.get_parameter("enabled").value)
        self.rear_filter_angle_deg = float(
            self.get_parameter("rear_filter_angle_deg").value
        )
        self.rear_axis = str(self.get_parameter("rear_axis").value).lower()
        self.min_xy_range_m = float(self.get_parameter("min_xy_range_m").value)
        self.log_period = int(self.get_parameter("log_period").value)
        self.output_frame_id = str(self.get_parameter("output_frame_id").value)

        self.center_angle = self._axis_to_angle(self.rear_axis)
        self.half_angle_rad = math.radians(
            max(0.0, min(360.0, self.rear_filter_angle_deg)) * 0.5
        )
        self.msg_count = 0
        self.filtered_points = 0
        self.kept_points = 0

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.publisher = self.create_publisher(
            PointCloud2, output_topic, qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self.callback, qos_profile_sensor_data
        )
        self.get_logger().info(
            "Rear-sector filtering %s -> %s enabled=%s axis=%s angle=%.1f deg"
            % (
                input_topic,
                output_topic,
                self.enabled,
                self.rear_axis,
                self.rear_filter_angle_deg,
            )
        )

    def _axis_to_angle(self, axis):
        if axis == "x":
            return 0.0
        if axis == "y":
            return math.pi * 0.5
        if axis == "-y":
            return -math.pi * 0.5
        if axis == "-x":
            return math.pi
        raise ValueError("rear_axis must be one of x, -x, y, -y")

    def _field_unpacker(self, msg, name):
        fields = {field.name: field for field in msg.fields}
        if name not in fields:
            raise ValueError("PointCloud2 is missing field '%s'" % name)
        field = fields[name]
        if field.datatype == PointField.FLOAT32:
            return field.offset, ("<f" if not msg.is_bigendian else ">f")
        if field.datatype == PointField.FLOAT64:
            return field.offset, ("<d" if not msg.is_bigendian else ">d")
        raise ValueError("PointCloud2 field '%s' must be FLOAT32 or FLOAT64" % name)

    def _is_in_rear_sector(self, x, y):
        xy_range = math.hypot(x, y)
        if xy_range <= self.min_xy_range_m:
            return False
        angle = math.atan2(y, x)
        diff = math.atan2(
            math.sin(angle - self.center_angle), math.cos(angle - self.center_angle)
        )
        return abs(diff) <= self.half_angle_rad

    def callback(self, msg):
        if not self.enabled:
            self.publisher.publish(msg)
            return

        try:
            x_offset, x_fmt = self._field_unpacker(msg, "x")
            y_offset, y_fmt = self._field_unpacker(msg, "y")
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            self.publisher.publish(msg)
            return

        data = bytes(msg.data)
        point_step = int(msg.point_step)
        input_points = int(msg.width) * int(msg.height)
        kept = bytearray()
        dropped = 0

        for index in range(input_points):
            start = index * point_step
            end = start + point_step
            point_bytes = data[start:end]
            if len(point_bytes) != point_step:
                continue
            x = struct.unpack_from(x_fmt, point_bytes, x_offset)[0]
            y = struct.unpack_from(y_fmt, point_bytes, y_offset)[0]
            if not math.isfinite(x) or not math.isfinite(y):
                dropped += 1
                continue
            if self._is_in_rear_sector(x, y):
                dropped += 1
                continue
            kept.extend(point_bytes)

        out = PointCloud2()
        out.header = msg.header
        if self.output_frame_id:
            out.header.frame_id = self.output_frame_id
        out.height = 1
        out.width = len(kept) // point_step if point_step > 0 else 0
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = out.width * msg.point_step
        out.data = bytes(kept)
        out.is_dense = False
        self.publisher.publish(out)

        self.msg_count += 1
        self.filtered_points += dropped
        self.kept_points += out.width
        if self.log_period > 0 and self.msg_count % self.log_period == 0:
            total = self.filtered_points + self.kept_points
            ratio = 100.0 * self.filtered_points / total if total else 0.0
            self.get_logger().info(
                "Rear-sector filter stats: kept=%d dropped=%d dropped_ratio=%.1f%%"
                % (self.kept_points, self.filtered_points, ratio)
            )


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudRearSectorFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
