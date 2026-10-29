#!/usr/bin/env python3
"""Lightweight world-origin and global-pose labels for RViz demonstrations."""
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from co_3dto2d_mapping.multi_agent.ros import values_of


class RvizAnnotations(Node):
    def __init__(self):
        super().__init__('multi_agent_rviz_annotations')
        defaults = dict(world_frame='map', axis_length_m=1.0, text_height_m=0.20,
                        pose_stale_sec=2.0, use_odometry_input=False)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.config = {key: self.get_parameter(key).value for key in defaults}
        if not self.config['world_frame']:
            raise ValueError('world_frame must not be empty')
        for key in ('axis_length_m', 'text_height_m', 'pose_stale_sec'):
            if not math.isfinite(self.config[key]) or self.config[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(MarkerArray, '/multi_agent/visualization_markers', latched)
        self.poses = {}
        self.epoch = None
        self.ready = True  # Older result bags may not contain map_status.
        self.last_ros_time = None
        for rid in (0, 1):
            use_odometry = self.config['use_odometry_input']
            kind = Odometry if use_odometry else PoseWithCovarianceStamped
            topic = f'/multi_agent/r{rid}/' + ('aligned_odometry' if use_odometry else 'global_pose')
            self.create_subscription(kind, topic,
                                     lambda msg, r=rid: self.pose(msg, r), latest)
        self.create_subscription(DiagnosticArray, '/multi_agent/map_status', self.status, latched)
        self.create_timer(0.2, self.publish)

    def status(self, msg):
        status = values_of(msg)
        epoch = status.get('coordinate_epoch')
        if epoch is not None and epoch != self.epoch:
            self.poses.clear()
            self.epoch = epoch
        self.ready = status.get('ready', True)
        if not self.ready:
            self.poses.clear()

    def pose(self, msg, rid):
        if not self.ready or msg.header.frame_id != self.config['world_frame']:
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values):
            return
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm < 1e-12:
            return
        self.poses[rid] = (values[:3], time.monotonic())

    def marker(self, namespace, identifier, kind, xyz, color):
        marker = Marker()
        marker.header.frame_id = self.config['world_frame']
        # The coordinates already belong to the fixed world frame.
        marker.ns = namespace
        marker.id = identifier
        marker.type = kind
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, xyz)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def text(self, namespace, identifier, xyz, text, color, height=None):
        marker = self.marker(namespace, identifier, Marker.TEXT_VIEW_FACING, xyz, color)
        marker.scale.z = height or self.config['text_height_m']
        marker.text = text
        return marker

    def publish(self):
        now_ros = self.get_clock().now().nanoseconds
        if self.last_ros_time is not None and now_ros < self.last_ros_time:
            self.poses.clear()
        self.last_ros_time = now_ros
        markers = []
        white = (1.0, 1.0, 1.0, 1.0)
        length = self.config['axis_length_m']
        for axis, color in enumerate(((1.0, 0.12, 0.12, 1.0), (0.1, 1.0, 0.2, 1.0), (0.2, 0.5, 1.0, 1.0))):
            endpoint = [0.0, 0.0, 0.0]
            endpoint[axis] = length
            arrow = self.marker('world_axes', axis, Marker.ARROW, (0, 0, 0), color)
            arrow.points = [Point(), Point(x=endpoint[0], y=endpoint[1], z=endpoint[2])]
            arrow.scale.x, arrow.scale.y, arrow.scale.z = length*0.035, length*0.09, length*0.15
            markers.append(arrow)
            label_position = list(endpoint)
            label_position[axis] += 0.12
            if axis == 2:
                label_position[0] = -0.2  # Separate Z from the origin in top-down view.
            markers.append(self.text('axis_labels', axis, label_position, 'XYZ'[axis], color, 0.18))
        origin = self.marker('world_origin', 0, Marker.SPHERE, (0, 0, 0), white)
        origin.scale.x = origin.scale.y = origin.scale.z = 0.08
        markers.append(origin)
        markers.append(self.text('world_origin', 1, (0, 0.3, 0.12),
                                 f'{self.config["world_frame"]} WORLD (0,0)', white, 0.20))
        now = time.monotonic()
        for rid, color in ((0, (1.0, 0.51, 0.16, 1.0)), (1, (0.0, 0.86, 0.86, 1.0))):
            # Remove old world-space numerical panels when replacing a running display.
            remove_panel = self.marker('pose_panels', rid, Marker.TEXT_VIEW_FACING, (0, 0, 0), color)
            remove_panel.action = Marker.DELETE
            markers.append(remove_panel)
            entry = self.poses.get(rid)
            if entry is None:
                remove = self.marker('robot_labels', rid, Marker.TEXT_VIEW_FACING, (0, 0, 0), color)
                remove.action = Marker.DELETE
                markers.append(remove)
            else:
                (x, y, z), received = entry
                if now-received > self.config['pose_stale_sec']:
                    color = (0.65, 0.65, 0.65, 1.0)
                markers.append(self.text('robot_labels', rid, (x, y, z+0.4), f'r{rid}', color, 0.24))
        self.publisher.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)
    node = RvizAnnotations()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
