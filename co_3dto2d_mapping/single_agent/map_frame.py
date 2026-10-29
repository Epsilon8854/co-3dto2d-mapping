#!/usr/bin/env python3
"""Publish map->raw odom from final mapping poses and same-time raw TF."""
from collections import deque
import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from co_3dto2d_mapping.multi_agent.frames import rigid_components, rigid_matrix, world_from_odom


class SingleAgentMapFrame(Node):
    def __init__(self):
        super().__init__('single_agent_map_frame')
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('tf_wait_timeout_sec', 1.0)
        rid = self.get_parameter('robot_id').value
        self.wait_timeout = self.get_parameter('tf_wait_timeout_sec').value
        if rid not in (0, 1) or not math.isfinite(self.wait_timeout) or self.wait_timeout <= 0:
            raise ValueError('Invalid robot ID or TF wait timeout')
        self.odom_frame, self.base_frame = f'r{rid}/odom', f'r{rid}/base_link'
        self.buffer = Buffer(cache_time=Duration(seconds=10.0), node=self)
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.pending = deque(maxlen=32)
        self.last_input = self.last_published = self.last_clock = None
        self.warned_frame = False
        self.create_subscription(Odometry, f'/r{rid}/mapping/occupancy_mapper_odometry', self.pose,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(0.05, self.publish)

    @staticmethod
    def stamp_ns(stamp):
        return stamp.sec*1_000_000_000 + stamp.nanosec

    def pose(self, msg):
        if msg.header.frame_id != 'map':
            if not self.warned_frame:
                self.get_logger().error('Single mapping pose must use map; update launch/config together.')
                self.warned_frame = True
            return
        stamp = self.stamp_ns(msg.header.stamp)
        if self.last_input is not None and stamp < self.last_input:
            self.pending.clear()
            self.last_published = None
            self.buffer.clear()
        if stamp == self.last_input:
            return
        self.last_input = stamp
        self.pending.append((msg, time.monotonic()))
        self.publish()

    def publish(self):
        clock = self.get_clock().now().nanoseconds
        if self.last_clock is not None and clock < self.last_clock:
            self.pending.clear()
            self.last_input = self.last_published = None
            self.buffer.clear()
        self.last_clock = clock
        now = time.monotonic()
        while self.pending and (now-self.pending[0][1] > self.wait_timeout or
              (self.last_published is not None and self.stamp_ns(self.pending[0][0].header.stamp) <= self.last_published)):
            self.pending.popleft()
        for msg, _ in reversed(self.pending):
            try:
                raw = self.buffer.lookup_transform(self.odom_frame, self.base_frame, Time.from_msg(msg.header.stamp))
                break
            except TransformException:
                continue
        else:
            return
        p, t = msg.pose.pose, raw.transform
        try:
            world_base = rigid_matrix([p.position.x, p.position.y, p.position.z],
                [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
            odom_base = rigid_matrix([t.translation.x, t.translation.y, t.translation.z],
                [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
            xyz, quat = rigid_components(world_from_odom(world_base, odom_base))
        except ValueError:
            return
        tf = TransformStamped()
        tf.header.frame_id = 'map'
        tf.child_frame_id = self.odom_frame
        tf.header.stamp = msg.header.stamp
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = xyz
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = quat
        self.broadcaster.sendTransform(tf)
        self.last_published = self.stamp_ns(msg.header.stamp)
        while self.pending and self.stamp_ns(self.pending[0][0].header.stamp) <= self.last_published:
            self.pending.popleft()


def main(args=None):
    rclpy.init(args=args)
    node = SingleAgentMapFrame()
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
