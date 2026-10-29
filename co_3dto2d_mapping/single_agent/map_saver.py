#!/usr/bin/env python3
"""Keep the latest map in memory and export it once at normal shutdown."""
from collections import deque
from pathlib import Path
import math
import signal
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from co_3dto2d_mapping.common.map_storage import Snapshot, save_map
from co_3dto2d_mapping.common.occupancy_submap import GridGeometry


class SingleAgentMapSaver(Node):
    def __init__(self):
        super().__init__('single_agent_map_saver')
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('output_directory', 'outputs/single_agent/maps')
        rid = self.get_parameter('robot_id').value
        self.out = Path(self.get_parameter('output_directory').value)
        self.latest_map = self.raw_path = None
        self.mapping_path = deque(maxlen=10000)
        self.saved = False
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(OccupancyGrid, f'/r{rid}/mapping/global_occupancy', self.map, latest)
        self.create_subscription(Odometry, f'/r{rid}/mapping/occupancy_mapper_odometry', self.pose, latest)
        self.create_subscription(RosPath, f'/r{rid}/mapping/debug/raw_odometry_path', self.path, latest)

    def map(self, msg):
        if msg.info.width and msg.info.height:
            self.latest_map = msg

    def pose(self, msg):
        p = msg.pose.pose.position
        if msg.header.frame_id == 'map' and math.isfinite(p.x) and math.isfinite(p.y):
            self.mapping_path.append((p.x, p.y))

    def path(self, msg):
        if msg.header.frame_id == 'map':
            self.raw_path = msg

    def save(self):
        if self.saved or self.latest_map is None:
            return
        msg = self.latest_map
        g = msg.info
        q = g.origin.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        snapshot = Snapshot(np.asarray(msg.data, dtype=np.int8).reshape(g.height, g.width),
            GridGeometry(g.resolution, g.width, g.height, g.origin.position.x, g.origin.position.y, yaw),
            int(msg.header.stamp.sec)*10**9+msg.header.stamp.nanosec, msg.header.frame_id)
        paths = {'mapping': list(self.mapping_path)}
        if self.raw_path is not None and self.raw_path.header.frame_id == snapshot.frame:
            paths['raw'] = [(p.pose.position.x, p.pose.position.y) for p in self.raw_path.poses]
        save_map(self.out, 'global_map', snapshot, paths)
        self.saved = True
        self.get_logger().info(f'Final map saved once: {self.out} (frame={snapshot.frame})')


def main():
    rclpy.init(args=None, signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = SingleAgentMapSaver()
    stopping = False

    def stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        try:
            # Drain already-published final updates before exporting once.
            end = time.monotonic()+.25
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.02)
            node.save()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
