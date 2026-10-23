#!/usr/bin/env python3
"""Save one agent's native map and mapping/raw paths on a separate process."""
from collections import deque
from pathlib import Path
import math
import signal
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid,Odometry
from co_3dto2d_mapping.common.map_storage import Snapshot,save_map
from co_3dto2d_mapping.common.occupancy_submap import GridGeometry


class SingleAgentMapSaver(Node):
    def __init__(self):
        super().__init__('single_agent_map_saver')
        self.declare_parameter('robot_id',0);self.declare_parameter('output_directory','outputs/single_agent/maps')
        rid=self.get_parameter('robot_id').value;self.out=Path(self.get_parameter('output_directory').value)
        self.snapshot=None;self.paths={k:deque(maxlen=10000) for k in ('mapping','raw')}
        self.create_subscription(OccupancyGrid,f'/r{rid}/mapping/global_occupancy',self.map,10)
        for kind,topic in (('mapping',f'/r{rid}/mapping/occupancy_mapper_odometry'),('raw',f'/r{rid}/odom')):
            self.create_subscription(Odometry,topic,lambda msg,k=kind:self.pose(msg,k),10)
        self.create_timer(2.,self.save)

    def map(self,msg):
        g=msg.info;q=g.origin.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.snapshot=Snapshot(np.asarray(msg.data,dtype=np.int8).reshape(g.height,g.width),
            GridGeometry(g.resolution,g.width,g.height,g.origin.position.x,g.origin.position.y,yaw),
            int(msg.header.stamp.sec)*10**9+msg.header.stamp.nanosec,msg.header.frame_id)

    def pose(self,msg,kind):
        p=msg.pose.pose.position
        if math.isfinite(p.x) and math.isfinite(p.y):self.paths[kind].append((p.x,p.y))

    def save(self):
        if self.snapshot is not None:save_map(self.out,'global_map',self.snapshot,{k:list(v) for k,v in self.paths.items()})


def main():
    rclpy.init(args=None,signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node=SingleAgentMapSaver();stopping=False
    def stop(*_):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        while rclpy.ok() and not stopping:rclpy.spin_once(node,timeout_sec=.05)
    finally:
        node.save();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
