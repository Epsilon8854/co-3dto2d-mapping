#!/usr/bin/env python3
"""Per-robot session heartbeat for Experiment 4; never publishes sensor TF."""
import hashlib
import json
import math
from pathlib import Path
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from co_3dto2d_mapping.multi_agent.ros import diagnostic, stamp_ns, pose_of
from diagnostic_msgs.msg import DiagnosticArray


class RobotSession(Node):
    def __init__(self):
        super().__init__('multi_agent_robot_session')
        for k,v in dict(robot_id=0,config_file='',session_id='').items():self.declare_parameter(k,v)
        self.rid=int(self.get_parameter('robot_id').value)
        self.config_file=str(self.get_parameter('config_file').value)
        self.session_id=str(self.get_parameter('session_id').value) or str(uuid.uuid4())
        self.last_stamp=-1;self.last_pose=None;self.reset_count=0
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub=self.create_publisher(DiagnosticArray,f'/multi_agent/r{self.rid}/session',qos)
        self.create_subscription(Odometry,f'/r{self.rid}/odom',self.odom,10)
        self.create_timer(1.,self.publish);self.publish()
    def odom(self,msg):
        ns=stamp_ns(msg.header.stamp)
        current=pose_of(msg)
        jumped=False
        if self.last_pose is not None and 0<ns-self.last_stamp<1_000_000_000:
            dx,dy=current[0]-self.last_pose[0],current[1]-self.last_pose[1]
            yaw=math.atan2(math.sin(current[2]-self.last_pose[2]),math.cos(current[2]-self.last_pose[2]))
            jumped=math.hypot(dx,dy)>2.0 or abs(yaw)>math.radians(45.)
        if ns<self.last_stamp or jumped:
            self.session_id=str(uuid.uuid4());self.reset_count+=1
        self.last_stamp=ns;self.last_pose=current
    def publish(self):
        config_hash=hashlib.sha256(Path(self.config_file).read_bytes()).hexdigest() if Path(self.config_file).is_file() else ''
        msg=DiagnosticArray();msg.header.stamp=self.get_clock().now().to_msg()
        msg.status=[diagnostic(f'r{self.rid}/session',dict(session_id=self.session_id,robot_id=self.rid,
            config_sha256=config_hash,reset_count=self.reset_count,last_raw_odom_stamp_ns=self.last_stamp))]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args);node=RobotSession()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
