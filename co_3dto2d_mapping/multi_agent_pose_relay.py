#!/usr/bin/env python3
"""Latest-only pose/TF process. No map, path, file or registration work."""
import math
import time
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from co_3dto2d_mapping.multi_agent_ros import stamp_ns,values_of,diagnostic,transformed_odometry


class MultiAgentPoseRelay(Node):
    def __init__(self):
        super().__init__('multi_agent_pose_relay')
        self.tf=TransformBroadcaster(self);self.alignment=None;self.epoch=None;self.blocked_epoch=None
        self.status_sessions={};self.sessions={};self.poses={};self.last_stamps={};self.receipt={}
        self.counts={0:0,1:0};self.last_callback_ms=0.;self.max_callback_ms=0.;self.last_clock=None
        self.received={0:0,1:0};self.callback_samples=deque(maxlen=500)
        self.latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.latest=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        # Keep output compatible with existing Reliable RViz/record clients.
        # Small pose messages retain depth 1; map traffic never enters this process.
        self.output_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(DiagnosticArray,'/multi_agent/alignment_status',self.status,self.latched)
        self.pubs={}
        for rid in (0,1):
            self.create_subscription(Odometry,f'/r{rid}/mapping/occupancy_mapper_odometry',
                                     lambda msg,r=rid:self.pose(msg,r),self.latest)
            self.create_subscription(DiagnosticArray,f'/multi_agent/r{rid}/session',lambda msg,r=rid:self.session(msg,r),self.latched)
            self.pubs[rid]=self.create_publisher(Odometry,f'/multi_agent/r{rid}/aligned_odometry',self.output_qos)
        self.diagnostics=self.create_publisher(DiagnosticArray,'/multi_agent/pose_relay_status',self.latched)
        self.create_timer(.05,self.broadcast_alignment)
        self.create_timer(1.,self.report)

    def session(self,msg,rid):
        identifier=values_of(msg).get('session_id')
        if identifier is None:return
        if rid in self.sessions and self.sessions[rid]!=identifier:
            self.alignment=None;self.blocked_epoch=self.epoch;self.poses.clear();self.last_stamps.clear()
        self.sessions[rid]=identifier

    def status(self,msg):
        value=values_of(msg);epoch=(value.get('central_instance'),value.get('generation',0))
        if epoch!=self.epoch:
            self.alignment=None;self.epoch=epoch;self.poses.clear();self.last_stamps.clear()
        self.status_sessions={int(k):v for k,v in value.get('sessions',{}).items()}
        transform=value.get('transform')
        if transform is None or value.get('state') in ('WAIT_INPUT','COLLECTING','SEARCHING','VERIFYING'):
            self.alignment=None;return
        if epoch==self.blocked_epoch:return
        # Both session identities must agree before a latched transform is used.
        if any(self.sessions.get(rid)!=self.status_sessions.get(rid) for rid in (0,1)):return
        if len(transform)==3 and np.all(np.isfinite(transform)):
            first=self.alignment is None;self.alignment=tuple(transform)
            if first:
                self.broadcast_alignment()
                for rid,pose in self.poses.items():self.publish_pose(pose,rid)

    def pose(self,msg,rid):
        started=time.perf_counter()
        if msg.header.frame_id!=f'r{rid}/odom':return
        p=msg.pose.pose;coords=[p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
        if not np.all(np.isfinite(coords)):return
        ns=stamp_ns(msg.header.stamp)
        if ns<self.last_stamps.get(rid,ns):
            self.alignment=None;self.blocked_epoch=self.epoch;self.poses.clear();self.last_stamps.clear()
        if ns==self.last_stamps.get(rid,-1):return
        self.last_stamps[rid]=ns;self.poses[rid]=msg;self.receipt[rid]=time.monotonic()
        self.received[rid]+=1
        t=TransformStamped();t.header=msg.header;t.child_frame_id=f'r{rid}/mapping_base_link'
        t.transform.translation.x=p.position.x;t.transform.translation.y=p.position.y;t.transform.translation.z=p.position.z
        t.transform.rotation=p.orientation;self.tf.sendTransform(t)
        if self.alignment is not None:self.publish_pose(msg,rid)
        self.last_callback_ms=(time.perf_counter()-started)*1000;self.max_callback_ms=max(self.max_callback_ms,self.last_callback_ms)
        self.callback_samples.append(self.last_callback_ms)

    def publish_pose(self,msg,rid):
        transform=self.alignment if rid else (0.,0.,0.)
        self.pubs[rid].publish(transformed_odometry(msg,rid,transform));self.counts[rid]+=1

    def broadcast_alignment(self):
        now=self.get_clock().now()
        if self.last_clock is not None and now.nanoseconds<self.last_clock:
            self.alignment=None;self.blocked_epoch=self.epoch
        self.last_clock=now.nanoseconds
        if self.alignment is None:return
        x,y,yaw=self.alignment;t=TransformStamped();t.header.stamp=now.to_msg()
        t.header.frame_id='r0/odom';t.child_frame_id='r1/odom'
        t.transform.translation.x=x;t.transform.translation.y=y
        t.transform.rotation.z=math.sin(yaw/2);t.transform.rotation.w=math.cos(yaw/2);self.tf.sendTransform(t)

    def report(self):
        msg=DiagnosticArray();msg.header.stamp=self.get_clock().now().to_msg()
        msg.status=[diagnostic('multi_agent/pose_relay',dict(alignment_valid=self.alignment is not None,
            epoch=self.epoch,published=self.counts,callback_ms=self.last_callback_ms,max_callback_ms=self.max_callback_ms,
            received=self.received,callback_p95_ms=float(np.percentile(self.callback_samples,95)) if self.callback_samples else None,
            receipt_ages_sec={r:time.monotonic()-t for r,t in self.receipt.items()},queue_depth=1,
            input_reliability='best_effort',output_reliability='reliable',source_timestamp_preserved=True))]
        self.diagnostics.publish(msg)


def main(args=None):
    rclpy.init(args=args);node=MultiAgentPoseRelay()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
