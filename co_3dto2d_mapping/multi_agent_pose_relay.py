#!/usr/bin/env python3
"""Latest-only pose/TF process. No map, path, file or registration work."""
import math
import time
from collections import deque
from copy import deepcopy
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from rclpy.duration import Duration
from rclpy.time import Time
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster,Buffer,TransformListener,TransformException
from co_3dto2d_mapping.multi_agent_ros import stamp_ns,values_of,diagnostic,transformed_odometry
from co_3dto2d_mapping.planar_transform_utils import compose_planar
from co_3dto2d_mapping.multi_agent_frames import rigid_matrix,rigid_components,world_from_odom


class MultiAgentPoseRelay(Node):
    def __init__(self):
        super().__init__('multi_agent_pose_relay')
        self.tf=TransformBroadcaster(self);self.alignment=None;self.epoch=None;self.blocked_epoch=None
        self.buffer=Buffer(cache_time=Duration(seconds=10.),node=self);self.listener=TransformListener(self.buffer,self)
        self.world_frame='map';self.world_from_reference=None
        self.mapping_frames={r:f'r{r}/mapping_odom' for r in (0,1)}
        self.corrections={};self.published_stamps={};self.tf_waits={0:0,1:0};self.bad_frames=set()
        self.pending={r:deque(maxlen=32) for r in (0,1)};self.published_at={};self.pending_expired={0:0,1:0}
        for key,value in dict(pose_relay_pose_timeout_sec=2.,pose_relay_tf_wait_timeout_sec=1.).items():self.declare_parameter(key,value)
        self.pose_timeout=float(self.get_parameter('pose_relay_pose_timeout_sec').value)
        self.tf_wait_timeout=float(self.get_parameter('pose_relay_tf_wait_timeout_sec').value)
        if not all(math.isfinite(v) and v>0 for v in (self.pose_timeout,self.tf_wait_timeout)):
            raise ValueError('invalid pose/TF timeout')
        self.status_sessions={};self.sessions={};self.poses={};self.last_stamps={};self.receipt={}
        self.counts={0:0,1:0};self.last_callback_ms=0.;self.max_callback_ms=0.;self.last_clock=None
        self.received={0:0,1:0};self.callback_samples=deque(maxlen=500)
        self.latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.latest=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        # Keep output compatible with existing Reliable RViz/record clients.
        # Small pose messages retain depth 1; map traffic never enters this process.
        self.output_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(DiagnosticArray,'/multi_agent/alignment_status',self.status,self.latched)
        self.pubs={};self.pose_pubs={}
        for rid in (0,1):
            self.create_subscription(Odometry,f'/r{rid}/mapping/occupancy_mapper_odometry',
                                     lambda msg,r=rid:self.pose(msg,r),self.latest)
            self.create_subscription(DiagnosticArray,f'/multi_agent/r{rid}/session',lambda msg,r=rid:self.session(msg,r),self.latched)
            self.pubs[rid]=self.create_publisher(Odometry,f'/multi_agent/r{rid}/aligned_odometry',self.output_qos)
            self.pose_pubs[rid]=self.create_publisher(PoseWithCovarianceStamped,f'/multi_agent/r{rid}/global_pose',self.output_qos)
        self.diagnostics=self.create_publisher(DiagnosticArray,'/multi_agent/pose_relay_status',self.latched)
        self.create_timer(.05,self.broadcast_alignment)
        self.create_timer(1.,self.report)

    def session(self,msg,rid):
        identifier=values_of(msg).get('session_id')
        if identifier is None:return
        if rid in self.sessions and self.sessions[rid]!=identifier:
            self.alignment=None;self.blocked_epoch=self.epoch;self.poses.clear();self.last_stamps.clear()
            self.corrections.clear();self.published_stamps.clear()
            self.clear_pending()
        self.sessions[rid]=identifier

    def status(self,msg):
        value=values_of(msg);epoch=(value.get('central_instance'),value.get('generation',0))
        if epoch!=self.epoch:
            self.alignment=None;self.epoch=epoch;self.poses.clear();self.last_stamps.clear()
            self.corrections.clear();self.published_stamps.clear()
            self.clear_pending()
        self.status_sessions={int(k):v for k,v in value.get('sessions',{}).items()}
        transform=value.get('transform')
        anchor=value.get('world_from_reference')
        if transform is None or anchor is None or value.get('state') in ('WAIT_INPUT','COLLECTING','SEARCHING','VERIFYING'):
            self.alignment=None;self.corrections.clear();return
        if epoch==self.blocked_epoch:return
        # Both session identities must agree before a latched transform is used.
        if any(self.sessions.get(rid)!=self.status_sessions.get(rid) for rid in (0,1)):return
        if len(transform)==3 and len(anchor)==3 and np.all(np.isfinite([transform,anchor])):
            self.world_frame=value.get('world_frame','map');self.world_from_reference=tuple(anchor)
            self.mapping_frames={int(r):frame for r,frame in value.get('mapping_frames',self.mapping_frames).items()}
            first=self.alignment is None;self.alignment=tuple(transform)
            if first:
                self.broadcast_alignment()
                for rid in self.poses:self.publish_pose(rid)

    def pose(self,msg,rid):
        started=time.perf_counter()
        if msg.header.frame_id!=self.mapping_frames[rid]:
            key=(rid,msg.header.frame_id)
            if key not in self.bad_frames:
                self.get_logger().warn(f'Unexpected r{rid} mapping pose frame {msg.header.frame_id!r}; expected {self.mapping_frames[rid]!r}. Update both agent launch/configs.')
                self.bad_frames.add(key)
            return
        p=msg.pose.pose;coords=[p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
        if not np.all(np.isfinite(coords)):return
        ns=stamp_ns(msg.header.stamp)
        if ns<self.last_stamps.get(rid,ns):
            self.alignment=None;self.blocked_epoch=self.epoch;self.poses.clear();self.last_stamps.clear()
            self.corrections.clear();self.published_stamps.clear()
            self.clear_pending()
            self.buffer.clear()  # A device/source clock reset may precede ROS clock reset.
        if ns==self.last_stamps.get(rid,-1):return
        self.last_stamps[rid]=ns;self.poses[rid]=msg;self.receipt[rid]=time.monotonic()
        if len(self.pending[rid])==self.pending[rid].maxlen:self.pending_expired[rid]+=1
        self.pending[rid].append((msg,self.receipt[rid]))
        self.received[rid]+=1
        if self.alignment is not None:self.publish_pose(rid)
        self.last_callback_ms=(time.perf_counter()-started)*1000;self.max_callback_ms=max(self.max_callback_ms,self.last_callback_ms)
        self.callback_samples.append(self.last_callback_ms)

    @staticmethod
    def matrix_message(matrix,parent,child,stamp):
        position,q=rigid_components(matrix)
        t=TransformStamped();t.header.frame_id=parent;t.header.stamp=stamp;t.child_frame_id=child
        t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=position
        t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=q
        return t

    def clear_pending(self):
        for queue in self.pending.values():queue.clear()
        self.published_at.clear()

    def publish_pose(self,rid):
        if self.alignment is None:return
        queue=self.pending[rid];now=time.monotonic()
        while queue and (now-queue[0][1]>self.tf_wait_timeout or
                         stamp_ns(queue[0][0].header.stamp)<=self.published_stamps.get(rid,-1)):
            old,received=queue.popleft()
            if now-received>self.tf_wait_timeout:self.pending_expired[rid]+=1
        # Choose the newest timestamp whose raw TF is available. Keeping a
        # short bounded buffer avoids starvation when TF consistently arrives
        # later than mapping poses; no old poses are replayed as a backlog.
        for msg,_ in reversed(queue):
            try:
                raw=self.buffer.lookup_transform(f'r{rid}/odom',f'r{rid}/base_link',Time.from_msg(msg.header.stamp))
                break
            except TransformException:self.tf_waits[rid]+=1
        else:return
        ns=stamp_ns(msg.header.stamp)
        transform=compose_planar(self.world_from_reference,self.alignment) if rid else self.world_from_reference
        out=transformed_odometry(msg,rid,transform,self.world_frame,f'r{rid}/base_link')
        p=out.pose.pose;t=raw.transform
        try:
            world_base=rigid_matrix([p.position.x,p.position.y,p.position.z],
                [p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
            _,q=rigid_components(world_base)
            p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=q
            odom_base=rigid_matrix([t.translation.x,t.translation.y,t.translation.z],
                [t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w])
            correction=self.matrix_message(world_from_odom(world_base,odom_base),self.world_frame,f'r{rid}/odom',msg.header.stamp)
            alias=self.matrix_message(world_base,self.world_frame,f'r{rid}/mapping_base_link',msg.header.stamp)
        except ValueError:
            return
        self.tf.sendTransform([correction,alias]);self.corrections[rid]=correction
        self.pubs[rid].publish(out)
        pose=PoseWithCovarianceStamped();pose.header=deepcopy(out.header);pose.pose=deepcopy(out.pose)
        self.pose_pubs[rid].publish(pose);self.counts[rid]+=1;self.published_stamps[rid]=ns;self.published_at[rid]=now
        while queue and stamp_ns(queue[0][0].header.stamp)<=ns:queue.popleft()

    def broadcast_alignment(self):
        now=self.get_clock().now()
        if self.last_clock is not None and now.nanoseconds<self.last_clock:
            self.alignment=None;self.blocked_epoch=self.epoch;self.corrections.clear()
            self.clear_pending();self.published_stamps.clear();self.poses.clear();self.last_stamps.clear()
        self.last_clock=now.nanoseconds
        if self.alignment is None:return
        transforms=[]
        for rid in (0,1):
            transform=compose_planar(self.world_from_reference,self.alignment) if rid else self.world_from_reference
            x,y,yaw=transform;t=TransformStamped();t.header.stamp=now.to_msg()
            t.header.frame_id=self.world_frame;t.child_frame_id=self.mapping_frames[rid]
            t.transform.translation.x=x;t.transform.translation.y=y
            t.transform.rotation.z=math.sin(yaw/2);t.transform.rotation.w=math.cos(yaw/2);transforms.append(t)
            self.publish_pose(rid)  # Retry when raw TF arrived after the pose callback.
            # map->raw odom is published at matched source timestamps only.
            # Restamping a held correction at now would insert old values after
            # newer source-time estimates when sensor/transport latency varies.
        self.tf.sendTransform(transforms)

    def report(self):
        msg=DiagnosticArray();msg.header.stamp=self.get_clock().now().to_msg()
        msg.status=[diagnostic('multi_agent/pose_relay',dict(alignment_valid=self.alignment is not None,
            epoch=self.epoch,published=self.counts,callback_ms=self.last_callback_ms,max_callback_ms=self.max_callback_ms,
            received=self.received,callback_p95_ms=float(np.percentile(self.callback_samples,95)) if self.callback_samples else None,
            world_frame=self.world_frame,world_from_reference=self.world_from_reference,
            raw_tf_wait_count=self.tf_waits,last_global_pose_source_stamps_ns=self.published_stamps,
            raw_tf_correction_available=list(self.corrections),pose_timeout_sec=self.pose_timeout,
            pending_pose_count={r:len(q) for r,q in self.pending.items()},pending_pose_expired=self.pending_expired,
            latest_source_lag_sec={r:(self.last_stamps[r]-self.published_stamps[r])/1e9
                if r in self.last_stamps and r in self.published_stamps else None for r in (0,1)},
            global_pose_fresh={r:self.alignment is not None and r in self.published_at and
                time.monotonic()-self.published_at[r]<=self.pose_timeout for r in (0,1)},
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
