#!/usr/bin/env python3
"""Compare on-robot receipt rates with fusion-PC receipt rates, without clouds.

Run --role local on the PC owning --robot-id. Run --role receiver on r0.
Map messages are observed only in local mode, as serialized bytes. Measurements
are subscriber observations, not instrumented publisher counters or wire latency.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import socket
import time
import uuid


class StreamStats:
    def __init__(self):
        self.count=0;self.bytes=0;self.first=None;self.last=None;self.max_gap=0.
        self.first_stamp=None;self.last_stamp=None;self.duplicates=0;self.backward=0;self.ages=deque(maxlen=1000)

    def add(self,now,size=0,stamp=None,ros_now=None):
        if self.first is None:self.first=now
        if self.last is not None:self.max_gap=max(self.max_gap,now-self.last)
        self.last=now;self.count+=1;self.bytes+=size
        if stamp is not None:
            if self.first_stamp is None:self.first_stamp=stamp
            if self.last_stamp==stamp:self.duplicates+=1
            if self.last_stamp is not None and stamp<self.last_stamp:self.backward+=1
            self.last_stamp=stamp
            if ros_now is not None:self.ages.append((ros_now-stamp)/1e9)

    def summary(self,now):
        ages=sorted(self.ages)
        span=self.last-self.first if self.count>1 else 0.
        return dict(count=self.count,receipt_hz=(self.count-1)/span if span else 0.,
            serialized_bytes=self.bytes,max_gap_sec=self.max_gap,
            receipt_age_sec=now-self.last if self.last is not None else None,
            first_header_stamp_ns=self.first_stamp,last_header_stamp_ns=self.last_stamp,
            duplicate_stamps=self.duplicates,backward_stamps=self.backward,
            header_age_median_sec=ages[len(ages)//2] if ages else None)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--role',choices=('local','receiver'),required=True)
    p.add_argument('--robot-id',type=int,choices=(0,1),required=True)
    p.add_argument('--duration',type=float,default=40.)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if not math.isfinite(args.duration) or args.duration<=0:p.error('duration must be positive')
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
    from nav_msgs.msg import Odometry,OccupancyGrid
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    rclpy.init();node=rclpy.create_node(f'experiment04_{args.role}_monitor_{args.robot_id}')
    latest=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    small=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    rid=args.robot_id;prefix=f'/r{rid}';start=time.monotonic();identity=str(uuid.uuid4())
    stats={};remote=[];snapshots=[];last_report=0.;subscriptions=[]
    diagnostic_topic=f'/multi_agent/r{rid}/pipeline_metrics'
    publisher=node.create_publisher(DiagnosticArray,diagnostic_topic,small) if args.role=='local' else None
    def observe(topic,msg):
        now=time.monotonic()
        if isinstance(msg,bytes):stats[topic].add(now,len(msg));return
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        stats[topic].add(now,stamp=stamp,ros_now=node.get_clock().now().nanoseconds)
    topics=[prefix+'/odom',prefix+'/mapping/floor_based_odometry',prefix+'/mapping/occupancy_mapper_odometry']
    if args.role=='receiver':topics.append(f'/multi_agent/r{rid}/aligned_odometry')
    for topic in topics:
        stats[topic]=StreamStats()
        subscriptions.append(node.create_subscription(Odometry,topic,lambda m,t=topic:observe(t,m),latest))
    if args.role=='local':
        for topic in (prefix+'/mapping/local_occupancy',prefix+'/mapping/global_occupancy'):
            stats[topic]=StreamStats()
            subscriptions.append(node.create_subscription(OccupancyGrid,topic,lambda m,t=topic:observe(t,m),latest,raw=True))
    def receive_metrics(msg):
        for status in msg.status:
            for item in status.values:
                if item.key=='metrics':
                    try:remote.append(dict(receiver_elapsed_sec=time.monotonic()-start,report=json.loads(item.value)))
                    except (ValueError,TypeError):pass
    if args.role=='receiver':subscriptions.append(node.create_subscription(DiagnosticArray,diagnostic_topic,receive_metrics,small))
    def report():
        now=time.monotonic()
        result=dict(role=args.role,robot_id=rid,hostname=socket.gethostname(),monitor_id=identity,
            elapsed_sec=now-start,report_ros_stamp_ns=node.get_clock().now().nanoseconds,
            topics={topic:value.summary(now) for topic,value in stats.items()},
            note='Subscriber observations with Best Effort depth 1. Header age includes clock offset; no raw cloud subscription.')
        snapshots.append(result)
        if publisher is not None:
            msg=DiagnosticArray();msg.header.stamp=node.get_clock().now().to_msg()
            msg.status=[DiagnosticStatus(name=f'r{rid}/pipeline_metrics',level=bytes([0]),message='on-robot subscriber observation',
                values=[KeyValue(key='metrics',value=json.dumps(result))])]
            publisher.publish(msg)
        print(json.dumps(dict(elapsed_sec=round(now-start,1),receipt_hz={t:round(s['receipt_hz'],2) for t,s in result['topics'].items()},local_reports_received=len(remote))),flush=True)
        return result
    try:
        while time.monotonic()-start<args.duration:
            rclpy.spin_once(node,timeout_sec=.05)
            if time.monotonic()-last_report>=1.:
                report();last_report=time.monotonic()
    except KeyboardInterrupt:pass
    finally:
        final=report();args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(dict(final=final,snapshots=snapshots,local_reports=remote,
            interpretation='Compare overlapping elapsed intervals and matching monitor_id. Different hosts require clock-sync checks for header-age comparisons. Local/remote subscribers may independently drop samples.'),indent=2))
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
