#!/usr/bin/env python3
"""Replay captured native grids/poses into the real central node, without ICP.

Uses the recorded initial transform and pre-lock seeds, original headers and
one source clock. It does not publish old merged maps or old central TF.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('capture',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--periodic',action='store_true');p.add_argument('--rate',type=float,default=.5)
    p.add_argument('--domain-id',type=int,default=174)
    args=p.parse_args()
    if not 0<args.rate<=1:raise ValueError('rate must be in (0,1] for this deterministic central replay')
    # Local offline experiment: never discover either live robot.
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id),ROS_LOCALHOST_ONLY='1')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid,Odometry
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import TransformStamped
    from rosgraph_msgs.msg import Clock
    from co_3dto2d_mapping.multi_agent_bag import read_capture,fusion_inputs
    from co_3dto2d_mapping.multi_agent_ros import grid_message,stamp_ns
    c=read_capture(args.capture);seeds,updates=fusion_inputs(c)
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    config=ROOT/'config/experiments/experiment_04_multi_agent_mapping'/('alignment_periodic.yaml' if args.periodic else 'alignment.yaml')
    (out/'alignment_config.yaml').write_bytes(config.read_bytes())
    hashes={}
    for src in (ROOT/'co_3dto2d_mapping').glob('multi_agent*.py'):
        content=src.read_bytes();target=out/'source_snapshot'/src.name;target.parent.mkdir(exist_ok=True)
        target.write_bytes(content);hashes[str(src.relative_to(ROOT))]=hashlib.sha256(content).hexdigest()
    x,y,yaw=c['alignment']
    command=[sys.executable,str(ROOT/'co_3dto2d_mapping/multi_agent_mapping.py'),'--ros-args',
        '--params-file',str(out/'alignment_config.yaml'),'-p','use_sim_time:=true','-p','mode:=manual_reference',
        '-p',f'manual_x_m:={x}','-p',f'manual_y_m:={y}','-p',f'manual_yaw_deg:={math.degrees(yaw)}',
        '-p',f'output_directory:={out}']
    (out/'replay.json').write_text(json.dumps(dict(capture=str(args.capture.resolve()),command=command,
        rate=args.rate,domain=args.domain_id,source_order='source timestamp, robot id',local_frames=len(updates),
        initial_alignment=c['alignment'],seed_stamps={r:s.stamp_ns for r,s in seeds.items()},source_sha256=hashes),indent=2))
    env=dict(os.environ,PYTHONPATH=str(ROOT)+':'+os.environ.get('PYTHONPATH',''))
    rclpy.init();node=Node('experiment_04_canonical_inputs')
    reliable=QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE)
    latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    maps={r:node.create_publisher(OccupancyGrid,f'/r{r}/mapping/local_occupancy',reliable) for r in (0,1)}
    globals_={r:node.create_publisher(OccupancyGrid,f'/r{r}/mapping/global_occupancy',latched) for r in (0,1)}
    poses={r:node.create_publisher(Odometry,f'/r{r}/mapping/occupancy_mapper_odometry',reliable) for r in (0,1)}
    sessions={r:node.create_publisher(DiagnosticArray,f'/multi_agent/r{r}/session',latched) for r in (0,1)}
    clock=node.create_publisher(Clock,'/clock',reliable);locked=[]
    node.create_subscription(TransformStamped,'/multi_agent/r1_to_r0_transform',lambda m:locked.append(m),latched)
    def publish_clock(ns):
        msg=Clock();msg.clock.sec=ns//10**9;msg.clock.nanosec=ns%10**9;clock.publish(msg)
    child=None
    try:
        with (out/'node.log').open('w') as log:
            child=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            first=min(stamp_ns(c['pose_messages'][r][0].header.stamp) for r in (0,1));started=time.monotonic()
            while not locked:
                if child.poll() is not None:raise RuntimeError('central exited during bootstrap; see node.log')
                if time.monotonic()-started>20:raise TimeoutError('central did not accept recorded initial alignment')
                publish_clock(max(0,first-30*10**9)+round((time.monotonic()-started)*1e8))
                for r in (0,1):
                    sessions[r].publish(c['sessions'][r]);poses[r].publish(c['pose_messages'][r][0])
                    globals_[r].publish(grid_message(seeds[r]))
                rclpy.spin_once(node,timeout_sec=.1)
            events=[(s.stamp_ns,r,'map',s) for r,s in updates]
            events.extend((stamp_ns(msg.header.stamp),r,'pose',msg) for r in (0,1)
                          for msg in c['pose_messages'][r] if stamp_ns(msg.header.stamp)>first)
            events.sort(key=lambda v:(v[0],v[1],v[2]));source0=events[0][0]
            real0=time.monotonic();index=0;last_report=real0
            while index<len(events):
                now=time.monotonic();ns=source0+round((now-real0)*args.rate*1e9);publish_clock(ns)
                if child.poll() is not None:raise RuntimeError('central exited during replay; see node.log')
                while index<len(events) and events[index][0]<=ns:
                    _,r,kind,value=events[index]
                    if kind=='map':maps[r].publish(grid_message(value))
                    else:poses[r].publish(value)
                    index+=1
                # Sessions are identities, not new observation evidence.
                if now-last_report>1:
                    for r in (0,1):sessions[r].publish(c['sessions'][r])
                    last_report=now
                rclpy.spin_once(node,timeout_sec=.005)
            finish=time.monotonic()
            while time.monotonic()-finish<4:
                publish_clock(events[-1][0]+round((time.monotonic()-finish)*1e9))
                rclpy.spin_once(node,timeout_sec=.02)
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=40)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=10)
        node.destroy_node();rclpy.shutdown()
        (out/'process_status.json').write_text(json.dumps(dict(node_exit=child.returncode if child else None),indent=2))
    if child.returncode:raise RuntimeError('central failed; see node.log')
    print('Central replay complete:',out,flush=True)

if __name__=='__main__':main()
