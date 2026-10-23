#!/usr/bin/env python3
"""ROS integration probe with synthetic map messages, never physical GT."""

from pathlib import Path as _WorkspacePath
import sys as _workspace_sys
_workspace_root = next(p for p in _WorkspacePath(__file__).resolve().parents if (p/"package.xml").is_file() and (p/"CMakeLists.txt").is_file())
_workspace_sys.path.insert(0, str(_workspace_root))
import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import signal
import subprocess
import time
import statistics
import yaml

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import OccupancyGrid,Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from co_3dto2d_mapping.multi_agent.core import Snapshot,transform_xy
from co_3dto2d_mapping.common.occupancy_submap import GridGeometry
from co_3dto2d_mapping.common.planar_transform_utils import invert_planar,normalize_angle
from co_3dto2d_mapping.multi_agent.mapping import grid_message,diagnostic,yaw_of


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--production-config',type=Path,help='use actual startup window/search parameters')
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    # Keep full-circle yaw coverage; use smaller point budget/window duration
    # only in this integration probe. Production YAML is tested separately.
    params=(yaml.safe_load(args.production_config.read_text()) if args.production_config else {'/**':{'ros__parameters':dict(
        startup_accumulation_sec=.5,verification_interval_sec=.4,confirmation_windows=3,
        **{'registration.search_max_points':180,'registration.coarse_translation_range_m':1.,'registration.min_correspondences':50})}})
    config=args.output/'probe_config.yaml';config.write_text(yaml.safe_dump(params))
    command=['ros2','launch','co_3dto2d_mapping','experiment_04_multi_agent.launch.py','role:=fusion',
        'alignment_config_file:='+str(config.resolve()),'output_directory:='+str(args.output.resolve())]
    log=(args.output/'node.log').open('wb')
    proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=Node('experiment04_synthetic_probe')
    latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    state=dict(ready=False,transform=None,merged=None,aligned=None,pose=None,status=None,pose_count=0,reliable_pose_count=0)
    published={};latencies=[]
    node.create_subscription(Bool,'/multi_agent/alignment_ready',lambda m:state.update(ready=m.data),latched)
    node.create_subscription(TransformStamped,'/multi_agent/r1_to_r0_transform',lambda m:state.update(transform=m),latched)
    node.create_subscription(OccupancyGrid,'/multi_agent/merged_global_occupancy',lambda m:state.update(merged=m),latched)
    node.create_subscription(OccupancyGrid,'/multi_agent/r1/aligned_global_occupancy',lambda m:state.update(aligned=m),latched)
    def got_pose(msg):
        state.update(pose=msg,pose_count=state['pose_count']+1)
        key=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if key in published:latencies.append(time.monotonic()-published[key])
    node.create_subscription(Odometry,'/multi_agent/r1/aligned_odometry',got_pose,qos_profile_sensor_data)
    node.create_subscription(Odometry,'/multi_agent/r1/aligned_odometry',
        lambda m:state.update(reliable_pose_count=state['reliable_pose_count']+1),
        QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
    node.create_subscription(DiagnosticArray,'/multi_agent/alignment_status',lambda m:state.update(status=m),latched)
    publishers={}
    for rid in (0,1):
        for suffix in ('global_occupancy','local_occupancy'):
            publishers[rid,suffix]=node.create_publisher(OccupancyGrid,f'/r{rid}/mapping/{suffix}',10)
        publishers[rid,'pose']=node.create_publisher(Odometry,f'/r{rid}/mapping/occupancy_mapper_odometry',10)
        publishers[rid,'raw']=node.create_publisher(Odometry,f'/r{rid}/odom',10)
        publishers[rid,'session']=node.create_publisher(DiagnosticArray,f'/multi_agent/r{rid}/session',latched)
    grid=np.zeros((81,101),dtype=np.int8)
    grid[10:70,10]=100;grid[10:70,89]=100;grid[10,10:90]=100;grid[69,10:90]=100
    grid[21:48,39]=100;grid[47,39:60]=100;grid[54:61,65:72]=100
    truth=(.4,-.3,math.pi/2);inverse=invert_planar(truth)
    origin=transform_xy([[-5.,-4.]],inverse)[0]
    snapshots=[Snapshot(grid,GridGeometry(.1,101,81,-5.,-4.),0,'r0/odom'),
               Snapshot(grid,GridGeometry(.1,101,81,*origin,inverse[2]),0,'r1/odom')]
    sessions=['synthetic0','synthetic1'];last_publish=0.;publish_global=True;extension=False
    def spin():
        nonlocal last_publish
        rclpy.spin_once(node,timeout_sec=.02)
        now=time.monotonic()
        if now-last_publish<.10:return
        last_publish=now;stamp=node.get_clock().now().to_msg()
        published[stamp.sec*10**9+stamp.nanosec]=time.monotonic()
        for rid in (0,1):
            session=DiagnosticArray();session.status=[diagnostic('session',dict(session_id=sessions[rid]))]
            publishers[rid,'session'].publish(session)
            pose=Odometry();pose.header.stamp=stamp;pose.header.frame_id=f'r{rid}/odom';pose.child_frame_id=f'r{rid}/base_link';pose.pose.pose.orientation.w=1.
            publishers[rid,'pose'].publish(pose);publishers[rid,'raw'].publish(pose)
            msg=grid_message(snapshots[rid]);msg.header.stamp=stamp
            if publish_global:publishers[rid,'global_occupancy'].publish(msg)
            if extension and rid==1:
                # New occupied observation outside the initial global seed.
                point=transform_xy([[6.,1.]],inverse)[0]
                msg=grid_message(Snapshot([[100]],GridGeometry(.1,1,1,*map(float,point)),0,'r1/odom'));msg.header.stamp=stamp
            publishers[rid,'local_occupancy'].publish(msg)
    try:
        deadline=time.monotonic()+60
        while time.monotonic()<deadline:
            if proc.poll() is not None:raise RuntimeError('central process exited; see node.log')
            spin()
            if state['ready'] and state['merged'] and state['aligned'] and state['pose']:break
        assert state['ready'],'did not reach ready'
        t=state['transform'];found=(t.transform.translation.x,t.transform.translation.y,yaw_of(t.transform.rotation))
        assert math.hypot(found[0]-truth[0],found[1]-truth[1])<.10,found
        assert abs(normalize_angle(found[2]-truth[2]))<math.radians(1.),found
        assert t.header.frame_id=='r0/odom' and t.child_frame_id=='r1/odom'
        assert state['merged'].header.frame_id=='r0/odom'
        assert state['aligned'].header.frame_id=='r0/odom'
        assert state['pose'].header.frame_id=='r0/odom'
        assert state['pose'].child_frame_id=='r1/mapping_base_link'
        assert node.count_subscribers('/r1/mapping/global_occupancy')==0,'global stream still subscribed after lock'
        assert node.count_subscribers('/r1/mapping/local_occupancy')==1
        publish_global=False;extension=True
        deadline=time.monotonic()+4
        while time.monotonic()<deadline:spin()
        assert state['reliable_pose_count']>0,'Reliable pose subscriber received no output'
        merged=state['merged'];origin=merged.info.origin.position
        values=np.asarray(merged.data).reshape(merged.info.height,merged.info.width)
        rr,cc=np.nonzero(values>50)
        xy=np.column_stack((origin.x+(cc+.5)*merged.info.resolution,origin.y+(rr+.5)*merged.info.resolution))
        assert np.any((xy[:,0]>5.8)&(xy[:,1]>.8)&(xy[:,1]<1.3)),'local-only extension missing or double-transformed'
        client=node.create_client(Trigger,'/multi_agent/save_map')
        assert client.wait_for_service(timeout_sec=3.)
        future=client.call_async(Trigger.Request())
        deadline=time.monotonic()+5
        while not future.done() and time.monotonic()<deadline:spin()
        assert future.done() and future.result().success,'map save failed'
        saved=np.load(args.output/'maps/merged_global_map_data.npz')
        assert saved['frame']=='r0/odom' and np.any(saved['data']==100)
        assert (args.output/'maps/merged_global_map.yaml').is_file()
        cases=list((args.output/'alignment_cases').glob('*/result.json'))
        required=params['/**']['ros__parameters'].get('confirmation_windows',1)
        accepted=[json.loads(path.read_text()) for path in cases]
        assert sum(case.get('accepted',False) for case in accepted)>=required,'insufficient successful startup checks'
        # Freeze publishing while resetting the source session: ready must be
        # invalidated; no stale merged map may look like a successful new map.
        sessions[1]='synthetic1_restart'
        state['ready']=True;deadline=time.monotonic()+3
        while time.monotonic()<deadline and state['ready']:spin()
        assert not state['ready'],'session restart did not invalidate alignment'
        count=state['pose_count'];deadline=time.monotonic()+1
        while time.monotonic()<deadline:spin()
        assert state['pose_count']<=count+1,'pose relay continued using invalidated transform'
        assert node.count_subscribers('/r1/mapping/global_occupancy')==1,'startup global subscription was not restored'
        summary=dict(validation='synthetic ROS integration, not a physical multi-agent accuracy result',
            truth=truth,estimated=found,translation_error_m=math.hypot(found[0]-truth[0],found[1]-truth[1]),
            yaw_error_deg=math.degrees(abs(normalize_angle(found[2]-truth[2]))),
            confirmation_cases=len(cases),required_confirmations=required,map_save=True,session_reset=True,local_only_extension=True,
            reliable_pose_compatible=state['reliable_pose_count']>0,
            pose_latency_median_ms=round(statistics.median(latencies)*1000,3) if latencies else None,
            pose_latency_p95_ms=round(float(np.percentile(latencies,95))*1000,3) if latencies else None,
            pose_latency_max_ms=round(max(latencies)*1000,3) if latencies else None,
            search_modes=[__import__('json').loads(p.read_text()).get('search_mode') for p in cases])
        (args.output/'validation.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
    finally:
        # Stop the publisher first, then let central finish its final save.
        import os
        if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=30)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=10)
        node.destroy_node();rclpy.shutdown();log.close()
if __name__=='__main__':main()
