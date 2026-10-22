#!/usr/bin/env python3
"""Experiment 4: stationary startup alignment, composition, and final saving.

One fusion host owns only r0/odom -> r1/odom. Existing ICP TF and all single
estimators stay local. Full-circle search runs on immutable snapshots in one
worker; ROS callbacks never run the search synchronously.
"""
from __future__ import annotations

from collections import deque
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from dataclasses import asdict, fields
from pathlib import Path
import json
import math
import signal
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from co_3dto2d_mapping.multi_agent_alignment import register_startup, StartupConfig
from co_3dto2d_mapping.multi_agent_core import (
    Snapshot, StationaryConsensus, compose_grids, transform_xy,
    atomic_write, save_map, json_safe,
)
from co_3dto2d_mapping.occupancy_submap import GridGeometry, extract_local_patch
from co_3dto2d_mapping.planar_transform_utils import (
    world_from_source_odom, normalize_angle,
)
from co_3dto2d_mapping.se2_map_registration import RegistrationConfig


def stamp_ns(stamp): return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)
def yaw_of(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def pose_of(msg):
    p=msg.pose.pose
    return (float(p.position.x),float(p.position.y),yaw_of(p.orientation))

def snapshot_of(msg):
    g=msg.info; origin=g.origin
    return Snapshot(np.asarray(msg.data,dtype=np.int8).reshape(g.height,g.width),
        GridGeometry(g.resolution,g.width,g.height,origin.position.x,origin.position.y,yaw_of(origin.orientation)),
        stamp_ns(msg.header.stamp),msg.header.frame_id)


def grid_message(snapshot):
    msg=OccupancyGrid();g=snapshot.geometry
    msg.header.frame_id=snapshot.frame
    msg.header.stamp.sec=snapshot.stamp_ns//1_000_000_000
    msg.header.stamp.nanosec=snapshot.stamp_ns%1_000_000_000
    msg.info.resolution=g.resolution;msg.info.width=g.width;msg.info.height=g.height
    msg.info.origin.position.x=g.origin_x;msg.info.origin.position.y=g.origin_y
    msg.info.origin.orientation.z=math.sin(g.origin_yaw/2)
    msg.info.origin.orientation.w=math.cos(g.origin_yaw/2)
    msg.data=snapshot.grid.ravel().tolist()
    return msg


def diagnostic(name, values, level=DiagnosticStatus.OK, message=''):
    item=DiagnosticStatus(name=name,level=level,message=message)
    item.values=[KeyValue(key=str(k),value=json.dumps(json_safe(v),ensure_ascii=False)) for k,v in values.items()]
    return item


class MultiAgentMapping(Node):
    def __init__(self):
        super().__init__('multi_agent_mapping')
        defaults=dict(output_directory='outputs/experiment_04_multi_agent_mapping/runs/direct',
            mode='auto_validated_locked',manual_x_m=0.,manual_y_m=0.,manual_yaw_deg=0.,
            window_sec=3.,window_min_frames=15,window_max_frames=80,confirmation_windows=3,
            consensus_translation_m=.10,consensus_yaw_deg=.5,
            stationary_translation_m=.15,stationary_yaw_deg=2.,
            pose_stamp_tolerance_sec=.4,input_timeout_sec=5.,submap_radius_m=12.,
            registration_resolution_m=.10,output_resolution_m=.05,
            max_grid_cells=16000000,merged_period_sec=.5,checkpoint_sec=10.,
            require_sessions=True)
        for name,value in defaults.items():self.declare_parameter(name,value)
        self.p={name:self.get_parameter(name).value for name in defaults}
        if self.p['mode'] not in ('auto_validated_locked','manual_reference'):raise ValueError('invalid mode')
        if not all(math.isfinite(self.p[k]) for k in ('manual_x_m','manual_y_m','manual_yaw_deg')):
            raise ValueError('manual reference pose must be finite')
        for name in ('window_sec','pose_stamp_tolerance_sec','input_timeout_sec','submap_radius_m',
                     'registration_resolution_m','output_resolution_m','merged_period_sec','checkpoint_sec'):
            if not math.isfinite(self.p[name]) or self.p[name]<=0:raise ValueError('invalid '+name)
        if self.p['window_min_frames']<2 or self.p['window_max_frames']<self.p['window_min_frames']:
            raise ValueError('invalid window frame limits')
        reg={}
        for field in fields(RegistrationConfig):
            name='registration.'+field.name;self.declare_parameter(name,getattr(RegistrationConfig(),field.name))
            reg[field.name]=self.get_parameter(name).value
        self.reg=RegistrationConfig(**reg).validated()
        search={}
        for field in fields(StartupConfig):
            name='search.'+field.name;self.declare_parameter(name,getattr(StartupConfig(),field.name))
            search[field.name]=self.get_parameter(name).value
        self.search=StartupConfig(**search).validated()
        self.out=Path(self.p['output_directory']).expanduser().resolve();self.out.mkdir(parents=True,exist_ok=True)
        self.consensus=StationaryConsensus(self.p['confirmation_windows'],self.p['consensus_translation_m'],self.p['consensus_yaw_deg'])
        self.worker=ThreadPoolExecutor(max_workers=1,thread_name_prefix='startup-alignment')
        self.future=None;self.generation=0;self.case_number=0;self.alignment=None
        self.state='WAIT_INPUT';self.reason='waiting for maps, poses, and sessions'
        self.maps={};self.odom={};self.raw={};self.sessions={};self.receipt={};self.windows={i:[] for i in (0,1)}
        self.pending={i:deque(maxlen=3) for i in (0,1)}
        self.pose_history={i:deque(maxlen=400) for i in (0,1)}
        self.paths={kind:{i:deque(maxlen=10000) for i in (0,1)} for kind in ('aligned','raw')}
        self.last_local={};self.startup_origins={};self.map_dirty=True;self.merged=None;self.aligned_map=None
        self.last_checkpoint=0.;self.last_clock_ns=None;self.last_report={}
        self.closing=False
        self.latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf=TransformBroadcaster(self)
        self.transform_pub=self.create_publisher(TransformStamped,'/multi_agent/r1_to_r0_transform',self.latched)
        self.ready_pub=self.create_publisher(Bool,'/multi_agent/alignment_ready',self.latched)
        self.status_pub=self.create_publisher(DiagnosticArray,'/multi_agent/alignment_status',self.latched)
        self.merged_pub=self.create_publisher(OccupancyGrid,'/multi_agent/merged_global_occupancy',self.latched)
        self.aligned_map_pub=self.create_publisher(OccupancyGrid,'/multi_agent/r1/aligned_global_occupancy',self.latched)
        self.pose_pubs={};self.path_pubs={};self.local_map_subscriptions={}
        for rid in (0,1):
            prefix=f'/r{rid}/mapping/'
            # Input mapper grids are currently volatile. Subscriber durability
            # must stay compatible; the new central snapshots are latched.
            self.create_subscription(OccupancyGrid,prefix+'global_occupancy',lambda m,r=rid:self.global_map(m,r),1)
            self.create_subscription(Odometry,prefix+'occupancy_mapper_odometry',lambda m,r=rid:self.pose(m,r),50)
            self.create_subscription(Odometry,f'/r{rid}/odom',lambda m,r=rid:self.raw_pose(m,r),50)
            self.create_subscription(DiagnosticArray,f'/multi_agent/r{rid}/session',lambda m,r=rid:self.session(m,r),self.latched)
            self.pose_pubs[rid]=self.create_publisher(Odometry,f'/multi_agent/r{rid}/aligned_odometry',10)
            for kind in ('aligned','raw'):
                self.path_pubs[kind,rid]=self.create_publisher(RosPath,f'/multi_agent/r{rid}/{kind}_path' if kind=='aligned' else f'/multi_agent/r{rid}/raw_path',10)
        self.set_local_map_subscriptions(True)
        self.create_service(Trigger,'/multi_agent/save_map',self.save_service)
        self.create_service(Trigger,'/multi_agent/reset_alignment',self.reset_service)
        self.create_timer(self.p['merged_period_sec'],self.tick)
        self.ready_pub.publish(Bool(data=False))
        effective={**self.p,'registration':asdict(self.reg),'search':asdict(self.search)}
        atomic_write(self.out/'effective_central_config.json',json.dumps(effective,indent=2).encode())

    def set_local_map_subscriptions(self,enabled):
        # Ignoring callbacks after lock still transfers full grids over DDS.
        # Remove the readers themselves; recreate them on session/reset changes.
        if enabled:
            for rid in (0,1):
                if rid not in self.local_map_subscriptions:
                    self.local_map_subscriptions[rid]=self.create_subscription(
                        OccupancyGrid,f'/r{rid}/mapping/local_occupancy',
                        lambda msg,r=rid:self.local_map(msg,r),10)
        else:
            for subscription in self.local_map_subscriptions.values():
                self.destroy_subscription(subscription)
            self.local_map_subscriptions.clear()

    def invalidate(self,reason):
        # Archive previous-session saves instead of leaving stale final names.
        archived=self.out/'archive'/f'generation_{self.generation:03d}'
        for name in ('maps','alignment','checkpoint','save_status.json','checkpoint_status.json'):
            previous=self.out/name
            if previous.exists():
                archived.mkdir(parents=True,exist_ok=True)
                previous.rename(archived/name)
        self.generation+=1;self.alignment=None;self.consensus.reset();self.maps.clear();self.odom.clear();self.raw.clear()
        self.windows={i:[] for i in (0,1)};self.pending={i:deque(maxlen=3) for i in (0,1)}
        self.pose_history={i:deque(maxlen=400) for i in (0,1)}
        self.paths={kind:{i:deque(maxlen=10000) for i in (0,1)} for kind in ('aligned','raw')}
        self.last_local.clear();self.startup_origins.clear();self.receipt.clear();self.merged=None;self.aligned_map=None;self.map_dirty=True
        self.state='WAIT_INPUT';self.reason=reason;self.ready_pub.publish(Bool(data=False))
        self.set_local_map_subscriptions(True)
        # Replace latched map contents with empty state so RViz does not retain
        # an apparently valid merged map from a previous odometry session.
        empty=OccupancyGrid();empty.header.frame_id='r0/odom'
        self.merged_pub.publish(empty);self.aligned_map_pub.publish(empty)
        for (kind,rid),pub in self.path_pubs.items():
            path=RosPath();path.header.frame_id='r0/odom';pub.publish(path)
        self.get_logger().warn('Alignment invalidated: '+reason)

    def session(self,msg,rid):
        values={v.key:v.value for item in msg.status for v in item.values}
        session_id=values.get('session_id')
        if not session_id:return
        if rid in self.sessions and self.sessions[rid]!=session_id:self.invalidate(f'r{rid} session changed')
        self.sessions[rid]=session_id;self.receipt['session',rid]=time.monotonic()

    def global_map(self,msg,rid):
        if msg.header.frame_id!=f'r{rid}/odom':self.reason=f'invalid r{rid} map frame';return
        ns=stamp_ns(msg.header.stamp)
        if rid in self.maps and ns<self.maps[rid].stamp_ns:self.invalidate(f'r{rid} map clock reset')
        if rid in self.maps and ns==self.maps[rid].stamp_ns:return
        try:self.maps[rid]=snapshot_of(msg)
        except (ValueError,TypeError) as exc:self.reason=str(exc);return
        self.receipt['map',rid]=time.monotonic();self.map_dirty=True

    def pose(self,msg,rid):
        if msg.header.frame_id!=f'r{rid}/odom' or not np.all(np.isfinite(pose_of(msg))):return
        ns=stamp_ns(msg.header.stamp)
        if rid in self.odom and ns<stamp_ns(self.odom[rid].header.stamp):self.invalidate(f'r{rid} pose clock reset')
        if rid in self.odom and ns==stamp_ns(self.odom[rid].header.stamp):return
        self.odom[rid]=msg;self.pose_history[rid].append(msg);self.receipt['pose',rid]=time.monotonic()
        self.paths['aligned'][rid].append(msg)
        # Separate corrected frame, never compete for the raw base_link child.
        t=TransformStamped();t.header=deepcopy(msg.header);t.child_frame_id=f'r{rid}/mapping_base_link'
        p=msg.pose.pose;t.transform.translation.x=p.position.x;t.transform.translation.y=p.position.y;t.transform.translation.z=p.position.z
        t.transform.rotation=deepcopy(p.orientation);self.tf.sendTransform(t)
        if self.alignment is not None:self.publish_pose(msg,rid)

    def raw_pose(self,msg,rid):
        if msg.header.frame_id!=f'r{rid}/odom':return
        if self.raw.get(rid) is not None and stamp_ns(msg.header.stamp)<=stamp_ns(self.raw[rid].header.stamp):return
        self.raw[rid]=msg;self.paths['raw'][rid].append(msg)

    def local_map(self,msg,rid):
        if self.alignment is not None or msg.header.frame_id!=f'r{rid}/odom':return
        ns=stamp_ns(msg.header.stamp)
        if ns<=self.last_local.get(rid,-1):return
        if not self.pose_history[rid]:return
        pose=min(self.pose_history[rid],key=lambda p:abs(stamp_ns(p.header.stamp)-ns))
        if abs(stamp_ns(pose.header.stamp)-ns)>self.p['pose_stamp_tolerance_sec']*1e9:return
        self.last_local[rid]=ns
        try:snapshot=snapshot_of(msg)
        except (ValueError,TypeError):return
        window=self.windows[rid]
        if rid not in self.startup_origins:self.startup_origins[rid]=pose_of(pose)
        if rid in self.startup_origins:
            initial=self.startup_origins[rid];current=pose_of(pose)
            if (math.hypot(current[0]-initial[0],current[1]-initial[1])>self.p['stationary_translation_m'] or
                abs(normalize_angle(current[2]-initial[2]))>math.radians(self.p['stationary_yaw_deg'])):
                self.windows={i:[] for i in (0,1)};self.pending={i:deque(maxlen=3) for i in (0,1)}
                self.startup_origins.clear();self.consensus.reset()
                self.generation+=1;self.reason=f'r{rid} moved during startup';return
        window.append((snapshot,pose))
        if len(window)>self.p['window_max_frames']:
            self.windows[rid]=[];self.reason='window_max_frames too small for input rate';return
        if (len(window)>=self.p['window_min_frames'] and
            ns-window[0][0].stamp_ns>=self.p['window_sec']*1e9):
            self.pending[rid].append(tuple(window));self.windows[rid]=[]

    def fresh(self):
        now=time.monotonic();timeout=self.p['input_timeout_sec']
        keys=[(kind,rid) for rid in (0,1) for kind in ('map','pose')]
        if self.p['require_sessions']:keys += [('session',rid) for rid in (0,1)]
        return all(key in self.receipt and now-self.receipt[key]<=timeout for key in keys)

    def process_windows(self,w0,w1):
        patches=[]
        for rid,window in enumerate((w0,w1)):
            merged=compose_grids([(s,(0.,0.,0.)) for s,_ in window],self.p['registration_resolution_m'],f'r{rid}/odom',self.p['max_grid_cells'])
            patches.append(extract_local_patch(merged.grid,merged.geometry,pose_of(window[-1][1]),
                self.p['submap_radius_m'],output_resolution=self.p['registration_resolution_m']))
        report=register_startup(patches[0],patches[1],self.reg,self.search)
        if report['accepted']:
            report['odom_transform']=world_from_source_odom(pose_of(w0[-1][1]),report['transform'],pose_of(w1[-1][1]))
        return report,patches

    def lock(self,transform):
        self.alignment=tuple(map(float,transform));self.state='LOCKED';self.reason='startup alignment verified and frozen'
        self.set_local_map_subscriptions(False)
        self.windows={i:[] for i in (0,1)};self.pending={i:deque(maxlen=3) for i in (0,1)}
        self.map_dirty=True;self.ready_pub.publish(Bool(data=True))
        t=self.transform_message(self.get_clock().now().to_msg());self.transform_pub.publish(t)
        atomic_write(self.out/'alignment/r1_to_r0_transform.json',json.dumps(dict(
            parent='r0/odom',child='r1/odom',x_m=transform[0],y_m=transform[1],yaw_rad=transform[2],
            sessions=self.sessions,mode=self.p['mode']),indent=2).encode())
        import yaml
        atomic_write(self.out/'alignment/r1_to_r0_transform.yaml',yaml.safe_dump(dict(
            parent='r0/odom',child='r1/odom',x_m=float(transform[0]),y_m=float(transform[1]),
            yaw_rad=float(transform[2]),sessions=self.sessions,mode=self.p['mode'])).encode())
        self.get_logger().info(f'Alignment LOCKED: r0 <- r1 {self.alignment}')

    def transform_message(self,stamp):
        msg=TransformStamped();msg.header.frame_id='r0/odom';msg.child_frame_id='r1/odom';msg.header.stamp=stamp
        x,y,yaw=self.alignment
        msg.transform.translation.x=x;msg.transform.translation.y=y
        msg.transform.rotation.z=math.sin(yaw/2);msg.transform.rotation.w=math.cos(yaw/2)
        return msg

    def publish_pose(self,msg,rid):
        transform=self.alignment if rid else (0.,0.,0.)
        output=deepcopy(msg);p=output.pose.pose
        p.position.x,p.position.y=map(float,transform_xy([[p.position.x,p.position.y]],transform)[0])
        c,s=math.cos(transform[2]/2),math.sin(transform[2]/2);q=deepcopy(p.orientation)
        p.orientation.x=c*q.x-s*q.y;p.orientation.y=s*q.x+c*q.y
        p.orientation.z=c*q.z+s*q.w;p.orientation.w=c*q.w-s*q.z
        output.header.frame_id='r0/odom';output.child_frame_id=f'r{rid}/mapping_base_link'
        # Pose covariance rotates in the parent frame. Body-frame twist stays
        # in its child convention; it is not rotated as if it were map velocity.
        r=np.eye(6);cc,ss=math.cos(transform[2]),math.sin(transform[2])
        r[:2,:2]=[[cc,-ss],[ss,cc]];r[3:5,3:5]=r[:2,:2]
        output.pose.covariance=(r@np.array(msg.pose.covariance).reshape(6,6)@r.T).ravel().tolist()
        self.pose_pubs[rid].publish(output)

    def publish_paths(self):
        for kind in ('aligned','raw'):
            for rid in (0,1):
                path=RosPath();path.header.frame_id='r0/odom'
                transform=self.alignment if rid else (0.,0.,0.)
                for odom in self.paths[kind][rid]:
                    p=PoseStamped();p.header=deepcopy(odom.header);p.header.frame_id='r0/odom';p.pose=deepcopy(odom.pose.pose)
                    p.pose.position.x,p.pose.position.y=map(float,transform_xy([[p.pose.position.x,p.pose.position.y]],transform)[0])
                    # Paths show position; keep orientation consistent as well.
                    q=deepcopy(p.pose.orientation);c,s=math.cos(transform[2]/2),math.sin(transform[2]/2)
                    p.pose.orientation.x=c*q.x-s*q.y;p.pose.orientation.y=s*q.x+c*q.y
                    p.pose.orientation.z=c*q.z+s*q.w;p.pose.orientation.w=c*q.w-s*q.z
                    path.poses.append(p)
                if path.poses:path.header.stamp=path.poses[-1].header.stamp
                self.path_pubs[kind,rid].publish(path)

    def compose(self):
        if self.alignment is None or set(self.maps)!={0,1}:return
        started=time.perf_counter()
        self.aligned_map=compose_grids([(self.maps[1],self.alignment)],self.p['output_resolution_m'],max_cells=self.p['max_grid_cells'])
        self.merged=compose_grids([(self.maps[0],(0.,0.,0.)),(self.maps[1],self.alignment)],
                                 self.p['output_resolution_m'],max_cells=self.p['max_grid_cells'])
        self.map_dirty=False
        self.last_report['merge_runtime_ms']=(time.perf_counter()-started)*1000

    def tick(self):
        tick_started=time.perf_counter()
        now_ns=self.get_clock().now().nanoseconds
        if self.last_clock_ns is not None and now_ns<self.last_clock_ns:self.invalidate('ROS clock moved backwards')
        self.last_clock_ns=now_ns
        if self.future is not None and self.future.done():
            future,meta=self.future,self.future_meta;self.future=None
            try:
                report,patches=future.result()
                if meta['generation']==self.generation and self.alignment is None:
                    self.case_number+=1;self.last_report=report
                    self.save_case(meta,report,patches)
                    if report['accepted'] and self.fresh():
                        fixed=self.consensus.add(report['odom_transform'],meta['windows'])
                        if fixed is not None:self.lock(fixed)
                        else:self.state='VERIFYING';self.reason=f'consistent windows {len(self.consensus.evidence)}/{self.consensus.count}'
                    else:
                        self.consensus.reset();self.state='SEARCHING';self.reason=report['reason']
            except Exception as exc:
                self.consensus.reset();self.state='SEARCHING';self.reason=f'worker failed: {exc}';self.get_logger().error(self.reason)
        fresh=self.fresh()
        if self.alignment is None and fresh:
            if self.p['mode']=='manual_reference':
                self.lock((self.p['manual_x_m'],self.p['manual_y_m'],math.radians(self.p['manual_yaw_deg'])))
            elif self.future is None and all(self.pending[i] for i in (0,1)):
                w0,w1=self.pending[0].popleft(),self.pending[1].popleft()
                self.future_meta=dict(generation=self.generation,windows=(w0[0][0].stamp_ns,w0[-1][0].stamp_ns,w1[0][0].stamp_ns,w1[-1][0].stamp_ns),
                                      poses=[pose_of(w0[-1][1]),pose_of(w1[-1][1])])
                self.future=self.worker.submit(self.process_windows,w0,w1);self.state='SEARCHING';self.reason='full-circle yaw/XY search running'
        if self.alignment is not None:
            self.tf.sendTransform(self.transform_message(self.get_clock().now().to_msg()))
            self.transform_pub.publish(self.transform_message(self.get_clock().now().to_msg()))
            if fresh and self.map_dirty:
                try:self.compose()
                except ValueError as exc:self.reason=str(exc)
            if fresh and self.merged is not None:
                self.merged_pub.publish(grid_message(self.merged));self.aligned_map_pub.publish(grid_message(self.aligned_map))
            self.publish_paths()
            if self.merged is not None and time.monotonic()-self.last_checkpoint>=self.p['checkpoint_sec']:
                self.save_outputs('checkpoint');self.last_checkpoint=time.monotonic()
        now=time.monotonic()
        required=[(kind,rid) for rid in (0,1) for kind in ('map','pose')]
        if self.p['require_sessions']:required += [('session',rid) for rid in (0,1)]
        ages={f'r{rid}/{kind}':now-self.receipt[kind,rid] if (kind,rid) in self.receipt else None
              for kind,rid in required}
        missing=[key for key,age in ages.items() if age is None or age>self.p['input_timeout_sec']]
        fresh=not missing
        state=self.state if fresh else 'STALE' if self.alignment else 'WAIT_INPUT'
        if state=='WAIT_INPUT' and fresh:state='COLLECTING'
        reason=self.reason if not missing else 'waiting for fresh input: '+', '.join(missing)
        self.last_report['central_tick_runtime_ms']=(time.perf_counter()-tick_started)*1000
        status=dict(state=state,reason=reason,
            ready=self.alignment is not None and fresh,inputs_fresh=fresh,transform=self.alignment,sessions=self.sessions,
            source_map_stamps={r:s.stamp_ns for r,s in self.maps.items()},
            consistent_windows=len(self.consensus.evidence),required_windows=self.consensus.count,
            input_receipt_ages_sec=ages,missing_inputs=missing,
            local_map_subscriptions_active=bool(self.local_map_subscriptions),
            window_frames={r:len(w) for r,w in self.windows.items()},metrics=self.last_report)
        msg=DiagnosticArray();msg.header.stamp=self.get_clock().now().to_msg()
        msg.status=[diagnostic('multi_agent/startup_alignment',status,DiagnosticStatus.OK if self.alignment and fresh else DiagnosticStatus.WARN,reason)]
        self.status_pub.publish(msg)
        self.ready_pub.publish(Bool(data=self.alignment is not None and fresh))

    def save_case(self,meta,report,patches):
        directory=self.out/'alignment_cases'/f'{self.case_number:03d}'
        details={**meta,**report}
        atomic_write(directory/'result.json',json.dumps(json_safe(details),indent=2).encode())
        for name,patch in zip(('r0','r1'),patches):
            buffer=__import__('io').BytesIO();np.savez_compressed(buffer,grid=patch.grid,resolution=patch.resolution,
                origin=[patch.origin_x,patch.origin_y],boundary=patch.boundary_points)
            atomic_write(directory/(name+'_patch.npz'),buffer.getvalue())
        # Self-contained review of accepted/rejected geometry. Colors and
        # labels distinguish before/after; full maps are saved separately.
        target=patches[0].boundary_points.tolist();source=patches[1].boundary_points.tolist()
        data=json.dumps(json_safe(dict(target=target,source=source,result=report)))
        html='''<!doctype html><html lang="ko"><meta charset="utf-8"><title>초기 정합 검토</title>
<style>body{font:15px system-ui;background:#15212b;color:white;margin:20px}canvas{max-width:100%;background:#20303c}button,select{margin:8px}pre{white-space:pre-wrap}</style>
<h1>초기 정합 후보 · r1 → r0</h1><label>후보 <select id="candidate"></select></label>
<p>흰색: r0 · 회색: 변환 전 r1 · 주황색: 후보 변환 후 r1</p><canvas id="map" width="950" height="700"></canvas><pre id="info"></pre>
<script>const data=PAYLOAD,sel=document.getElementById('candidate'),cv=document.getElementById('map'),ctx=cv.getContext('2d');
const candidates=data.result.candidates||[];candidates.forEach((v,i)=>{const o=document.createElement('option');o.value=i;o.textContent=(i+1)+' yaw '+(v.transform[2]*180/Math.PI).toFixed(2)+'° '+v.reason;sel.append(o)});
function draw(){const v=candidates[Number(sel.value)]||{transform:[0,0,0]},t=v.transform,c=Math.cos(t[2]),s=Math.sin(t[2]);const aligned=data.source.map(p=>[t[0]+c*p[0]-s*p[1],t[1]+s*p[0]+c*p[1]]),all=[...data.target,...data.source,...aligned];if(!all.length)return;const xs=all.map(p=>p[0]),ys=all.map(p=>p[1]),xmin=Math.min(...xs)-.5,xmax=Math.max(...xs)+.5,ymin=Math.min(...ys)-.5,ymax=Math.max(...ys)+.5,k=Math.min(910/(xmax-xmin),660/(ymax-ymin));ctx.clearRect(0,0,950,700);for(const [pts,color]of[[data.source,'#718096'],[data.target,'white'],[aligned,'#ffad55']]){ctx.fillStyle=color;for(const p of pts)ctx.fillRect(20+(p[0]-xmin)*k,680-(p[1]-ymin)*k,2,2)}document.getElementById('info').textContent=JSON.stringify({accepted:data.result.accepted,reason:data.result.reason,score_gap:data.result.score_gap_ratio,candidate:v},null,2)}sel.onchange=draw;draw();</script></html>'''.replace('PAYLOAD',data)
        atomic_write(directory/'inspection.html',html.encode())

    def save_outputs(self,stage):
        try:
            if self.alignment is not None and self.map_dirty:self.compose()
            directory=self.out/('checkpoint' if stage=='checkpoint' else 'maps')
            if self.merged is not None:
                paths={}
                for kind in ('aligned','raw'):
                    for rid in (0,1):
                        transform=self.alignment if rid else (0.,0.,0.)
                        paths[f'r{rid}_{kind}']=transform_xy([[m.pose.pose.position.x,m.pose.pose.position.y]
                            for m in self.paths[kind][rid]],transform).tolist()
                save_map(directory,'merged_global_map',self.merged,paths)
                save_map(directory,'r0_global_map',self.maps[0])
                save_map(directory,'r1_aligned_global_map',self.aligned_map)
                atomic_write(directory/'paths.json',json.dumps(paths).encode())
            else:
                for rid,snapshot in self.maps.items():save_map(directory,f'r{rid}_unaligned_global_map',snapshot)
            atomic_write(self.out/('checkpoint_status.json' if stage=='checkpoint' else 'save_status.json'),json.dumps(json_safe(dict(
                stage=stage,merged_saved=self.merged is not None,alignment=self.alignment,
                source_map_stamps={r:s.stamp_ns for r,s in self.maps.items()},sessions=self.sessions,
                inputs_fresh=self.fresh(),reason=self.reason,metrics=self.last_report)),indent=2).encode())
            return self.merged is not None
        except Exception as exc:
            self.get_logger().error(f'Map save failed: {exc}');return False

    def save_service(self,request,response):
        response.success=self.save_outputs('requested');response.message=str(self.out);return response
    def reset_service(self,request,response):
        self.invalidate('explicit reset requested');response.success=True;response.message='stationary startup reset';return response

    def close(self):
        if self.closing:return
        self.closing=True
        self.worker.shutdown(wait=True,cancel_futures=True)
        self.save_outputs('final')


def main(args=None):
    # Keep the context alive during shutdown saving. The runner stops producers
    # before requesting this node's clean shutdown, giving callbacks drain time.
    rclpy.init(args=args,signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node=MultiAgentMapping();stopping=False
    def stop(*_):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        while rclpy.ok() and not stopping:rclpy.spin_once(node,timeout_sec=.1)
    finally:
        node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
