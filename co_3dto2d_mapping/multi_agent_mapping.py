#!/usr/bin/env python3
"""Global-snapshot startup registration and worker-owned local-map fusion.

Pose/TF delivery belongs to the independent multi_agent_pose_relay process.
"""
from collections import Counter,deque
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from dataclasses import asdict,fields,replace
from pathlib import Path
import json
import math
import signal
import time
import uuid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus
from nav_msgs.msg import OccupancyGrid,Odometry,Path as RosPath
from geometry_msgs.msg import TransformStamped,PoseStamped
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from co_3dto2d_mapping.multi_agent_alignment import register_startup,StartupConfig
from co_3dto2d_mapping.multi_agent_core import SnapshotConsensus,IncrementalFusion,atomic_write,save_map,json_safe,transform_xy
from co_3dto2d_mapping.multi_agent_ros import stamp_ns,yaw_of,pose_of,snapshot_of,grid_message,diagnostic,values_of,transformed_odometry
from co_3dto2d_mapping.occupancy_submap import extract_local_patch
from co_3dto2d_mapping.planar_transform_utils import world_from_source_odom,normalize_angle
from co_3dto2d_mapping.se2_map_registration import RegistrationConfig


class MultiAgentMapping(Node):
    def __init__(self):
        super().__init__('multi_agent_mapping')
        defaults=dict(output_directory='outputs/experiment_04_multi_agent_mapping/runs/direct',
            mode='auto_validated_locked',manual_x_m=0.,manual_y_m=0.,manual_yaw_deg=0.,
            startup_accumulation_sec=3.,verification_interval_sec=1.,confirmation_windows=1,
            consensus_translation_m=.10,consensus_yaw_deg=.5,stationary_translation_m=.15,stationary_yaw_deg=2.,
            input_timeout_sec=5.,submap_radius_m=12.,registration_resolution_m=.10,output_resolution_m=.05,
            max_grid_cells=16000000,merged_period_sec=.5,checkpoint_sec=10.,require_sessions=True,
            local_queue_size=100,path_publish_period_sec=1.,path_max_poses=5000,
            verification_prior_growth_ratio=.25,verification_translation_range_m=.15)
        for key,value in defaults.items():self.declare_parameter(key,value)
        self.p={key:self.get_parameter(key).value for key in defaults}
        if self.p['mode'] not in ('auto_validated_locked','manual_reference'):raise ValueError('invalid mode')
        for key in ('startup_accumulation_sec','verification_interval_sec','input_timeout_sec','submap_radius_m',
                    'registration_resolution_m','output_resolution_m','merged_period_sec','checkpoint_sec','path_publish_period_sec'):
            if not math.isfinite(self.p[key]) or self.p[key]<=0:raise ValueError('invalid '+key)
        if self.p['local_queue_size']<1 or self.p['confirmation_windows']<1:raise ValueError('invalid queue/confirmation count')
        rd=RegistrationConfig(max_free_conflict_ratio=.18);reg={}
        for field in fields(rd):
            name='registration.'+field.name;self.declare_parameter(name,getattr(rd,field.name));reg[field.name]=self.get_parameter(name).value
        self.reg=RegistrationConfig(**reg).validated();search={}
        for field in fields(StartupConfig):
            name='search.'+field.name;self.declare_parameter(name,getattr(StartupConfig(),field.name));search[field.name]=self.get_parameter(name).value
        self.search=StartupConfig(**search).validated()
        self.out=Path(self.p['output_directory']).expanduser().resolve();self.out.mkdir(parents=True,exist_ok=True)
        self.instance=str(uuid.uuid4());self.generation=0;self.case_number=0;self.alignment=None
        self.consensus=SnapshotConsensus(self.p['confirmation_windows'],self.p['consensus_translation_m'],self.p['consensus_yaw_deg'])
        self.search_worker=ThreadPoolExecutor(max_workers=1,thread_name_prefix='startup-search')
        self.map_worker=ThreadPoolExecutor(max_workers=1,thread_name_prefix='incremental-fusion')
        self.io_worker=ThreadPoolExecutor(max_workers=1,thread_name_prefix='map-save')
        self.path_worker=ThreadPoolExecutor(max_workers=1,thread_name_prefix='path-build')
        self.path_future=None
        self.fusion=IncrementalFusion(self.p['output_resolution_m'],self.p['max_grid_cells'])
        self.search_future=None;self.map_future=None;self.io_jobs=[];self.checkpoint_future=None
        self.maps={};self.layers={};self.merged=None;self.aligned_map=None;self.odom={};self.raw={};self.sessions={}
        self.receipt={};self.first_global_stamps={};self.last_attempt_stamps={};self.last_local={};self.anchors={}
        self.local_input_stats={rid:Counter() for rid in (0,1)};self.updates=deque();self.pending_seeds={}
        self.paths={kind:{r:deque(maxlen=self.p['path_max_poses']) for r in (0,1)} for kind in ('aligned','raw')}
        self.cached_modes=None;self.cached_sizes=None;self.last_report={};self.state='WAIT_INPUT';self.reason='waiting for stationary cumulative global maps'
        self.last_checkpoint=0.;self.last_map_submit=0.;self.last_path_publish=0.;self.last_status=0.;self.last_clock=None;self.closing=False
        self.latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.latest=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.transform_pub=self.create_publisher(TransformStamped,'/multi_agent/r1_to_r0_transform',self.latched)
        self.ready_pub=self.create_publisher(Bool,'/multi_agent/alignment_ready',self.latched)
        self.status_pub=self.create_publisher(DiagnosticArray,'/multi_agent/alignment_status',self.latched)
        self.merged_pub=self.create_publisher(OccupancyGrid,'/multi_agent/merged_global_occupancy',self.latched)
        self.aligned_map_pub=self.create_publisher(OccupancyGrid,'/multi_agent/r1/aligned_global_occupancy',self.latched)
        self.path_pubs={};self.local_map_subscriptions={};self.global_map_subscriptions={}
        for rid in (0,1):
            self.create_subscription(Odometry,f'/r{rid}/mapping/occupancy_mapper_odometry',lambda m,r=rid:self.pose(m,r),self.latest)
            self.create_subscription(Odometry,f'/r{rid}/odom',lambda m,r=rid:self.raw_pose(m,r),self.latest)
            self.create_subscription(DiagnosticArray,f'/multi_agent/r{rid}/session',lambda m,r=rid:self.session(m,r),self.latched)
            for kind in ('aligned','raw'):
                self.path_pubs[kind,rid]=self.create_publisher(RosPath,f'/multi_agent/r{rid}/{kind}_path',1)
        self.set_subscriptions('global',True)
        self.create_service(Trigger,'/multi_agent/save_map',self.save_service)
        self.create_service(Trigger,'/multi_agent/reset_alignment',self.reset_service)
        self.create_timer(.1,self.tick);self.ready_pub.publish(Bool(data=False))
        atomic_write(self.out/'effective_central_config.json',json.dumps({**self.p,'registration':asdict(self.reg),'search':asdict(self.search)},indent=2).encode())

    def set_subscriptions(self,kind,enabled,ids=(0,1)):
        subscriptions=self.global_map_subscriptions if kind=='global' else self.local_map_subscriptions
        callback=self.global_map if kind=='global' else self.local_map
        for rid in ids:
            if enabled and rid not in subscriptions:
                subscriptions[rid]=self.create_subscription(OccupancyGrid,f'/r{rid}/mapping/{kind}_occupancy',lambda msg,r=rid:callback(msg,r),1 if kind=='global' else 10)
            elif not enabled and rid in subscriptions:self.destroy_subscription(subscriptions.pop(rid))

    def required_input_keys(self):
        kind='map' if self.alignment is None else 'local_map'
        keys=[(name,rid) for rid in (0,1) for name in (kind,'pose')]
        if self.p['require_sessions']:keys += [('session',rid) for rid in (0,1)]
        return keys

    def fresh(self):
        now=time.monotonic()
        return all(key in self.receipt and now-self.receipt[key]<=self.p['input_timeout_sec'] for key in self.required_input_keys())

    def invalidate(self,reason):
        # Finish old IO before archiving: an old checkpoint must not overwrite a new session.
        for future in self.io_jobs:
            try:future.result(timeout=90)
            except Exception as exc:self.get_logger().error(str(exc))
        self.io_jobs.clear()
        archive=self.out/'archive'/f'generation_{self.generation:03d}'
        for name in ('maps','alignment','checkpoint','save_status.json','checkpoint_status.json'):
            p=self.out/name
            if p.exists():archive.mkdir(parents=True,exist_ok=True);p.rename(archive/name)
        self.generation+=1;self.alignment=None;self.consensus.reset();self.maps.clear();self.layers.clear();self.merged=None;self.aligned_map=None
        self.receipt.clear();self.first_global_stamps.clear();self.last_attempt_stamps.clear();self.last_local.clear();self.anchors.clear()
        self.odom.clear();self.raw.clear();self.updates.clear();self.pending_seeds.clear();self.cached_modes=None;self.cached_sizes=None
        self.local_input_stats={rid:Counter() for rid in (0,1)}
        for group in self.paths.values():
            for path in group.values():path.clear()
        self.set_subscriptions('local',False);self.set_subscriptions('global',True)
        self.state='WAIT_INPUT';self.reason=reason;self.ready_pub.publish(Bool(data=False))
        empty=OccupancyGrid();empty.header.frame_id='r0/odom';self.merged_pub.publish(empty);self.aligned_map_pub.publish(empty)
        for pub in self.path_pubs.values():path=RosPath();path.header.frame_id='r0/odom';pub.publish(path)
        self.publish_status();self.get_logger().warn('Alignment invalidated: '+reason)

    def session(self,msg,rid):
        identifier=values_of(msg).get('session_id')
        if identifier is None:return
        if rid in self.sessions and identifier!=self.sessions[rid]:self.invalidate(f'r{rid} session changed')
        self.sessions[rid]=identifier;self.receipt['session',rid]=time.monotonic()

    def pose(self,msg,rid):
        if msg.header.frame_id!=f'r{rid}/odom' or not np.all(np.isfinite(pose_of(msg))):return
        ns=stamp_ns(msg.header.stamp)
        if rid in self.odom and ns<stamp_ns(self.odom[rid].header.stamp):self.invalidate(f'r{rid} pose clock reset')
        if rid in self.odom and ns==stamp_ns(self.odom[rid].header.stamp):return
        self.odom[rid]=msg;self.receipt['pose',rid]=time.monotonic();self.paths['aligned'][rid].append(msg)
        current=pose_of(msg)
        if self.alignment is None:
            if rid not in self.anchors:self.anchors[rid]=current
            anchor=self.anchors[rid]
            if math.hypot(current[0]-anchor[0],current[1]-anchor[1])>self.p['stationary_translation_m'] or abs(normalize_angle(current[2]-anchor[2]))>math.radians(self.p['stationary_yaw_deg']):
                self.generation+=1;self.consensus.reset();self.cached_modes=None
                self.anchors={r:pose_of(m) for r,m in self.odom.items()}
                self.first_global_stamps={r:s.stamp_ns for r,s in self.maps.items()};self.last_attempt_stamps.clear()
                self.reason=f'r{rid} moved; stationary accumulation restarted'

    def raw_pose(self,msg,rid):
        if msg.header.frame_id!=f'r{rid}/odom':return
        if rid in self.raw and stamp_ns(msg.header.stamp)<=stamp_ns(self.raw[rid].header.stamp):return
        self.raw[rid]=msg;self.paths['raw'][rid].append(msg)

    def global_map(self,msg,rid):
        if msg.header.frame_id!=f'r{rid}/odom':return
        ns=stamp_ns(msg.header.stamp)
        if rid in self.maps and ns<self.maps[rid].stamp_ns:self.invalidate(f'r{rid} map clock reset');return
        if rid in self.maps and ns==self.maps[rid].stamp_ns:return
        try:snapshot=snapshot_of(msg)
        except (ValueError,TypeError):return
        self.maps[rid]=snapshot;self.receipt['map',rid]=time.monotonic()
        self.first_global_stamps.setdefault(rid,ns)
        if self.alignment is not None:
            self.pending_seeds[rid]=snapshot;self.set_subscriptions('global',False,(rid,))
            self.local_input_stats[rid]['global_resync_received']+=1

    def local_map(self,msg,rid):
        counts=self.local_input_stats[rid];counts['received']+=1
        if self.alignment is None or msg.header.frame_id!=f'r{rid}/odom':counts['invalid_phase_or_frame']+=1;return
        ns=stamp_ns(msg.header.stamp)
        if ns<=self.last_local.get(rid,-1):counts['duplicate_or_seeded_stamp']+=1;return
        try:snapshot=snapshot_of(msg)
        except (ValueError,TypeError):counts['invalid_grid']+=1;return
        self.last_local[rid]=ns;self.receipt['local_map',rid]=time.monotonic();counts['accepted']+=1
        if len(self.updates)>=self.p['local_queue_size']:
            dropped_rid,_=self.updates.popleft();self.local_input_stats[dropped_rid]['queue_overflow']+=1
            # Recover missed observations with a single full snapshot, then return to local-only transport.
            self.set_subscriptions('global',True,(dropped_rid,))
        self.updates.append((rid,snapshot))

    def process_snapshots(self,snapshots,anchors,priors):
        patches=[extract_local_patch(s.grid,s.geometry,anchors[r],self.p['submap_radius_m'],output_resolution=self.p['registration_resolution_m']) for r,s in enumerate(snapshots)]
        sizes=tuple(p.occupied_boundary_count for p in patches)
        if priors and self.cached_sizes and any(abs(n-o)/max(1,o)>self.p['verification_prior_growth_ratio'] for n,o in zip(sizes,self.cached_sizes)):priors=None
        started=time.perf_counter()
        reg=replace(self.reg,fine_translation_range_m=self.p['verification_translation_range_m']) if priors else self.reg
        report=register_startup(patches[0],patches[1],reg,self.search,priors)
        if priors and not report['accepted']:
            report=register_startup(patches[0],patches[1],self.reg,self.search);report['fast_fallback']=True
        report['runtime_ms']=(time.perf_counter()-started)*1000
        report.setdefault('search_mode','cached_modes' if priors else 'full_circle')
        if report['accepted']:report['odom_transform']=world_from_source_odom(anchors[0],report['transform'],anchors[1])
        report['boundary_sizes']=sizes
        return report,patches

    def lock(self,transform):
        self.alignment=tuple(map(float,transform));self.state='LOCKED';self.reason='startup global snapshots verified; transform frozen'
        self.pending_seeds=dict(self.maps);self.last_local={r:s.stamp_ns for r,s in self.maps.items()}
        self.set_subscriptions('global',False);self.set_subscriptions('local',True)
        t=TransformStamped();t.header.stamp=self.get_clock().now().to_msg();t.header.frame_id='r0/odom';t.child_frame_id='r1/odom'
        t.transform.translation.x=transform[0];t.transform.translation.y=transform[1]
        t.transform.rotation.z=math.sin(transform[2]/2);t.transform.rotation.w=math.cos(transform[2]/2);self.transform_pub.publish(t)
        payload=dict(parent='r0/odom',child='r1/odom',x_m=transform[0],y_m=transform[1],yaw_rad=transform[2],sessions=dict(self.sessions),mode=self.p['mode'])
        def write_alignment():
            import yaml
            atomic_write(self.out/'alignment/r1_to_r0_transform.json',json.dumps(payload,indent=2).encode())
            atomic_write(self.out/'alignment/r1_to_r0_transform.yaml',yaml.safe_dump(payload).encode())
        self.io_jobs.append(self.io_worker.submit(write_alignment));self.publish_status()
        self.get_logger().info(f'Alignment LOCKED: r0 <- r1 {self.alignment}')

    def tick(self):
        started=time.perf_counter();now=time.monotonic();clock=self.get_clock().now().nanoseconds
        if self.last_clock is not None and clock<self.last_clock:self.invalidate('ROS clock moved backwards')
        self.last_clock=clock
        if self.search_future is not None and self.search_future.done():
            future,meta=self.search_future,self.search_meta;self.search_future=None
            try:
                report,patches=future.result()
                if meta['generation']==self.generation and self.alignment is None:
                    self.case_number+=1;meta['case_number']=self.case_number;self.last_report=dict(report)
                    self.io_jobs.append(self.io_worker.submit(self.save_case,deepcopy(meta),deepcopy(report),patches))
                    if report['accepted'] and self.fresh():
                        fixed=self.consensus.add(report['odom_transform'],meta['stamps'])
                        self.cached_modes=[c['transform'] for c in report['candidates']];self.cached_sizes=report['boundary_sizes']
                        if fixed is not None:self.lock(fixed)
                        else:self.state='VERIFYING';self.reason=f'consistent cumulative snapshots {len(self.consensus.evidence)}/{self.consensus.count}'
                    else:
                        self.consensus.reset();self.cached_modes=None;self.state='SEARCHING'
                        self.reason=report['reason'] if not report['accepted'] else 'valid candidate deferred: inputs stale'
            except Exception as exc:self.consensus.reset();self.state='SEARCHING';self.reason=str(exc);self.get_logger().error(str(exc))
        if self.alignment is None and self.search_future is None and self.fresh() and len(self.maps)==2 and len(self.anchors)==2:
            if self.p['mode']=='manual_reference':self.lock((self.p['manual_x_m'],self.p['manual_y_m'],math.radians(self.p['manual_yaw_deg'])))
            elif all(s.stamp_ns-self.first_global_stamps[r]>=self.p['startup_accumulation_sec']*1e9 and s.stamp_ns-self.last_attempt_stamps.get(r,0)>=self.p['verification_interval_sec']*1e9 for r,s in self.maps.items()):
                snapshots=tuple(self.maps[r] for r in (0,1));stamps=tuple(s.stamp_ns for s in snapshots)
                self.last_attempt_stamps=dict(enumerate(stamps))
                # First and final checks search all yaw modes. Only intermediate verification reuses prior modes.
                priors=self.cached_modes if 0<len(self.consensus.evidence)<self.consensus.count-1 else None
                self.search_meta=dict(generation=self.generation,stamps=stamps,poses=[self.anchors[r] for r in (0,1)],evidence_kind='advancing cumulative global snapshots')
                self.search_future=self.search_worker.submit(self.process_snapshots,snapshots,dict(self.anchors),deepcopy(priors))
                self.state='SEARCHING';self.reason='cached-mode verification' if priors else 'full-circle global-map search'
            elif self.state=='WAIT_INPUT':self.state='COLLECTING';self.reason='waiting for stationary global-map accumulation'
        if self.map_future is not None and self.map_future.done():
            future,generation=self.map_future,self.map_generation;self.map_future=None
            try:
                merged,layers,runtime=future.result()
                if generation==self.generation and self.alignment is not None:
                    self.merged,self.layers=merged,layers;self.aligned_map=layers.get(1);self.last_report['merge_runtime_ms']=runtime
                    if merged is not None:self.merged_pub.publish(grid_message(merged))
                    if self.aligned_map is not None:self.aligned_map_pub.publish(grid_message(self.aligned_map))
            except Exception as exc:self.reason='map worker failed: '+str(exc);self.get_logger().error(self.reason)
        if self.alignment is not None and self.map_future is None and (self.pending_seeds or self.updates) and now-self.last_map_submit>=self.p['merged_period_sec']:
            seeds,self.pending_seeds=self.pending_seeds,{};updates=list(self.updates);self.updates.clear();self.map_generation=self.generation
            alignment=self.alignment;generation=self.generation
            def update_maps():
                began=time.perf_counter();merged,layers=self.fusion.update(generation,seeds,updates,alignment)
                return merged,layers,(time.perf_counter()-began)*1000
            self.map_future=self.map_worker.submit(update_maps);self.last_map_submit=now
        if self.path_future is not None and self.path_future.done():
            try:
                generation,paths=self.path_future.result()
                if generation==self.generation and self.alignment is not None:
                    for key,path in paths.items():self.path_pubs[key].publish(path)
            except Exception as exc:self.get_logger().error('Path worker failed: '+str(exc))
            self.path_future=None
        if self.alignment is not None and self.path_future is None and now-self.last_path_publish>=self.p['path_publish_period_sec']:
            paths={kind:{r:tuple(v) for r,v in group.items()} for kind,group in self.paths.items()}
            self.path_future=self.path_worker.submit(self.build_paths,paths,self.alignment,self.generation);self.last_path_publish=now
        if self.merged is not None and now-self.last_checkpoint>=self.p['checkpoint_sec'] and (self.checkpoint_future is None or self.checkpoint_future.done()):
            self.checkpoint_future=self.io_worker.submit(self.write_bundle,self.capture_bundle('checkpoint'));self.io_jobs.append(self.checkpoint_future);self.last_checkpoint=now
        self.io_jobs=[f for f in self.io_jobs if not f.done()]
        self.last_report['central_tick_runtime_ms']=(time.perf_counter()-started)*1000
        if now-self.last_status>=.5:self.publish_status();self.last_status=now

    def publish_status(self):
        now=time.monotonic();ages={f'r{rid}/{kind}':now-self.receipt[kind,rid] if (kind,rid) in self.receipt else None for kind,rid in self.required_input_keys()}
        missing=[k for k,v in ages.items() if v is None or v>self.p['input_timeout_sec']]
        state=self.state if not missing else ('STALE' if self.alignment else 'WAIT_INPUT')
        if self.alignment is not None and self.merged is None:state='SEEDING_MAP'
        ready=self.alignment is not None and self.merged is not None and not missing
        values=dict(state=state,reason=self.reason if not missing else 'waiting for fresh input: '+', '.join(missing),ready=ready,
            transform=self.alignment,sessions=self.sessions,generation=self.generation,central_instance=self.instance,
            consistent_windows=len(self.consensus.evidence),required_windows=self.consensus.count,input_receipt_ages_sec=ages,missing_inputs=missing,
            local_map_subscriptions_active=bool(self.local_map_subscriptions),global_map_subscriptions_active=bool(self.global_map_subscriptions),
            local_input_stats=self.local_input_stats,pending_local_updates=len(self.updates),source_map_stamps={r:s.stamp_ns for r,s in self.maps.items()},
            startup_accumulation_sec=self.p['startup_accumulation_sec'],snapshot_spans_sec={r:(s.stamp_ns-self.first_global_stamps[r])/1e9 for r,s in self.maps.items() if r in self.first_global_stamps},
            metrics=self.last_report,evidence_kind='cumulative snapshot stability; not independent observations')
        msg=DiagnosticArray();msg.header.stamp=self.get_clock().now().to_msg();msg.status=[diagnostic('multi_agent/startup_alignment',values,DiagnosticStatus.OK if ready else DiagnosticStatus.WARN,values['reason'])]
        self.status_pub.publish(msg);self.ready_pub.publish(Bool(data=ready))

    @staticmethod
    def build_paths(paths,alignment,generation):
        result={}
        for kind in ('aligned','raw'):
            for rid in (0,1):
                path=RosPath();path.header.frame_id='r0/odom';transform=alignment if rid else (0.,0.,0.)
                poses=paths[kind][rid];xy=transform_xy([[m.pose.pose.position.x,m.pose.pose.position.y] for m in poses],transform)
                c,s=math.cos(transform[2]/2),math.sin(transform[2]/2)
                for odom,point in zip(poses,xy):
                    pose=PoseStamped();pose.header=deepcopy(odom.header);pose.header.frame_id='r0/odom';pose.pose=deepcopy(odom.pose.pose)
                    pose.pose.position.x,pose.pose.position.y=map(float,point);q=deepcopy(pose.pose.orientation)
                    pose.pose.orientation.x=c*q.x-s*q.y;pose.pose.orientation.y=s*q.x+c*q.y
                    pose.pose.orientation.z=c*q.z+s*q.w;pose.pose.orientation.w=c*q.w-s*q.z;path.poses.append(pose)
                if path.poses:path.header.stamp=path.poses[-1].header.stamp
                result[kind,rid]=path
        return generation,result

    def save_case(self,meta,report,patches):
        directory=self.out/'alignment_cases'/f'{meta["case_number"]:03d}'
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


    def capture_bundle(self,stage):
        return dict(stage=stage,merged=self.merged,layers=dict(self.layers),maps=dict(self.maps),alignment=self.alignment,
            paths={kind:{r:tuple(values) for r,values in group.items()} for kind,group in self.paths.items()},
            sessions=dict(self.sessions),reason=self.reason,fresh=self.fresh(),metrics=deepcopy(self.last_report),generation=self.generation)

    def write_bundle(self,bundle):
        stage=bundle['stage'];directory=self.out/('checkpoint' if stage=='checkpoint' else 'maps');paths={}
        if bundle['merged'] is not None:
            for kind,group in bundle['paths'].items():
                for rid,poses in group.items():
                    transform=bundle['alignment'] if rid else (0.,0.,0.)
                    paths[f'r{rid}_{kind}']=transform_xy([[m.pose.pose.position.x,m.pose.pose.position.y] for m in poses],transform).tolist()
            save_map(directory,'merged_global_map',bundle['merged'],paths)
            for rid,snapshot in bundle['layers'].items():
                if snapshot is not None:save_map(directory,'r0_global_map' if rid==0 else 'r1_aligned_global_map',snapshot)
            atomic_write(directory/'paths.json',json.dumps(paths).encode())
        else:
            for rid,snapshot in bundle['maps'].items():save_map(directory,f'r{rid}_unaligned_global_map',snapshot)
        atomic_write(self.out/('checkpoint_status.json' if stage=='checkpoint' else 'save_status.json'),json.dumps(json_safe(dict(
            stage=stage,merged_saved=bundle['merged'] is not None,alignment=bundle['alignment'],sessions=bundle['sessions'],
            inputs_fresh=bundle['fresh'],reason=bundle['reason'],metrics=bundle['metrics'],generation=bundle['generation'],
            fusion_policy='initial global seeds + conservative local observations; occupied wins',paths_saved=True)),indent=2).encode())
        return bundle['merged'] is not None

    def save_outputs(self,stage):
        future=self.io_worker.submit(self.write_bundle,self.capture_bundle(stage));self.io_jobs.append(future)
        try:return future.result(timeout=90)
        except Exception as exc:self.get_logger().error('Map save failed: '+str(exc));return False

    def save_service(self,request,response):
        response.success=self.save_outputs('requested');response.message=str(self.out);return response

    def reset_service(self,request,response):
        self.invalidate('explicit reset requested');response.success=True;response.message='stationary global-snapshot startup reset';return response

    def close(self):
        if self.closing:return
        self.closing=True;self.search_worker.shutdown(wait=True,cancel_futures=True)
        if self.map_future is not None:
            try:
                merged,layers,_=self.map_future.result(timeout=120)
                if self.map_generation==self.generation:self.merged,self.layers=merged,layers
            except Exception as exc:self.get_logger().error(str(exc))
        if self.alignment is not None and (self.pending_seeds or self.updates):
            try:self.merged,self.layers=self.fusion.update(self.generation,self.pending_seeds,list(self.updates),self.alignment)
            except Exception as exc:self.get_logger().error(str(exc))
        self.map_worker.shutdown(wait=True,cancel_futures=True)
        self.path_worker.shutdown(wait=True,cancel_futures=True)
        self.save_outputs('final');self.io_worker.shutdown(wait=True)


def main(args=None):
    rclpy.init(args=args,signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node=MultiAgentMapping();stopping=False
    def stop(*_):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        while rclpy.ok() and not stopping:rclpy.spin_once(node,timeout_sec=.05)
    finally:
        node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
