#!/usr/bin/env python3
"""Owned process groups, snapshots, and one clock for Experiment 4."""
import argparse
from datetime import datetime
from zoneinfo import ZoneInfo
import hashlib
import json
import math
import os
import platform
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

import yaml
from livox_driver_guard import conflicting_drivers

ROOT=Path(__file__).resolve().parents[1]
CONFIG=ROOT/'config/experiments/experiment_04_multi_agent_mapping'
stop_requested=False
children=[]


def args_parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=['run','replay','record'])
    p.add_argument('--role',choices=['robot','fusion','all'],default='robot',help='all = this robot + fusion')
    p.add_argument('--robot-id',type=int,choices=[0,1],default=0)
    p.add_argument('--scope',choices=['robot','central','combined'],default='robot',help='recording scope')
    p.add_argument('--start-driver',action='store_true',help='start one local Livox driver (live only)')
    p.add_argument('--driver-config',type=Path)
    p.add_argument('--livox-workspace',type=Path,help='built Livox workspace; otherwise auto-detect')
    p.add_argument('--robot-config',type=Path)
    p.add_argument('--alignment-config',type=Path,default=CONFIG/'alignment.yaml')
    p.add_argument('--odometry-config',type=Path,default=ROOT/'config/lidar_odometry.yaml')
    p.add_argument('--bag',type=Path,help='combined raw rosbag, one shared timeline')
    p.add_argument('--loop',action='store_true',help='restart every single + fusion node per pass')
    p.add_argument('--rate',type=float,default=1.)
    p.add_argument('--domain-id',type=int,default=int(os.environ.get('ROS_DOMAIN_ID','72')))
    p.add_argument('--run-id',default=datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d_%H%M%S_%f'))
    p.add_argument('--no-rviz',action='store_true')
    p.add_argument('--status-only',action='store_true',help='show live session/alignment status without starting or stopping mapping nodes')
    p.add_argument('--dry-run',action='store_true',help='show resolved roles/configs only')
    return p


def shutdown_signal(*_):
    global stop_requested
    stop_requested=True


def console(label,message):
    stamp=datetime.now(ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S')
    print(f'[{stamp}] [{label}] {message}',flush=True)


class StatusConsole:
    """Small diagnostic subscriptions only; never subscribe to raw sensors/maps."""
    def __init__(self,args):
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
        self.rclpy=rclpy;self.context=Context()
        rclpy.init(context=self.context,signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
        self.node=rclpy.create_node(f'experiment04_console_{os.getpid()}',context=self.context)
        self.executor=SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.label='fusion' if args.role=='fusion' else f'r{args.robot_id}'
        params=yaml.safe_load(args.alignment_config.read_text())['/**']['ros__parameters']
        self.required_windows=params.get('confirmation_windows',1)
        self.latest=None;self.received=0.;self.last_print=0.;self.signature=None;self.sessions={}
        self.relay_status={}
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(DiagnosticArray,'/multi_agent/alignment_status',self.status,qos)
        self.node.create_subscription(DiagnosticArray,'/multi_agent/pose_relay_status',lambda msg:setattr(self,'relay_status',self.values(msg)),qos)
        ids=(0,1) if args.action=='replay' or args.role in ('fusion','all') else (args.robot_id,)
        for rid in ids:
            self.node.create_subscription(DiagnosticArray,f'/multi_agent/r{rid}/session',
                                          lambda msg,r=rid:self.session(msg,r),qos)
        console(self.label,'정합 상태 수신 대기 / waiting for fusion status (Ctrl+C: 종료)')

    @staticmethod
    def values(msg):
        result={}
        for item in msg.status:
            for value in item.values:
                try:result[value.key]=json.loads(value.value)
                except (ValueError,TypeError):result[value.key]=value.value
        return result

    def session(self,msg,rid):
        values=self.values(msg);identifier=values.get('session_id')
        if identifier and self.sessions.get(rid)!=identifier:
            restarted=rid in self.sessions;self.sessions[rid]=identifier
            console(self.label,f'SESSION r{rid}: {identifier} / '+('변경됨; 초기 정합 재확인' if restarted else '수신 시작'))

    def status(self,msg):
        self.latest=self.values(msg);self.received=time.monotonic()

    def poll(self):
        for _ in range(5):self.executor.spin_once(timeout_sec=0.)
        now=time.monotonic()
        if self.latest is None or now-self.received>5.:
            signature=('NO_FUSION_STATUS',)
            text=('중앙 정합 상태 수신 대기; r0 fusion 실행·ROS domain·네트워크 확인'
                  if self.latest is None else f'중앙 상태 수신 공백 {now-self.received:.1f}s; 정합 완료 상태를 신뢰하지 마세요')
        else:
            value=self.latest;state=value.get('state','UNKNOWN');reason=value.get('reason','')
            count=value.get('consistent_windows',0);total=value.get('required_windows',self.required_windows)
            ready=value.get('ready',False)
            signature=(state,reason,count,ready)
            labels={'WAIT_INPUT':'정합 입력 대기','COLLECTING':'초기 정지 관측 수집 중',
                    'SEARCHING':'XY/yaw 정합 검색 중','VERIFYING':'정합 반복 검증 중',
                    'WAIT_GLOBAL_MAP':'정합 변환 확정; 병합용 global map 수신 대기','SEEDING_MAP':'초기 지도 구성 중',
                    'LOCKED':'초기 정합 완료·변환 고정','STALE':'입력 수신 지연'}
            text=f'{state}: {labels.get(state,state)} | 검증 {count}/{total} | ready={str(ready).lower()}'
            transform=value.get('transform')
            if ready and isinstance(transform,list) and len(transform)==3:
                text+=f' | r0←r1 x={transform[0]:.3f}m y={transform[1]:.3f}m yaw={math.degrees(transform[2]):.2f}deg'
            else:text+=' | '+str(reason)
            if 'local_map_subscriptions_active' in value:
                text+=' | local-map 구독='+('ON' if value['local_map_subscriptions_active'] else 'OFF')
            if 'global_map_subscriptions_active' in value:
                text+=' global-map 구독='+('ON' if value['global_map_subscriptions_active'] else 'OFF')
            frames=value.get('window_frames',{});minimum=value.get('window_min_frames',15)
            if not ready and value.get('local_map_subscriptions_active') and frames:
                text+=' | 현재 창 '+', '.join(f'r{rid}={count}/{minimum} maps' for rid,count in frames.items())
                drops=[]
                for rid,counts in value.get('local_input_stats',{}).items():
                    selected={k:v for k,v in counts.items() if k not in ('received','accepted') and v}
                    drops.append(f'r{rid}:수신={counts.get("received",0)},채택={counts.get("accepted",0)},제외={selected}')
                if drops:text+=' | '+'; '.join(drops)
            runtime=value.get('metrics',{}).get('central_tick_runtime_ms')
            spans=value.get('snapshot_spans_sec',{})
            if value.get('global_map_subscriptions_active') and spans:
                text+=' | 누적 지도 경과 '+', '.join(f'r{rid}={span:.1f}s' for rid,span in spans.items())
            if isinstance(runtime,(float,int)):text+=f' | 중앙 처리={runtime:.0f}ms'
            if value.get('world_frame'):
                text+=f' | world={value["world_frame"]} map_revision={value.get("map_revision",0)}'
            temporal=value.get('temporal_fusion',{})
            if temporal:
                text+=' | 중앙 동적 필터='+('ON' if temporal.get('enabled') else 'OFF')
                if temporal.get('enabled'):
                    common=temporal.get('common',{})
                    text+=f' 삭제누적={common.get("cleared_cells",0)}cells 과거관측제외={common.get("stale_cell_observations",0)}cells'
            relay_ms=self.relay_status.get('callback_p95_ms')
            if isinstance(relay_ms,(float,int)):
                ages=self.relay_status.get('receipt_ages_sec',{})
                text+=f' | pose 중계 p95={relay_ms:.2f}ms 수신공백={ages}'
        if signature!=self.signature or now-self.last_print>=5.:
            console(self.label,text);self.signature=signature;self.last_print=now

    def close(self):
        self.executor.remove_node(self.node);self.executor.shutdown()
        self.node.destroy_node();self.rclpy.shutdown(context=self.context)


def start(command,log,kind):
    logfile=log.open('wb')
    env=dict(os.environ,LIVOX_LAUNCHER_PID=str(os.getpid())) if kind=='driver' else None
    try:proc=subprocess.Popen(list(map(str,command)),start_new_session=True,stdout=logfile,stderr=subprocess.STDOUT,env=env)
    finally:logfile.close()
    children.append((proc,kind))
    console('START',f'{kind} pid={proc.pid} | log={log}')
    return proc


def stop_process(proc,timeout=30):
    # The launch leader may already have exited while its children remain.
    # Always address our original owned process group, not a name-wide pkill.
    def group_alive():
        proc.poll()  # Reap our child; waiting for it alone misses grandchildren.
        try:os.killpg(proc.pid,0);return True
        except ProcessLookupError:return False
    for sig,grace in ((signal.SIGINT,timeout),(signal.SIGTERM,10),(signal.SIGKILL,5)):
        if not group_alive():return
        console('STOP',f'owned process group {proc.pid}: {sig.name}, waiting up to {grace}s for all children')
        try:os.killpg(proc.pid,sig)
        except ProcessLookupError:return
        end=time.monotonic()+grace
        while group_alive() and time.monotonic()<end:time.sleep(.1)
    if group_alive():console('STOP',f'process group {proc.pid} remains after SIGKILL; inspect OS process state')


def cleanup():
    # Finish producers first. Fusion remains alive to drain their final maps.
    for proc,kind in children:
        if kind in ('driver','player','robot','recorder'):stop_process(proc,timeout=5 if kind=='driver' else 30)
    if any(kind=='fusion' and proc.poll() is None for proc,kind in children):time.sleep(2)
    for proc,kind in children:
        if kind=='fusion':stop_process(proc,timeout=90)
    for proc,kind in children:
        if kind=='rviz':stop_process(proc)
    children.clear()


def graph_preflight(args):
    if args.start_driver:
        config=args.driver_config or os.environ.get('LIVOX_CONFIG')
        found=conflicting_drivers(config)
        if found:raise RuntimeError('local Livox driver process already running (even if no topics yet): '+json.dumps(found))
    import rclpy
    rclpy.init();node=rclpy.create_node('experiment04_preflight')
    try:
        until=time.monotonic()+2
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.1)
        def exists(topic):return bool(node.count_publishers(topic))
        if args.action=='replay' and exists('/clock'):raise RuntimeError('another /clock publisher is active in this domain')
        if args.action=='replay':
            for rid in (0,1):
                if exists(f'/r{rid}/livox/lidar') or exists(f'/r{rid}/mapping/global_occupancy'):
                    raise RuntimeError(f'r{rid} live/replay publishers already active; choose another domain')
        if args.action=='run':
            if args.role in ('robot','all') and exists(f'/r{args.robot_id}/mapping/global_occupancy'):
                raise RuntimeError('this robot already has a mapper')
            if args.start_driver and any(exists(f'/r{args.robot_id}/livox/{suffix}') for suffix in ('lidar','imu')):
                raise RuntimeError('sensor publisher exists; do not start a duplicate driver')
        if args.role in ('fusion','all') or args.action=='replay':
            if exists('/multi_agent/alignment_ready'):raise RuntimeError('fusion host already active')
    finally:node.destroy_node();rclpy.shutdown()


def launch(role,rid,config,alignment,odometry,output,sim=False,static=True):
    return ['ros2','launch','co_3dto2d_mapping','experiment_04_multi_agent.launch.py',
        'role:='+role,'robot_id:='+str(rid),'robot_config_file:='+str(config),
        'alignment_config_file:='+str(alignment),'odometry_config_file:='+str(odometry),
        'output_directory:='+str(output),'use_sim_time:='+str(sim).lower(),
        'publish_sensor_static_tf:='+str(static).lower(),'rviz:=false']


def topics(args):
    ids=(0,1) if args.scope in ('central','combined') else (args.robot_id,)
    result=['/tf','/tf_static']
    for rid in ids:
        prefix=f'/r{rid}'
        if args.scope!='central':result += [prefix+'/livox/lidar',prefix+'/livox/imu']
        result += [prefix+'/odom',prefix+'/mapping/floor_based_odometry',prefix+'/mapping/occupancy_mapper_odometry',
                   prefix+'/mapping/local_occupancy',prefix+'/mapping/global_occupancy',prefix+'/mapping/pipeline_diagnostics',
                   f'/multi_agent/r{rid}/session']
    if args.scope in ('central','combined'):
        result += ['/multi_agent/'+suffix for suffix in ('r1_to_r0_transform','map_anchor','map_status','alignment_ready','alignment_status',
            'pose_relay_status',
            'r1/aligned_global_occupancy','merged_global_occupancy','r0/aligned_odometry','r1/aligned_odometry',
            'r0/aligned_path','r1/aligned_path','r0/raw_path','r1/raw_path')]
    return result


def main():
    args=args_parser().parse_args()
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*',args.run_id):raise ValueError('invalid run id')
    if not 0<=args.domain_id<=232:raise ValueError('invalid ROS_DOMAIN_ID')
    if not args.rate>0:raise ValueError('rate must be positive')
    if args.start_driver and (args.action!='run' or args.role=='fusion'):raise ValueError('--start-driver requires robot role')
    if args.action=='replay' and (args.bag is None or not (args.bag/'metadata.yaml').is_file()):raise ValueError('--bag must contain metadata.yaml')
    if args.dry_run:
        print(json.dumps(dict(action=args.action,role=args.role,robot_id=args.robot_id,domain=args.domain_id,
            alignment_config=str(args.alignment_config),topics=topics(args) if args.action=='record' else []),indent=2));return
    os.environ['ROS_DOMAIN_ID']=str(args.domain_id);os.environ.setdefault('ROS_LOCALHOST_ONLY','0')
    if args.status_only:
        for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,shutdown_signal)
        monitor=StatusConsole(args)
        try:
            while not stop_requested:monitor.poll();time.sleep(.1)
        finally:monitor.close()
        return
    if args.start_driver:
        candidates=([args.livox_workspace/'install/local_setup.bash'] if args.livox_workspace else [])
        if os.environ.get('LIVOX_SETUP'):candidates.append(Path(os.environ['LIVOX_SETUP']))
        candidates += [p/'install/local_setup.bash' for p in
            (ROOT/'third_party/ws_livox',ROOT.parent/'ws_livox',Path.home()/'ws_livox',Path.home()/'livox_ws')]
        found=next((p for p in candidates if p.is_file()),None)
        if found is not None:os.environ['LIVOX_SETUP']=str(found.resolve())
        elif subprocess.run(['ros2','pkg','prefix','livox_ros_driver2'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode:
            raise ValueError('Livox workspace not found; pass --livox-workspace or set LIVOX_SETUP')
    console('START',f'Experiment 4 {args.action} role={args.role} robot=r{args.robot_id} domain={args.domain_id}; 시작 전 topic 확인')
    graph_preflight(args)
    category='captures' if args.action=='record' else 'runs'
    run=ROOT/'outputs/experiment_04_multi_agent_mapping'/category/args.run_id
    run.mkdir(parents=True,exist_ok=False);(run/'logs').mkdir();(run/'configs').mkdir()
    # Always snapshot effective entrypoint YAMLs; never refer to mutable files
    # later in an already-running comparison.
    configs={}
    for name,source in [('alignment.yaml',args.alignment_config),('lidar_odometry.yaml',args.odometry_config)]+[
        (f'robot{rid}.yaml',args.robot_config if args.robot_config and rid==args.robot_id else CONFIG/f'robot{rid}.yaml') for rid in (0,1)]:
        data=source.read_bytes();target=run/'configs'/name;target.write_bytes(data);configs[name]=target.resolve()
    meta=dict(action=args.action,robot_id=args.robot_id,domain_id=args.domain_id,
        host=platform.node(),ros_distribution=os.environ.get('ROS_DISTRO',''),
        rmw_implementation=os.environ.get('RMW_IMPLEMENTATION','default'),
        recorded_topics=topics(args) if args.action=='record' else [],
        timestamp_policy='original source headers preserved; replay uses one clock',
        started_at=datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),arguments={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},
        source_config_sha256={name:hashlib.sha256(p.read_bytes()).hexdigest() for name,p in configs.items()},
        branch=subprocess.check_output(['git','branch','--show-current'],cwd=ROOT,text=True).strip(),
        commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip())
    (run/'run.yaml').write_text(yaml.safe_dump(meta,sort_keys=False))
    driver_config=args.driver_config or (Path(os.environ['LIVOX_CONFIG']) if os.environ.get('LIVOX_CONFIG') else None)
    if driver_config is not None and driver_config.is_file():
        (run/'configs/driver_config.json').write_bytes(driver_config.read_bytes())
    (run/'working_tree.patch').write_bytes(subprocess.check_output(['git','diff','--binary'],cwd=ROOT))
    # Preserve changed and untracked relevant source too; diff alone omits new modules.
    source_hashes={}
    for folder in ('co_3dto2d_mapping','launch','config','scripts','include','src','rviz'):
        for path in (ROOT/folder).rglob('*'):
            if not path.is_file() or '__pycache__' in path.parts:continue
            data=path.read_bytes();relative=path.relative_to(ROOT)
            target=run/'source_snapshot'/relative;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
            source_hashes[str(relative)]=hashlib.sha256(data).hexdigest()
    (run/'source_sha256.json').write_text(json.dumps(source_hashes,indent=2))
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,shutdown_signal)
    print('Experiment 4 output:',run,flush=True)
    monitor=None
    try:
        if args.action=='record':
            recorder=start(['ros2','bag','record','--storage','sqlite3','--output',run/'bag',*topics(args)],run/'logs/record.log','recorder')
            while not stop_requested and recorder.poll() is None:time.sleep(.2)
            return
        cycle=0
        while not stop_requested:
            cycle+=1;output=run/f'cycle_{cycle:03d}';output.mkdir()
            monitor=StatusConsole(args)
            if args.action=='run' and args.start_driver:
                env_config=str(args.driver_config.resolve()) if args.driver_config else os.environ.get('LIVOX_CONFIG','')
                if env_config:os.environ['LIVOX_CONFIG']=env_config
                start(['bash',ROOT/'scripts/run_livox_robot.sh',str(args.robot_id+1)],run/'logs/driver.log','driver')
            ids=(0,1) if args.action=='replay' else (args.robot_id,) if args.role in ('robot','all') else ()
            for rid in ids:start(launch('robot',rid,configs[f'robot{rid}.yaml'],configs['alignment.yaml'],configs['lidar_odometry.yaml'],output,
                    sim=args.action=='replay',static=args.action!='replay'),output/f'robot{rid}.log','robot')
            if args.action=='replay' or args.role in ('fusion','all'):
                start(launch('fusion',0,configs['robot0.yaml'],configs['alignment.yaml'],configs['lidar_odometry.yaml'],output,
                    sim=args.action=='replay'),output/'fusion.log','fusion')
            if not args.no_rviz and (args.action=='replay' or args.role in ('fusion','all')):
                start(['rviz2','-d',ROOT/'rviz/experiment_04_multi_agent_mapping.rviz','--ros-args',
                       '-p','use_sim_time:='+str(args.action=='replay').lower()],output/'rviz.log','rviz')
            if args.action=='replay':
                import rosbag2_py
                reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(args.bag.resolve()),storage_id=''),rosbag2_py.ConverterOptions('',''))
                names={t.name for t in reader.get_all_topics_and_types()}
                required={f'/r{rid}/livox/{kind}' for rid in (0,1) for kind in ('lidar','imu')}
                if not required<=names:raise ValueError('combined raw bag missing: '+str(required-names))
                if '/tf_static' not in names:raise ValueError('combined bag requires namespaced sensor /tf_static')
                time.sleep(4)
                player=start(['ros2','bag','play','--clock','--rate',str(args.rate),str(args.bag.resolve()),'--topics',
                    *sorted(required),'/tf_static'],output/'bag_play.log','player')
                while not stop_requested and player.poll() is None:
                    monitor.poll()
                    for proc,kind in children:
                        if kind in ('robot','fusion') and proc.poll() is not None:raise RuntimeError(kind+' exited; see logs')
                    time.sleep(.2)
                if player.poll() not in (None,0) and not stop_requested:raise RuntimeError('bag playback failed')
                time.sleep(2)
                monitor.close();monitor=None
                cleanup()
                if not args.loop:break
            else:
                while not stop_requested:
                    monitor.poll()
                    for proc,kind in children:
                        if kind!='rviz' and proc.poll() is not None:raise RuntimeError(kind+' exited; see logs')
                    time.sleep(.2)
    finally:
        # Console teardown must not skip owned process cleanup if ROS shutdown
        # itself fails while a driver is still running.
        try:
            if monitor is not None:monitor.close()
        finally:
            cleanup()
            meta['finished_at']=datetime.now(ZoneInfo('Asia/Seoul')).isoformat()
            (run/'run.yaml').write_text(yaml.safe_dump(meta,sort_keys=False))

if __name__=='__main__':
    try:main()
    except (ValueError,RuntimeError,OSError) as exc:
        print('error:',exc,file=sys.stderr);sys.exit(1)
