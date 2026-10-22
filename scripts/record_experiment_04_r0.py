#!/usr/bin/env python3
"""Record mapper inputs/poses on r0; optionally include raw LiDAR/IMU."""
import argparse
from datetime import datetime
from zoneinfo import ZoneInfo
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import signal
import sqlite3
import subprocess
import time
import yaml

ROOT=Path(__file__).resolve().parents[1]
CONFIG=ROOT/'config/experiments/experiment_04_multi_agent_mapping'
STOP=False


def now():return datetime.now(ZoneInfo('Asia/Seoul')).isoformat()


def selected_topics(profile,with_maps,include_raw):
    result=[]
    for scope in ('per_robot','shared'):
        for group,entries in profile[scope].items():
            if group=='optional_maps' and not with_maps:continue
            if group=='optional_raw' and not include_raw:continue
            for robot in profile['robots'] if scope=='per_robot' else ('',):
                for entry in entries:
                    item=dict(entry);item['topic']=item['topic'].format(robot=robot);item['group']=group
                    result.append(item)
    names=[item['topic'] for item in result]
    if len(names)!=len(set(names)):raise ValueError('Duplicate topic in recording profile')
    if any(not name.startswith('/') for name in names):raise ValueError('Topics must be absolute')
    return result


def qos_for(item):
    return dict(history='keep_last',depth=item.get('depth',100),
        reliability='reliable' if item.get('reliable') else 'best_effort',
        durability='transient_local' if item.get('latched') else 'volatile')


def graph_snapshot(node,items):
    result={}
    for item in items:
        result[item['topic']]=[dict(node_name=p.node_name,node_namespace=p.node_namespace,
            topic_type=p.topic_type,qos=dict(depth=p.qos_profile.depth,
                reliability=p.qos_profile.reliability.name,durability=p.qos_profile.durability.name))
            for p in node.get_publishers_info_by_topic(item['topic'])]
    return result


def snapshot_parameters(node):
    # Read actual running nodes; local YAML copies are only reference configs.
    import rclpy
    from rcl_interfaces.srv import ListParameters,GetParameters
    from rclpy.parameter import parameter_value_to_python
    result={};budget=time.monotonic()+12.
    targets=sorted(set((ns.rstrip('/')+'/'+name) for name,ns in node.get_node_names_and_namespaces()
        if ns in ('/r0','/r1') or ns.startswith(('/r0/','/r1/')) or
        name in ('multi_agent_mapping','multi_agent_pose_relay','livox_lidar_r0','livox_lidar_r1')))
    def await_result(future):
        end=min(time.monotonic()+1.5,budget)
        while not future.done() and time.monotonic()<end and not STOP:
            rclpy.spin_once(node,timeout_sec=.05)
        if not future.done():raise TimeoutError('parameter service timeout')
        return future.result()
    for target in targets:
        if STOP:break
        if time.monotonic()>=budget:
            result[target]=dict(status='unavailable',reason='snapshot time budget exceeded');continue
        clients=[]
        try:
            listing=node.create_client(ListParameters,target+'/list_parameters');clients.append(listing)
            getting=node.create_client(GetParameters,target+'/get_parameters');clients.append(getting)
            if not listing.wait_for_service(timeout_sec=.2) or not getting.wait_for_service(timeout_sec=.2):
                result[target]=dict(status='services unavailable');continue
            names=await_result(listing.call_async(ListParameters.Request())).result.names
            request=GetParameters.Request();request.names=names
            values=await_result(getting.call_async(request)).values
            result[target]=dict(status='captured',parameters={k:parameter_value_to_python(v) for k,v in zip(names,values)})
        except Exception as exc:result[target]=dict(status='unavailable',reason=str(exc))
        finally:
            for client in clients:node.destroy_client(client)
    return dict(captured_at=now(),nodes=result,note='Remote Git/source revision is not inferred from local repository.')


def audit_bag(bag,items,frames):
    """Read SQLite message indexes and static TF only; do not load cloud blobs."""
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage
    from std_msgs.msg import Bool
    metadata=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    stats={};edges=set();alignment_was_ready=False
    for relative in metadata['relative_file_paths']:
        path=(bag/relative).resolve()
        if not path.is_relative_to(bag.resolve()):raise ValueError('Bag file outside capture directory')
        with sqlite3.connect(path.as_uri()+'?mode=ro',uri=True) as db:
            for name,kind,count,first,last in db.execute(
                'SELECT t.name,t.type,COUNT(*),MIN(m.timestamp),MAX(m.timestamp) '
                'FROM messages m JOIN topics t ON t.id=m.topic_id GROUP BY t.name,t.type'):
                row=stats.setdefault(name,dict(type=kind,count=0,first_receipt_ns=first,last_receipt_ns=last))
                row['count']+=count;row['first_receipt_ns']=min(first,row['first_receipt_ns']);row['last_receipt_ns']=max(last,row['last_receipt_ns'])
            for (data,) in db.execute("SELECT m.data FROM messages m JOIN topics t ON t.id=m.topic_id WHERE t.name='/tf_static'"):
                for tf in deserialize_message(data,TFMessage).transforms:
                    edges.add((tf.header.frame_id,tf.child_frame_id))
            for (data,) in db.execute("SELECT m.data FROM messages m JOIN topics t ON t.id=m.topic_id WHERE t.name='/multi_agent/alignment_ready'"):
                alignment_was_ready=alignment_was_ready or deserialize_message(data,Bool).data
    for row in stats.values():
        span=(row['last_receipt_ns']-row['first_receipt_ns'])/1e9
        row['receipt_hz']=(row['count']-1)/span if span>0 else None
    def connected(a,b):
        seen={a};pending=[a]
        while pending:
            current=pending.pop()
            for parent,child in edges:
                other=child if parent==current else parent if child==current else None
                if other is not None and other not in seen:seen.add(other);pending.append(other)
        return b in seen
    missing=[i['topic'] for i in items if not stats.get(i['topic'],{}).get('count')]
    wrong=[i['topic'] for i in items if i['topic'] in stats and stats[i['topic']]['type']!=i['type']]
    absent_frames=[pair for pair in frames if not connected(*pair)]
    incomplete=lambda group:any(i['topic'] in missing+wrong for i in items if i['group']==group)
    def ranges_overlap(groups):
        ranges=[stats[i['topic']] for i in items if i['group'] in groups and i['type']!='tf2_msgs/msg/TFMessage' and i['topic'] in stats]
        return bool(ranges) and min(r['last_receipt_ns'] for r in ranges)>max(r['first_receipt_ns'] for r in ranges)
    overlap=ranges_overlap(('mapping_required','pose_required'))
    raw_requested=any(i['group']=='optional_raw' for i in items)
    static_present='/tf_static' in stats and '/tf_static' not in wrong and not absent_frames
    mapping_complete=not incomplete('mapping_required') and not incomplete('pose_required') and static_present and overlap
    return dict(topics=stats,missing_topics=missing,type_mismatches=wrong,static_tf_edges=sorted(edges),
        missing_sensor_static_paths=absent_frames,mapping_receipt_ranges_overlap=overlap,
        filtered_mapping_minimum_complete=mapping_complete,
        multi_agent_minimum_complete=mapping_complete and not incomplete('alignment_required') and alignment_was_ready,
        alignment_was_ready=alignment_was_ready,
        raw_inputs_requested=raw_requested,
        raw_replay_minimum_complete=raw_requested and not incomplete('optional_raw') and static_present and ranges_overlap(('optional_raw',)),
        fixed_pose_reference_minimum_complete=not incomplete('pose_required'),
        alignment_reference_available=stats.get('/multi_agent/r1_to_r0_transform',{}).get('count',0)>0,
        merged_map_reference_available=stats.get('/multi_agent/merged_global_occupancy',{}).get('count',0)>0,
        note='Minimum presence/type/TF checks only. Receipt rate does not prove loss-free sensor delivery or clock synchronization.')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--name',default='multi_agent_walk_dynamic',help='scenario prefix; timestamp is appended')
    p.add_argument('--domain-id',type=int,default=72)
    p.add_argument('--profile',type=Path,default=CONFIG/'recording_r0.yaml')
    p.add_argument('--with-agent-maps',action='store_true',help='also record both agents local/global grids; adds remote traffic')
    p.add_argument('--include-raw',action='store_true',help='add both raw LiDAR/IMU streams for full ICP/filter reprocessing')
    p.add_argument('--cache-mb',type=int,default=128)
    p.add_argument('--split-seconds',type=int,default=300)
    p.add_argument('--dry-run',action='store_true',help='print topics without ROS discovery or recording')
    args=p.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*',args.name):p.error('invalid scenario name')
    if not 0<=args.domain_id<=232 or args.cache_mb<1 or args.split_seconds<0:p.error('invalid domain/cache/split value')
    profile=yaml.safe_load(args.profile.read_text());items=selected_topics(profile,args.with_agent_maps,args.include_raw)
    if args.dry_run:
        print(yaml.safe_dump(dict(domain_id=args.domain_id,topics=items),sort_keys=False));return
    os.environ['ROS_DOMAIN_ID']=str(args.domain_id);os.environ['ROS_LOCALHOST_ONLY']='0'
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from std_msgs.msg import Bool
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('experiment04_r0_capture');alignment={'ready':False}
    node.create_subscription(Bool,'/multi_agent/alignment_ready',lambda msg:alignment.update(ready=msg.data),
        QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    capture=ROOT/'outputs/experiment_04_multi_agent_mapping/captures'/(
        args.name+'_'+datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d_%H%M%S_%f'))
    capture.mkdir(parents=True,exist_ok=False);(capture/'configs').mkdir();(capture/'logs').mkdir()
    shutil.copy2(args.profile,capture/'configs/recording_profile.yaml')
    for source in sorted(CONFIG.glob('*.yaml'))+[ROOT/'config/lidar_odometry.yaml']:
        shutil.copy2(source,capture/'configs'/('repository_'+source.name))
    qos_path=capture/'configs/record_qos.yaml'
    qos_path.write_text(yaml.safe_dump({i['topic']:qos_for(i) for i in items},sort_keys=False))
    def git(*cmd):return subprocess.check_output(['git',*cmd],cwd=ROOT,text=True).strip()
    from rclpy.utilities import get_rmw_implementation_identifier
    meta=dict(experiment='experiment_04_multi_agent_mapping',capture_role='r0_both_robots',
        purpose='multi-agent temporal dynamic-object clearing and merged-map evaluation',
        started_at=now(),host=platform.node(),domain_id=args.domain_id,branch=git('branch','--show-current'),commit=git('rev-parse','HEAD'),
        ros_distribution=os.environ.get('ROS_DISTRO',''),rmw_implementation=get_rmw_implementation_identifier(),
        arguments={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},
        topics=items,raw_inputs_included=args.include_raw,bag_uri='bag',profile_sha256=hashlib.sha256(args.profile.read_bytes()).hexdigest(),
        timestamp_policy='recording receipt time on r0; original message headers preserved; live /clock is not recorded',
        config_note='repository_* YAMLs are references; runtime_parameters.yaml is a best-effort snapshot of actual services')
    (capture/'working_tree.patch').write_text(git('diff','--binary'))
    (capture/'capture.yaml').write_text(yaml.safe_dump(meta,sort_keys=False))
    command=['ros2','bag','record','--storage','sqlite3','--output',str(capture/'bag'),
        '--max-cache-size',str(args.cache_mb*1024*1024),'--max-bag-duration',str(args.split_seconds),
        '--qos-profile-overrides-path',str(qos_path),*[i['topic'] for i in items]]
    (capture/'record_command.json').write_text(json.dumps(command,indent=2))
    print('Capture:',capture,flush=True)
    print('Recorder starts now and discovers topics as both robots start. Ctrl+C stops only this recorder.',flush=True)
    print('Default: mapper-input filtered clouds and recorded poses. Raw LiDAR/IMU: '+str(args.include_raw),flush=True)
    print('Remote clouds add network traffic. Start moving after r0 LOCKED and required publishers are visible.',flush=True)
    global STOP
    def stop(*_):
        global STOP
        STOP=True
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,stop)
    proc=None;log=(capture/'logs/record.log').open('wb');snapshot_done=False;last_check=0.;graph={};error=None
    try:
        proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        while not STOP:
            if proc.poll() is not None:raise RuntimeError(f'recorder exited ({proc.returncode}); see {capture}/logs/record.log')
            rclpy.spin_once(node,timeout_sec=.1)
            if time.monotonic()-last_check<5.:continue
            last_check=time.monotonic();graph=graph_snapshot(node,items)
            missing=[i['topic'] for i in items if i['group'] in ('mapping_required','pose_required','alignment_required','optional_raw') and
                     not any(v['topic_type']==i['type'] for v in graph[i['topic']])]
            print(now(),'alignment_ready='+str(alignment['ready']),
                'WAIT required publishers: '+', '.join(missing) if missing else 'Required publishers visible; actual message counts checked on stop.',flush=True)
            if not missing and not snapshot_done:
                print('Saving actual node parameters (best effort, time budget 12s); recorder continues.',flush=True)
                (capture/'runtime_parameters.yaml').write_text(yaml.safe_dump(snapshot_parameters(node),sort_keys=False))
                snapshot_done=True
    except Exception as exc:error=str(exc);print('Capture error:',error,flush=True)
    finally:
        if proc is not None and proc.poll() is None:
            try:os.killpg(proc.pid,signal.SIGINT)
            except ProcessLookupError:pass
            # Graceful rosbag close writes metadata/indexes. Do not kill mapping.
            try:proc.wait(timeout=120.)
            except subprocess.TimeoutExpired:
                error=error or 'Recorder close timed out; inspect bag before use'
                try:os.killpg(proc.pid,signal.SIGTERM)
                except ProcessLookupError:pass
                try:proc.wait(timeout=10.)
                except subprocess.TimeoutExpired:
                    try:os.killpg(proc.pid,signal.SIGKILL)
                    except ProcessLookupError:pass
                    proc.wait(timeout=10.)
        log.close()
        if proc is not None and proc.returncode not in (0,130,-signal.SIGINT):
            error=error or f'Recorder return code {proc.returncode}; inspect record.log'
        meta.update(finished_at=now(),recorder_returncode=proc.returncode if proc else None,error=error,
            graph_at_last_check=graph,alignment_ready_at_stop=alignment['ready'])
        if not snapshot_done:
            (capture/'runtime_parameters.yaml').write_text(yaml.safe_dump(dict(status='not captured',reason='required publishers never all visible before stop')))
        try:
            audit=audit_bag(capture/'bag',items,profile['sensor_static_frames'])
            (capture/'capture_audit.yaml').write_text(yaml.safe_dump(audit,sort_keys=False))
            print('Filtered mapping minimum:',audit['filtered_mapping_minimum_complete'],
                '| multi-agent minimum:',audit['multi_agent_minimum_complete'],flush=True)
            if args.include_raw:print('Full raw replay minimum:',audit['raw_replay_minimum_complete'],flush=True)
            if audit['missing_topics']:print('Missing topics:',', '.join(audit['missing_topics']),flush=True)
            if not audit['filtered_mapping_minimum_complete']:error=error or 'Required mapper input/pose/static TF check failed'
            if not audit['multi_agent_minimum_complete']:error=error or 'Multi-agent alignment reference/ready check failed; inspect audit'
            if args.include_raw and not audit['raw_replay_minimum_complete']:error=error or 'Requested raw input check failed'
        except Exception as exc:error=error or 'Bag audit failed: '+str(exc)
        meta['error']=error
        (capture/'capture.yaml').write_text(yaml.safe_dump(meta,sort_keys=False))
        print('Saved:',capture,flush=True)
        print('Bag URI:',capture/'bag',flush=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    if error:raise SystemExit(error)


if __name__=='__main__':main()
