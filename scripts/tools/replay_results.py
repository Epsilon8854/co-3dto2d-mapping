#!/usr/bin/env python3
"""Play recorded final maps/poses in RViz without starting mapping nodes."""
from pathlib import Path
import argparse
from copy import deepcopy
from datetime import datetime
import json
import math
import os
import re
import signal
import sys
import time
from zoneinfo import ZoneInfo

import yaml

ROOT = next(p for p in Path(__file__).resolve().parents if (p / 'package.xml').is_file())
sys.path.insert(0, str(ROOT))
from scripts.lib.runtime import children, start, stop_process

STOP = False


def stop(*_):
    global STOP
    STOP = True


def replay_topics(metadata, pipeline, robot_id):
    rows = metadata['rosbag2_bagfile_information']['topics_with_message_count']
    topics = {r['topic_metadata']['name']: r['topic_metadata']['type']
              for r in rows if r['message_count'] > 0}
    if pipeline == 'auto':
        pipeline = 'multi' if '/multi_agent/merged_global_occupancy' in topics else 'single'
    if pipeline == 'single' and robot_id is None:
        ids = sorted(int(m[1]) for name in topics
                     if (m := re.fullmatch(r'/r([01])/mapping/global_occupancy', name)))
        if not ids:
            raise ValueError('No recorded single-agent global map found')
        robot_id = ids[0]
    ids = (0, 1) if pipeline == 'multi' else (robot_id,)
    map_topic = ('/multi_agent/merged_global_occupancy' if pipeline == 'multi'
                 else f'/r{robot_id}/mapping/global_occupancy')
    required = {map_topic: 'nav_msgs/msg/OccupancyGrid'}
    wanted = {'/tf', '/tf_static', '/multi_agent/map_status'}
    for rid in ids:
        odom = (f'/multi_agent/r{rid}/aligned_odometry' if pipeline == 'multi'
                else f'/r{rid}/mapping/occupancy_mapper_odometry')
        required[odom] = 'nav_msgs/msg/Odometry'
        if pipeline == 'multi':
            wanted.update((f'/multi_agent/r{rid}/global_pose', f'/multi_agent/r{rid}/aligned_path'))
        else:
            wanted.add(f'/r{rid}/mapping/debug/local_submap_corrected_odometry_path')
    missing = [name for name, kind in required.items() if topics.get(name) != kind]
    if missing:
        raise ValueError('Final map/pose data missing or wrong type: ' + ', '.join(missing))
    selected = {name: topics[name] for name in (wanted | required.keys()) if name in topics}
    return pipeline, robot_id, selected


def rviz_config(pipeline, robot_id, topics):
    config = yaml.safe_load((ROOT / f'rviz/{pipeline}_agent.rviz').read_text())
    manager = config['Visualization Manager']
    if pipeline == 'single':
        config = yaml.safe_load(yaml.safe_dump(config).replace('r0/', f'r{robot_id}/'))
        manager = config['Visualization Manager']
    frame = 'map' if pipeline == 'multi' else f'r{robot_id}/odom'
    manager['Global Options']['Fixed Frame'] = frame
    manager['Views']['Current']['Target Frame'] = frame
    displays = []
    for source in manager['Displays']:
        item = deepcopy(source)
        kind = item['Class'].rsplit('/', 1)[-1]
        if kind == 'Grid':
            displays.append(item)
            continue
        if kind not in ('Map', 'Odometry', 'Path'):
            continue
        topic = item.get('Topic', {}).get('Value')
        if pipeline == 'single':
            replacements = {
                f'/r{robot_id}/mapping/floor_based_odometry': f'/r{robot_id}/mapping/occupancy_mapper_odometry',
                f'/r{robot_id}/mapping/debug/floor_based_odometry_path': f'/r{robot_id}/mapping/debug/local_submap_corrected_odometry_path',
            }
            topic = replacements.get(topic, topic)
        if topic not in topics or any(d.get('Topic', {}).get('Value') == topic for d in displays):
            continue
        item['Topic'].update(Value=topic, **{'Reliability Policy': 'Reliable'})
        if kind == 'Map':
            item['Topic']['Durability Policy'] = 'Transient Local'
            item['Name'] = 'Final Global Map'
        elif kind == 'Odometry':
            item['Name'] = 'Final Pose ' + (re.search(r'/r[01]/', topic)[0].strip('/'))
        else:
            item['Name'] = 'Final Path ' + (re.search(r'/r[01]/', topic)[0].strip('/'))
        item.update(Enabled=True, Value=True)
        displays.append(item)
    manager['Displays'] = displays
    return config


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bag', type=Path, required=True, help='capture directory or its bag/ directory')
    p.add_argument('--pipeline', choices=['auto', 'single', 'multi'], default='auto')
    p.add_argument('--robot-id', type=int, choices=[0, 1])
    p.add_argument('--domain-id', type=int, default=73)
    p.add_argument('--rate', type=float, default=1.0)
    p.add_argument('--loop', action='store_true')
    p.add_argument('--no-rviz', action='store_true')
    p.add_argument('--output-root', type=Path)
    args = p.parse_args()
    if not 0 <= args.domain_id <= 232 or not math.isfinite(args.rate) or args.rate <= 0:
        p.error('Invalid domain or replay rate')
    bag = args.bag.expanduser().resolve()
    if not (bag / 'metadata.yaml').is_file() and (bag / 'bag/metadata.yaml').is_file():
        bag = bag / 'bag'
    if not (bag / 'metadata.yaml').is_file():
        p.error('metadata.yaml not found in capture/bag directory')
    metadata = yaml.safe_load((bag / 'metadata.yaml').read_text())
    try:
        pipeline, robot_id, topics = replay_topics(metadata, args.pipeline, args.robot_id)
    except ValueError as exc:
        p.error(str(exc))
    run = (args.output_root or ROOT / 'outputs' / f'{pipeline}_agent') / 'replays' / datetime.now(
        ZoneInfo('Asia/Seoul')).strftime('%Y%m%d_%H%M%S')
    run.mkdir(parents=True, exist_ok=False)
    config_path = run / 'results.rviz'
    config_path.write_text(yaml.safe_dump(rviz_config(pipeline, robot_id, topics), sort_keys=False))
    qos = {name: dict(reliability='reliable', durability='transient_local' if
           kind == 'nav_msgs/msg/OccupancyGrid' or name == '/tf_static' else 'volatile',
           history='keep_last', depth=100 if kind == 'tf2_msgs/msg/TFMessage' else 10)
           for name, kind in topics.items()}
    qos_path = run / 'playback_qos.yaml'
    qos_path.write_text(yaml.safe_dump(qos, sort_keys=False))
    (run / 'replay.json').write_text(json.dumps(dict(bag=str(bag), pipeline=pipeline,
        topics=topics, rate=args.rate, domain_id=args.domain_id, recomputation=False), indent=2))
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY='1')
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop)
    print(f'Result playback: {bag}\nRViz frame: {"map" if pipeline == "multi" else f"r{robot_id}/odom"}'
          f' | local domain {args.domain_id} | no driver/ICP/mapping nodes', flush=True)
    cycle = 0
    while not STOP:
        cycle += 1
        viewer = player = None
        try:
            if not args.no_rviz:
                viewer = start(['rviz2', '-d', config_path, '--ros-args', '-p', 'use_sim_time:=true'],
                               run / f'rviz_{cycle:03d}.log', 'rviz')
            player = start(['ros2', 'bag', 'play', str(bag), '--clock', '--rate', str(args.rate),
                            '--delay', '2', '--qos-profile-overrides-path', str(qos_path),
                            '--topics', *sorted(topics)], run / f'player_{cycle:03d}.log', 'player')
            while not STOP and player.poll() is None:
                if viewer is not None and viewer.poll() is not None:
                    raise RuntimeError('RViz exited; see playback logs')
                time.sleep(.1)
            if not STOP and player.returncode:
                raise RuntimeError('Bag player failed; see playback logs')
            if not args.loop and not STOP and viewer is not None:
                print('Playback complete. Final map remains visible; Ctrl+C to close.', flush=True)
                while not STOP and viewer.poll() is None:
                    time.sleep(.1)
        finally:
            if player is not None:
                stop_process(player)
            if viewer is not None:
                stop_process(viewer)
            children.clear()
        if not args.loop:
            break


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError) as exc:
        raise SystemExit(str(exc))
