"""Read experiment-4 central captures without starting DDS or a ROS executor."""
from collections import Counter, defaultdict
from pathlib import Path
import sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from co_3dto2d_mapping.multi_agent_ros import snapshot_of, stamp_ns, pose_of, yaw_of, values_of


def read_capture(capture):
    capture = Path(capture).resolve()
    bag = capture / 'bag' if (capture / 'bag').is_dir() else capture
    result = dict(capture=str(capture), local={0: [], 1: []}, global_maps={0: [], 1: []},
                  poses={0: [], 1: []}, recorded_merged=[], diagnostics=[], anchor=None,
                  alignment=None, lock_receipt_ns=None, topic_audit={})
    result['pose_messages']={0: [], 1: []};result['sessions']={}
    samples = defaultdict(list)
    frames = defaultdict(Counter)
    # sqlite chunks have non-overlapping receipt intervals in this recorder.
    for path in sorted(bag.glob('*.db3')):
        db = sqlite3.connect(f'file:{path}?mode=ro', uri=True)
        topics = {i: (name, typ) for i, name, typ in db.execute('SELECT id,name,type FROM topics')}
        classes = {}
        for topic_id, receipt, data in db.execute('SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp,id'):
            name, typ = topics[topic_id]
            # Do not deserialize multi-MB point clouds just to inspect stamps.
            if typ == 'sensor_msgs/msg/PointCloud2':
                # CDR header: little-endian int32 sec,uint32 nanosec after encapsulation.
                import struct
                endian = '<' if data[1] & 1 else '>'
                sec, ns = struct.unpack_from(endian+'iI', data, 4)
                samples[name].append((receipt, sec*10**9+ns))
                continue
            if typ not in ('nav_msgs/msg/OccupancyGrid', 'nav_msgs/msg/Odometry',
                           'geometry_msgs/msg/TransformStamped', 'diagnostic_msgs/msg/DiagnosticArray'):
                samples[name].append((receipt, receipt)); continue
            if typ not in classes: classes[typ] = get_message(typ)
            msg = deserialize_message(data, classes[typ])
            ns = stamp_ns(msg.header.stamp)
            samples[name].append((receipt, ns)); frames[name][msg.header.frame_id] += 1
            if name == '/multi_agent/map_anchor':
                t = msg.transform
                result['anchor'] = (t.translation.x, t.translation.y, yaw_of(t.rotation))
            elif name == '/multi_agent/r1_to_r0_transform':
                t = msg.transform
                if result['alignment'] is None:
                    result['alignment'] = (t.translation.x, t.translation.y, yaw_of(t.rotation))
                    result['lock_receipt_ns'] = receipt
            elif name == '/multi_agent/merged_global_occupancy':
                if msg.info.width: result['recorded_merged'].append((receipt, snapshot_of(msg)))
            elif name in ('/multi_agent/alignment_status', '/multi_agent/map_status'):
                result['diagnostics'].append((receipt, name, values_of(msg)))
            elif name in ('/multi_agent/r0/session','/multi_agent/r1/session'):
                result['sessions'].setdefault(int(name.split('/')[2][1:]),msg)
            for rid in (0, 1):
                if name == f'/r{rid}/mapping/local_occupancy' and msg.info.width:
                    result['local'][rid].append((receipt, snapshot_of(msg)))
                elif name == f'/r{rid}/mapping/global_occupancy' and msg.info.width:
                    result['global_maps'][rid].append((receipt, snapshot_of(msg)))
                elif name == f'/r{rid}/mapping/occupancy_mapper_odometry':
                    result['poses'][rid].append((receipt, ns, pose_of(msg)))
                    result['pose_messages'][rid].append(msg)
        db.close()
    for name, entries in samples.items():
        a = np.asarray(entries, dtype=np.int64); delta = np.diff(a[:, 1])/1e9
        lag = (a[:, 0]-a[:, 1])/1e9
        result['topic_audit'][name] = dict(count=len(a), frames=dict(frames[name]),
            source_first_ns=int(a[0, 1]), source_last_ns=int(a[-1, 1]),
            duplicate_stamps=int(np.count_nonzero(delta == 0)), backwards_stamps=int(np.count_nonzero(delta < 0)),
            source_interval_p50_sec=float(np.median(delta)) if len(delta) else None,
            source_interval_max_sec=float(delta.max()) if len(delta) else None,
            receipt_minus_source_p50_sec=float(np.median(lag)), receipt_minus_source_p95_sec=float(np.percentile(lag, 95)))
    result['stamp_pairing']={}
    for rid in (0,1):
        cloud={ns for _,ns in samples[f'/r{rid}/mapping/plane_height_filtered']}
        pose={ns for _,ns,_ in result['poses'][rid]}
        local={s.stamp_ns for _,s in result['local'][rid]}
        result['stamp_pairing'][rid]=dict(cloud_stamps=len(cloud),pose_stamps=len(pose),local_stamps=len(local),
            cloud_without_exact_mapping_pose=len(cloud-pose),local_without_exact_mapping_pose=len(local-pose))
    if result['alignment'] is None or result['anchor'] is None:
        raise ValueError('capture has no initial alignment/world anchor')
    return result


def fusion_inputs(capture):
    """Freeze the same initial transform and pre-lock seed across all ablations.

    Receipt-time selection reconstructs the bootstrap; local updates are then
    ordered by original source timestamp (robot ID is the deterministic tie).
    This is an offline counterfactual, not an assertion of exact live batching.
    """
    seeds = {}
    updates = []
    for rid in (0, 1):
        available = [(t, s) for t, s in capture['global_maps'][rid] if t <= capture['lock_receipt_ns']]
        if not available: raise ValueError(f'r{rid} has no pre-lock global snapshot')
        seeds[rid] = available[-1][1]
        updates.extend((rid, s) for _, s in capture['local'][rid] if s.stamp_ns > seeds[rid].stamp_ns)
    return seeds, sorted(updates, key=lambda v: (v[1].stamp_ns, v[0]))
