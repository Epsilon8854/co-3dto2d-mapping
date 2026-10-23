#!/usr/bin/env python3
"""Combine real r0/r1 raw bags by recording time, preserving source stamps.

No time shifting, sensor relabeling, or pretend second trajectory is performed.
Use genuinely namespaced robot bags with a synchronized clock basis.
"""

from pathlib import Path as _WorkspacePath
import sys as _workspace_sys
_workspace_root = next(p for p in _WorkspacePath(__file__).resolve().parents if (p/"package.xml").is_file() and (p/"CMakeLists.txt").is_file())
_workspace_sys.path.insert(0, str(_workspace_root))
import argparse
import heapq
import json
from pathlib import Path
import rosbag2_py


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag0',type=Path,required=True);parser.add_argument('--bag1',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():parser.error('output already exists')
    readers=[];topic_types={}
    for rid,path in enumerate((args.bag0,args.bag1)):
        reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(path.resolve()),storage_id=''),rosbag2_py.ConverterOptions('',''))
        types={t.name:t.type for t in reader.get_all_topics_and_types()}
        required={f'/r{rid}/livox/lidar',f'/r{rid}/livox/imu','/tf_static'}
        if not required<=types.keys():parser.error(f'bag{rid} missing {required-types.keys()}')
        for name in required:
            if name in topic_types and topic_types[name]!=types[name]:parser.error('topic type collision')
            topic_types[name]=types[name]
        readers.append((reader,required))
    writer=rosbag2_py.SequentialWriter();writer.open(rosbag2_py.StorageOptions(uri=str(args.output.resolve()),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    for name,type_name in topic_types.items():
        # Preserve static TF late-join behavior; raw sensors remain volatile.
        qos='- history: 1\n  depth: 1\n  reliability: 1\n  durability: 1\n' if name=='/tf_static' else ''
        writer.create_topic(rosbag2_py.TopicMetadata(name=name,type=type_name,serialization_format='cdr',offered_qos_profiles=qos))
    def next_valid(rid):
        reader,allowed=readers[rid]
        while reader.has_next():
            name,data,ns=reader.read_next()
            if name in allowed:return ns,rid,name,data
        return None
    queue=[]
    for rid in (0,1):
        item=next_valid(rid)
        if item:heapq.heappush(queue,item)
    counts={};ranges={}
    while queue:
        ns,rid,name,data=heapq.heappop(queue)
        writer.write(name,data,ns);counts[name]=counts.get(name,0)+1
        if name!='/tf_static':
            if rid not in ranges:ranges[rid]=[ns,ns]
            ranges[rid][1]=ns
        item=next_valid(rid)
        if item:heapq.heappush(queue,item)
    del writer
    (args.output/'source_manifest.json').write_text(json.dumps(dict(bag0=str(args.bag0.resolve()),bag1=str(args.bag1.resolve()),
        counts=counts,time_ranges_ns=ranges,timestamp_policy='original recording and header stamps preserved'),indent=2))
    if len(ranges)!=2 or min(v[1] for v in ranges.values())<=max(v[0] for v in ranges.values()):
        raise RuntimeError('bags have no overlapping recording time; output not suitable for live-synchronous validation')
    print(args.output)
if __name__=='__main__':main()
