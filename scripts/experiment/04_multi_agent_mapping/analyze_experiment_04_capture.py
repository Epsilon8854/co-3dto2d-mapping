#!/usr/bin/env python3
"""Deterministic fixed-trajectory experiment-4 ablations, without DDS."""

from pathlib import Path as _WorkspacePath
import sys as _workspace_sys
_workspace_root = next(p for p in _WorkspacePath(__file__).resolve().parents if (p/"package.xml").is_file() and (p/"CMakeLists.txt").is_file())
_workspace_sys.path.insert(0, str(_workspace_root))
import argparse
from collections import deque
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np
import yaml

ROOT=next(p for p in Path(__file__).resolve().parents if (p/'package.xml').is_file() and (p/'CMakeLists.txt').is_file())
sys.path.insert(0,str(ROOT))
from co_3dto2d_mapping.multi_agent.bag import read_capture,fusion_inputs
from co_3dto2d_mapping.multi_agent.core import IncrementalFusion,save_map,json_safe,transform_xy
from co_3dto2d_mapping.multi_agent.temporal import TemporalFusionConfig
from co_3dto2d_mapping.multi_agent.periodic import PeriodicConfig,PeriodicFusion,register_periodic,patch_metrics
from co_3dto2d_mapping.common.planar_transform_utils import compose_planar,invert_planar
from co_3dto2d_mapping.common.se2_map_registration import RegistrationConfig


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('capture',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--periodic',action='store_true');p.add_argument('--probe',action='store_true')
    p.add_argument('--free-count',type=int,default=30);p.add_argument('--clear-range',type=float,default=6.)
    p.add_argument('--gain',type=float,default=.5);p.add_argument('--interval',type=float,default=2.)
    p.add_argument('--filter-off',action='store_true');args=p.parse_args()
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    c=read_capture(args.capture);seeds,updates=fusion_inputs(c)
    temporal=TemporalFusionConfig(not args.filter_off,args.free_count,5.,.1,args.clear_range).validated()
    pc=PeriodicConfig(enabled=args.periodic,correction_gain=args.gain,interval_sec=args.interval).validated()
    reg=RegistrationConfig(max_free_conflict_ratio=.18).validated()
    engine=PeriodicFusion(.05,16000000,temporal,'map',pc,reg,out/'observations') if pc.enabled else IncrementalFusion(.05,16000000,temporal,'map')
    align=c['alignment'];t0=updates[0][1].stamp_ns;times=[];probes=[];series=[];last_export=t0;last_probe=t0
    source_recent=deque();probe_ref=None
    if args.probe:
        from co_3dto2d_mapping.multi_agent.temporal import TemporalOccupancyAccumulator
        from co_3dto2d_mapping.multi_agent.core import compose_grids
        from co_3dto2d_mapping.multi_agent.periodic import sensor_center
        probe_ref=TemporalOccupancyAccumulator(.05,temporal,'r0/mapping_odom')
        probe_ref.seed(compose_grids([(seeds[0],(0.,0.,0.))],.05,frame='r0/mapping_odom'))
    paths={};source_manifest={}
    for f in [*(ROOT/'co_3dto2d_mapping').rglob('*.py'),Path(__file__).resolve()]:
        content=f.read_bytes();relative=f.relative_to(ROOT)
        target=out/'source_snapshot'/relative;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(content)
        source_manifest[str(relative)]=hashlib.sha256(content).hexdigest()
    (out/'config.json').write_text(json.dumps(json_safe(dict(capture=str(args.capture.resolve()),
        temporal=asdict(temporal),periodic=asdict(pc),registration=asdict(reg),initial_alignment=align,
        anchor=c['anchor'],source_order='original header timestamp, robot id; 0.5s source-time batches',
        initial_seed_stamps={r:s.stamp_ns for r,s in seeds.items()},source_sha256=source_manifest)),indent=2))
    (out/'bag_audit.json').write_text(json.dumps(json_safe(c['topic_audit']),indent=2))
    began=time.perf_counter();idx=0;batch_index=0
    while idx<len(updates):
        end=idx+1;limit=updates[idx][1].stamp_ns+500_000_000
        while end<len(updates) and updates[end][1].stamp_ns<limit:end+=1
        batch=updates[idx:end];t=batch[-1][1].stamp_ns
        started=time.perf_counter()
        merged,layers=engine.update(0,seeds if idx==0 else {},batch,align,c['anchor'])
        elapsed=(time.perf_counter()-started)*1000;times.append(elapsed)
        if pc.enabled:align=engine.alignment
        if probe_ref is not None:
            for r,s in batch:
                if r==0:probe_ref.observe(compose_grids([(s,(0.,0.,0.))],.05,frame=s.frame),sensor_center(s))
                else:source_recent.append(s)
            while source_recent and t-source_recent[0].stamp_ns>3e9:source_recent.popleft()
            if t-last_probe>=args.interval*1e9 and len(source_recent)>=10:
                report,patches=register_periodic(probe_ref.snapshot(),list(source_recent),c['alignment'],pc,reg)
                revised=compose_planar(invert_planar(report['reference_anchor']),
                    compose_planar(align,report['source_anchor']))
                report['current_transform_metrics_on_fixed_support']=patch_metrics(*patches,revised,reg)
                report['elapsed_sec']=(t-t0)/1e9;probes.append(report);last_probe=t
        if t-last_export>=10e9 or end==len(updates):
            for r in (0,1):
                pts=[pose[:2] for _,ns,pose in c['poses'][r] if ns<=t]
                paths[f'r{r}']=transform_xy(pts,compose_planar(c['anchor'],align) if r else c['anchor']).tolist()
            save_map(out/'timeline',f'{int((t-t0)/1e9):03d}',merged,paths)
            last_export=t
        series.append(dict(elapsed_sec=(t-t0)/1e9,alignment=list(align),worker_ms=elapsed,
                           occupied_cells=int(np.count_nonzero(merged.grid>50))))
        if batch_index%20==0:print(f'{out.name}: {end}/{len(updates)}, t={(t-t0)/1e9:.1f}s, worker={elapsed:.1f}ms, C yaw={np.degrees(align[2]):.3f}',flush=True)
        idx=end;batch_index+=1
    save_map(out,'merged_global_map',merged,paths)
    for r,s in layers.items():
        if s is not None:save_map(out,f'r{r}_layer',s)
    if isinstance(engine,PeriodicFusion):common=engine.engine.common;reports=engine.reports
    else:common=engine.common;reports=[]
    if common is not None:
        np.savez_compressed(out/'temporal_evidence.npz',grid=common.grid,last_observed_ns=common.last_observed_ns,
            free_counts=common.free_counts,bin_occupied=common.bin_occupied,origin=common.origin,resolution=common.resolution)
    (out/'registration.json').write_text(json.dumps(json_safe(reports),indent=2))
    (out/'probes.json').write_text(json.dumps(json_safe(probes),indent=2))
    (out/'series.json').write_text(json.dumps(json_safe(series)))
    summary=dict(duration_source_sec=(updates[-1][1].stamp_ns-t0)/1e9,updates=len(updates),batches=len(times),
        wall_sec=time.perf_counter()-began,worker_p50_ms=float(np.percentile(times,50)),worker_p95_ms=float(np.percentile(times,95)),
        worker_max_ms=float(max(times)),initial_alignment=c['alignment'],final_alignment=align,
        final_occupied_cells=int(np.count_nonzero(merged.grid>50)),diagnostics=engine.diagnostics(),
        registration_attempts=len(reports),registration_accepted=sum(r['accepted'] for r in reports),
        probe_reasons=dict(__import__('collections').Counter(r['reason'] for r in probes)))
    (out/'summary.json').write_text(json.dumps(json_safe(summary),indent=2));print(json.dumps(json_safe(summary),indent=2),flush=True)

if __name__=='__main__':main()
