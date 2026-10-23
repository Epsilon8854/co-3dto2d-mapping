#!/usr/bin/env python3
"""Attribute retained occupied cells to observed temporal evidence (not GT)."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from co_3dto2d_mapping.multi_agent_bag import read_capture,fusion_inputs
from co_3dto2d_mapping.multi_agent_core import compose_grids,json_safe,transform_xy
from co_3dto2d_mapping.multi_agent_periodic import sensor_center
from co_3dto2d_mapping.multi_agent_temporal import TemporalFusionConfig,TemporalOccupancyAccumulator
from co_3dto2d_mapping.planar_transform_utils import compose_planar


def sample(array,origin,res,xy,fill):
    ij=np.floor((xy-origin)/res+1e-6).astype(int);x,y=ij.T
    valid=(x>=0)&(y>=0)&(x<array.shape[1])&(y<array.shape[0])
    a=np.full(len(x),fill,dtype=array.dtype);a[valid]=array[y[valid],x[valid]];return a


def main():
    p=argparse.ArgumentParser();p.add_argument('capture',type=Path);p.add_argument('--baseline',type=Path,required=True)
    args=p.parse_args();out=args.baseline
    with np.load(out/'merged_global_map_data.npz') as z:
        grid=z['data'];res=float(z['resolution']);origin=z['origin'][:2];rows,cols=np.nonzero(grid>50)
        xy=origin+np.column_stack((cols+.5,rows+.5))*res
    c=read_capture(args.capture);seeds,updates=fusion_inputs(c)
    cfg=TemporalFusionConfig(True,30,5.,.1,6.)
    acc=TemporalOccupancyAccumulator(res,cfg,'map');last={r:s.stamp_ns for r,s in seeds.items()}
    for r,s in seeds.items():
        t=c['anchor'] if r==0 else compose_planar(c['anchor'],c['alignment'])
        acc.seed(compose_grids([(s,t)],res,frame='map'))
    stats={key:np.zeros(len(xy),np.int32) for key in ('occupied_observations','free_observations','range_blocked',
        'new_bin_free_votes','occupied_streak_resets','timeout_streak_resets','same_bin_occupied_blocks','max_free_streak',
        'r0_occupied_observations','r1_occupied_observations','r0_free_observations','r1_free_observations',
        'cleared_transitions','reoccupied_after_clear')}
    last_occ=np.full(len(xy),-1,np.int64);last_free=last_occ.copy()
    for r,s in updates:
        if s.stamp_ns<=last[r]:continue
        last[r]=s.stamp_ns;t=c['anchor'] if r==0 else compose_planar(c['anchor'],c['alignment'])
        projected=compose_grids([(s,t)],res,frame='map');g=projected.geometry
        v=sample(projected.grid,np.array([g.origin_x,g.origin_y]),res,xy,-1)
        old_counts=sample(acc.free_counts,acc.origin,res,xy,0)
        old_last=sample(acc.last_observed_ns,acc.origin,res,xy,-1)
        old_occupied=sample(acc.bin_occupied,acc.origin,res,xy,False)
        old_grid=sample(acc.grid,acc.origin,res,xy,-1)
        center=transform_xy([sensor_center(s)],t)[0]
        free=(v>=0)&(v<=50);occ=v>50
        blocked=free&(old_grid>50)&(np.linalg.norm(xy-center,axis=1)>cfg.clear_max_range_m)
        fresh=(s.stamp_ns//acc.bin_ns>old_last//acc.bin_ns)|(old_last<0)
        stale=(old_last>=0)&(s.stamp_ns//acc.bin_ns<old_last//acc.bin_ns)
        eligible=~stale&~blocked
        stats['occupied_observations']+=occ;stats['free_observations']+=free
        stats[f'r{r}_occupied_observations']+=occ;stats[f'r{r}_free_observations']+=free
        stats['range_blocked']+=blocked
        stats['new_bin_free_votes']+=free&eligible&fresh
        stats['same_bin_occupied_blocks']+=free&eligible&~fresh&old_occupied
        stats['occupied_streak_resets']+=occ&eligible&(old_counts>0)
        stats['timeout_streak_resets']+=(v>=0)&eligible&fresh&(old_last>=0)&(s.stamp_ns-old_last>acc.timeout_ns)&(old_counts>0)
        last_occ[occ&eligible]=s.stamp_ns;last_free[free&eligible]=s.stamp_ns
        acc.observe(projected,center)
        new_grid=sample(acc.grid,acc.origin,res,xy,-1)
        stats['reoccupied_after_clear']+=(old_grid<=50)&(new_grid>50)&(stats['cleared_transitions']>0)
        stats['cleared_transitions']+=(old_grid>50)&(new_grid==0)
        new_counts=sample(acc.free_counts,acc.origin,res,xy,0).astype(np.int32)
        stats['max_free_streak']=np.maximum(stats['max_free_streak'],new_counts)
    final_counts=sample(acc.free_counts,acc.origin,res,xy,0)
    actual=sample(acc.grid,acc.origin,res,xy,-1)
    np.savez_compressed(out/'retained_cell_diagnostics.npz',xy=xy,rows=rows,cols=cols,
        final_free_streak=final_counts,last_occupied_ns=last_occ,last_free_ns=last_free,**stats)
    summary=dict(remaining_occupied_cells=len(xy),replay_agrees_on_remaining=int(np.count_nonzero(actual>50)),
        no_free_observations=int(np.count_nonzero(stats['free_observations']==0)),
        any_range_blocked=int(np.count_nonzero(stats['range_blocked']>0)),
        any_occupied_streak_reset=int(np.count_nonzero(stats['occupied_streak_resets']>0)),
        any_timeout_streak_reset=int(np.count_nonzero(stats['timeout_streak_resets']>0)),
        any_same_bin_occupied_block=int(np.count_nonzero(stats['same_bin_occupied_blocks']>0)),
        final_streak_histogram={str(k):int(np.count_nonzero(final_counts==k)) for k in range(30)},
        free_observed_but_never_reached_threshold=int(np.count_nonzero((stats['free_observations']>0)&(stats['max_free_streak']<30))),
        note='Overlapping categories across all retained cells, including real static walls; not a dynamic-object GT label.')
    (out/'retained_cell_summary.json').write_text(json.dumps(json_safe(summary),indent=2));print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
