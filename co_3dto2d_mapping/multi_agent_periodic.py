"""Local-prior inter-agent registration and source-time map reprojection.

Only C = r0/mapping_odom <- r1/mapping_odom changes. The world anchor and
both agents' native poses remain untouched. A worker commits a new C only
after all retained original observations have been replayed under that C.
"""
from collections import deque
from dataclasses import dataclass, asdict, replace
from pathlib import Path
import math
import time
import numpy as np
from scipy.spatial import cKDTree
from co_3dto2d_mapping.multi_agent_core import (
    Snapshot, IncrementalFusion, OccupancyAccumulator, compose_grids, transform_xy,
)
from co_3dto2d_mapping.multi_agent_temporal import TemporalOccupancyAccumulator
from co_3dto2d_mapping.occupancy_submap import GridGeometry, extract_local_patch, sample_points
from co_3dto2d_mapping.planar_transform_utils import compose_planar, invert_planar, normalize_angle
from co_3dto2d_mapping.se2_map_registration import (
    RegistrationConfig, _trimmed_icp, _direction_metrics, _free_conflict_ratio, _grid_values,
)
from co_3dto2d_mapping.multi_agent_alignment import _observability


@dataclass(frozen=True)
class PeriodicConfig:
    enabled: bool = False
    interval_sec: float = 2.0
    source_window_sec: float = 3.0
    radius_m: float = 8.0
    resolution_m: float = .10
    max_input_age_sec: float = 5.0
    max_translation_m: float = .30
    max_yaw_deg: float = 3.0
    correction_gain: float = .5
    min_relative_improvement: float = .02
    min_observability: float = .005
    journal_max_mb: float = 2048.0
    max_reprojection_sec: float = 4.0

    def validated(self):
        if type(self.enabled) is not bool: raise ValueError('periodic enabled must be boolean')
        for key in ('interval_sec','source_window_sec','radius_m','resolution_m',
                    'max_input_age_sec','max_translation_m','max_yaw_deg','journal_max_mb','max_reprojection_sec'):
            if not math.isfinite(getattr(self,key)) or getattr(self,key)<=0: raise ValueError('invalid periodic '+key)
        if not 0<self.correction_gain<=1 or not 0<=self.min_relative_improvement<1 or not 0<=self.min_observability<1:
            raise ValueError('invalid periodic gain/confidence')
        return self


def sensor_center(snapshot):
    g=snapshot.geometry
    return transform_xy([[g.width*g.resolution/2,g.height*g.resolution/2]],
                        (g.origin_x,g.origin_y,g.origin_yaw))[0]


def patch_metrics(target,source,transform,reg):
    if min(len(target.boundary_points),len(source.boundary_points))<reg.min_correspondences:
        return dict(cost=math.inf,rmse_m=math.inf,overlap=0.,correspondences=0,free_conflict=1.)
    f,fo,fc=_direction_metrics(target.boundary_points,source.boundary_points,transform,reg)
    r,ro,rc=_direction_metrics(source.boundary_points,target.boundary_points,invert_planar(transform),reg)
    d=np.concatenate((f,r)); rmse=float(np.sqrt(np.mean(d*d))) if d.size else math.inf
    # Include all boundary distances: selecting only already-close inliers can
    # make a wrong partial match appear better than a correct larger overlap.
    tp=sample_points(target.boundary_points,reg.icp_max_points)
    sp=sample_points(source.boundary_points,reg.icp_max_points)
    fd=cKDTree(tp).query(transform_xy(sp,transform))[0]
    rd=cKDTree(sp).query(transform_xy(tp,invert_planar(transform)))[0]
    cost=float(np.mean(np.minimum(np.r_[fd,rd],reg.distance_clip_m)**2))
    return dict(cost=cost,rmse_m=rmse,overlap=min(fo,ro),correspondences=min(fc,rc),
                free_conflict=_free_conflict_ratio(target,source,transform,reg))


def register_periodic(reference,recent,alignment,config,reg):
    started=time.perf_counter(); center=sensor_center(recent[-1])
    predicted=transform_xy([center],alignment)[0]
    reference_anchor=(*map(float,predicted),0.)
    source_anchor=(*map(float,center),0.)
    # Occupied wins only within this short window; no r1 evidence enters the
    # historical reference used to score r1 (avoids matching against itself).
    acc=OccupancyAccumulator(config.resolution_m,frame=recent[-1].frame)
    for s in recent:
        acc.add(compose_grids([(s,(0.,0.,0.))],config.resolution_m,frame=s.frame))
    source=acc.snapshot()
    target=extract_local_patch(reference.grid,reference.geometry,reference_anchor,config.radius_m,
                               output_resolution=config.resolution_m)
    source=extract_local_patch(source.grid,source.geometry,source_anchor,config.radius_m,
                               output_resolution=config.resolution_m)
    prior=(0.,0.,alignment[2])
    # Score only boundaries inside the OTHER patch's observed support under
    # the fixed prior. A historical target contains whole walls that the
    # current sensor cannot see; those are not failed correspondences. Freeze
    # these masks before optimizing so a candidate cannot game its own mask.
    target=replace(target,boundary_points=target.boundary_points[
        _grid_values(source,transform_xy(target.boundary_points,invert_planar(prior)))>=0])
    source=replace(source,boundary_points=source.boundary_points[
        _grid_values(target,transform_xy(source.boundary_points,prior))>=0])
    before=patch_metrics(target,source,prior,reg)
    report=dict(accepted=False,reason='insufficient_boundary_points',stamp_ns=recent[-1].stamp_ns,
                before=before,source_frames=len(recent),reference_stamp_ns=reference.stamp_ns,
                alignment_before=list(alignment),reference_anchor=reference_anchor,source_anchor=source_anchor)
    if before['correspondences']<reg.min_correspondences:
        report['runtime_ms']=(time.perf_counter()-started)*1000;return report,(target,source)
    candidates=[]
    for dy in (-config.max_yaw_deg,0.,config.max_yaw_deg):
        pose=_trimmed_icp(target.boundary_points,source.boundary_points,
                          (0.,0.,alignment[2]+math.radians(dy)),reg)
        metrics=patch_metrics(target,source,pose,reg)
        candidates.append((metrics['cost'],pose,metrics))
    _,pose,after=min(candidates,key=lambda v:(v[0],v[1]))
    delta_yaw=normalize_angle(pose[2]-prior[2]); delta_xy=math.hypot(*pose[:2])
    observable=_observability(target,source,pose,reg)
    reason='accepted'
    if delta_xy>config.max_translation_m or abs(delta_yaw)>math.radians(config.max_yaw_deg):reason='correction_limit'
    elif after['correspondences']<reg.min_correspondences:reason='insufficient_correspondences'
    elif after['overlap']<reg.min_symmetric_overlap:reason='low_overlap'
    elif after['rmse_m']>reg.max_symmetric_rmse_m:reason='high_rmse'
    elif after['free_conflict']>reg.max_free_conflict_ratio:reason='free_space_conflict'
    elif observable<config.min_observability:reason='unobservable_geometry'
    elif after['cost']>=before['cost']*(1-config.min_relative_improvement):reason='insufficient_improvement'
    damped=(pose[0]*config.correction_gain,pose[1]*config.correction_gain,
            normalize_angle(prior[2]+config.correction_gain*delta_yaw))
    applied=patch_metrics(target,source,damped,reg)
    if reason=='accepted':
        # The full fit must exceed the confidence threshold above. Damping
        # deliberately reduces that improvement; require its actual step to
        # decrease the same loss, rather than asking half a step to meet the
        # full-step threshold again.
        if applied['cost']>=before['cost']:reason='insufficient_damped_improvement'
        elif applied['overlap']<reg.min_symmetric_overlap:reason='damped_low_overlap'
        elif applied['rmse_m']>reg.max_symmetric_rmse_m:reason='damped_high_rmse'
        elif applied['free_conflict']>reg.max_free_conflict_ratio:reason='damped_free_space_conflict'
    candidate=compose_planar(reference_anchor,compose_planar(damped,invert_planar(source_anchor)))
    report.update(accepted=reason=='accepted',reason=reason,after=after,applied=applied,
        patch_transform=pose,applied_patch_transform=damped,alignment_candidate=list(candidate),
        correction_at_sensor_m=delta_xy,correction_yaw_deg=math.degrees(delta_yaw),observability=observable,
        runtime_ms=(time.perf_counter()-started)*1000)
    return report,(target,source)


class ObservationJournal:
    """Lossless native grids and source timestamps; disk usage is bounded.

    No Python history of dense arrays grows with run length. Existing journals
    are never reused implicitly. Exhaustion disables further correction rather
    than silently rebuilding a map with a truncated observation history.
    """
    def __init__(self,directory,max_mb):
        self.directory=Path(directory);self.directory.mkdir(parents=True,exist_ok=False)
        self.max_bytes=int(max_mb*1024**2);self.bytes=0;self.count=0

    def append(self,seeds,updates):
        entries=[(True,r,s) for r,s in sorted(seeds.items())]+[(False,r,s) for r,s in updates]
        arrays={};meta=[]
        for i,(seed,r,s) in enumerate(entries):
            arrays[f'g{i}']=s.grid;g=s.geometry
            meta.append([seed,r,g.resolution,g.width,g.height,g.origin_x,g.origin_y,g.origin_yaw,s.stamp_ns,s.frame])
        arrays['meta']=np.asarray(__import__('json').dumps(meta))
        from io import BytesIO
        b=BytesIO();np.savez_compressed(b,**arrays);data=b.getvalue()
        if self.bytes+len(data)>self.max_bytes:raise OSError('periodic observation journal capacity exceeded')
        from co_3dto2d_mapping.multi_agent_core import atomic_write
        atomic_write(self.directory/f'{self.count:08d}.npz',data)
        self.bytes+=len(data);self.count+=1

    def batches(self):
        import json
        for i in range(self.count):
            with np.load(self.directory/f'{i:08d}.npz',allow_pickle=False) as z:
                seeds={};updates=[]
                for k,row in enumerate(json.loads(str(z['meta']))):
                    seed,r,res,w,h,x,y,yaw,ns,frame=row
                    s=Snapshot(z[f'g{k}'],GridGeometry(res,w,h,x,y,yaw),ns,frame)
                    if seed:seeds[r]=s
                    else:updates.append((r,s))
            yield seeds,updates


class PeriodicFusion:
    """Single map-worker owner; ROS and offline replay use the same code."""
    def __init__(self,resolution,max_cells,temporal_config,frame,config,registration,journal_directory):
        self.args=(resolution,max_cells,temporal_config,frame)
        self.config=config.validated();self.registration=registration.validated()
        self.engine=IncrementalFusion(*self.args)
        self.journal=ObservationJournal(journal_directory,self.config.journal_max_mb) if config.enabled else None
        self.reference=None;self.recent=deque(maxlen=256);self.last_attempt_ns=None;self.last_sources={}
        self.reference_bucket=None;self.reference_versions=deque(maxlen=16)
        self.interval_ns=round(self.config.interval_sec*1e9)
        self.generation=None;self.alignment=None;self.revision=0;self.reports=[];self.failure=None
        self.last_reprojection_ms=0.;self.peak_reprojection_ms=0.;self.last_patches=None
        self.reprojection_backoff_until_ns=0

    def diagnostics(self):
        d=self.engine.diagnostics()
        d['periodic']=dict(config=asdict(self.config),revision=self.revision,
            attempts=len(self.reports),accepted=sum(r['accepted'] for r in self.reports),
            latest=self.reports[-1] if self.reports else None,disabled_reason=self.failure,
            journal_bytes=self.journal.bytes if self.journal else 0,
            reprojection_ms=self.last_reprojection_ms,peak_reprojection_ms=self.peak_reprojection_ms,
            reprojection_backoff_until_ns=self.reprojection_backoff_until_ns,
            alignment=self.alignment)
        return d

    def update(self,generation,seeds,updates,alignment,world_from_reference=(0.,0.,0.)):
        if self.generation is not None and generation!=self.generation:
            raise ValueError('PeriodicFusion must be recreated for a new session generation')
        self.generation=generation
        if self.alignment is None:self.alignment=tuple(alignment)
        if tuple(alignment)!=self.alignment:raise ValueError('alignment revision not committed by caller')
        merged,layers=self.engine.update(generation,seeds,updates,self.alignment,world_from_reference)
        if not self.config.enabled or self.failure:return merged,layers
        try:self.journal.append(seeds,updates)
        except OSError as exc:self.failure=str(exc);return merged,layers
        # Independent r0 reference in its original frame; no r1/self evidence.
        if self.reference is None:
            cfg=replace(self.args[2],enabled=True)
            self.reference=TemporalOccupancyAccumulator(self.args[0],cfg,'r0/mapping_odom',self.args[1])
        for r,s in seeds.items():
            if r==0 and np.any(s.grid>=0):
                self.reference.seed(compose_grids([(s,(0.,0.,0.))],self.args[0],frame=s.frame))
                if self.reference_bucket is None:self.reference_bucket=s.stamp_ns//self.interval_ns
        for r,s in sorted(updates,key=lambda v:(v[1].stamp_ns,v[0])):
            if s.stamp_ns<=self.last_sources.get(r,-1):continue
            self.last_sources[r]=s.stamp_ns
            if not np.any(s.grid>=0):continue
            if r==0:
                bucket=s.stamp_ns//self.interval_ns
                if self.reference_bucket is not None and bucket>self.reference_bucket:
                    frozen=self.reference.snapshot()
                    # Snapshots are immutable. Registration uses r0 history
                    # strictly before the SAME absolute source-time boundary
                    # used for r1, independent of map-worker callback phase.
                    for old in range(max(self.reference_bucket+1,bucket-15),bucket+1):
                        if frozen is not None:self.reference_versions.append((old*self.interval_ns,frozen))
                self.reference_bucket=bucket
                self.reference.observe(compose_grids([(s,(0.,0.,0.))],self.args[0],frame=s.frame),sensor_center(s))
            else:self.recent.append(s)
        if not self.recent or len(self.last_sources)<2:return merged,layers
        latest=self.recent[-1].stamp_ns
        retention=(self.config.source_window_sec+self.config.max_input_age_sec+2*self.config.interval_sec)*1e9
        while self.recent and latest-self.recent[0].stamp_ns>retention:self.recent.popleft()
        if abs(self.last_sources[0]-latest)>self.config.max_input_age_sec*1e9:return merged,layers
        newest=min(self.last_sources.values())//self.interval_ns*self.interval_ns
        recent=[s for s in self.recent if newest-self.config.source_window_sec*1e9<=s.stamp_ns<newest]
        reference=next((s for ns,s in reversed(self.reference_versions) if ns==newest),None)
        if reference is None or not recent or newest-recent[0].stamp_ns<self.config.source_window_sec*.8e9:return merged,layers
        if self.last_attempt_ns is not None and newest<=self.last_attempt_ns:return merged,layers
        if newest<self.reprojection_backoff_until_ns:return merged,layers
        self.last_attempt_ns=newest
        report,patches=register_periodic(reference,recent,self.alignment,self.config,self.registration)
        report['window_end_ns']=newest
        self.last_patches=patches
        if report['accepted']:
            began=time.perf_counter();new=IncrementalFusion(*self.args)
            candidate=tuple(report['alignment_candidate'])
            # Build separately. An exception leaves the previous transform and
            # complete old map intact; never publish a half-reprojected map.
            try:
                for old_seeds,old_updates in self.journal.batches():
                    new_merged,new_layers=new.update(generation,old_seeds,old_updates,candidate,world_from_reference)
                    if time.perf_counter()-began>self.config.max_reprojection_sec:
                        raise TimeoutError('source replay exceeded periodic.max_reprojection_sec; previous map/transform retained')
                self.engine=new;self.alignment=candidate;self.revision+=1
                merged,layers=new_merged,new_layers
                self.last_reprojection_ms=(time.perf_counter()-began)*1000
                self.peak_reprojection_ms=max(self.peak_reprojection_ms,self.last_reprojection_ms)
                report['reprojection_ms']=self.last_reprojection_ms;report['revision']=self.revision
            except Exception as exc:
                report.update(accepted=False,reason='reprojection_failed',error=str(exc),
                              reprojection_ms=(time.perf_counter()-began)*1000)
                # Repeating a too-expensive replay every 2 s would overwhelm
                # an otherwise healthy live map worker. Keep the old C and
                # continue incremental mapping during this source-time pause.
                self.reprojection_backoff_until_ns=newest+round(10*self.config.interval_sec*1e9)
        self.reports.append(report)
        return merged,layers
