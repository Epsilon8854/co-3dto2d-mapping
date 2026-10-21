"""Full-circle startup SE(2) registration; no zero-heading assumption."""
from __future__ import annotations
from dataclasses import dataclass, asdict, replace
import math
import time

import numpy as np
from scipy.spatial import cKDTree

from co_3dto2d_mapping.occupancy_submap import sample_points
from co_3dto2d_mapping.planar_transform_utils import normalize_angle, invert_planar
from co_3dto2d_mapping.se2_map_registration import (
    RegistrationConfig, _boundary_distance_field, _best_raster_score_for_yaw,
    _translation_offsets, _trimmed_icp, _direction_metrics, _free_conflict_ratio,
    transform_points,
)


@dataclass(frozen=True)
class StartupConfig:
    coarse_yaw_step_deg: float = 5.0
    top_candidates: int = 5
    distinct_yaw_deg: float = 15.0
    distinct_translation_m: float = .75
    min_score_gap_ratio: float = .10
    min_observability_ratio: float = .001
    # Coarse raster search has a flat zero plateau at cell resolution. Fine
    # verification uses both symmetric point distance and known/free conflict.
    def validated(self):
        if not 0 < self.coarse_yaw_step_deg <= 15 or self.top_candidates < 2:
            raise ValueError('full-circle search requires step <=15 degrees and >=2 candidates')
        if not 0 < self.distinct_yaw_deg < 180 or self.distinct_translation_m <= 0:
            raise ValueError('invalid mode separation')
        if not 0 <= self.min_score_gap_ratio < 1 or not 0 <= self.min_observability_ratio < 1:
            raise ValueError('invalid confidence settings')
        return self


def _distinct(a,b,cfg):
    return (abs(normalize_angle(a[2]-b[2])) >= math.radians(cfg.distinct_yaw_deg) or
            math.hypot(a[0]-b[0],a[1]-b[1]) >= cfg.distinct_translation_m)


def _observability(target, source, transform, config):
    """Normalized point-to-wall information; rejects unconstrained wall slide."""
    target_points = target.boundary_points
    if len(target_points) < 6: return 0.0
    tree = cKDTree(target_points)
    _, neighbors = tree.query(target_points, k=min(7,len(target_points)))
    local = target_points[neighbors]
    local -= local.mean(axis=1,keepdims=True)
    covariance = np.einsum('nki,nkj->nij',local,local)
    _, vecs = np.linalg.eigh(covariance)
    normals = vecs[:,:,0]
    points = transform_points(source.boundary_points,transform)
    distances, indices = tree.query(points,k=1)
    valid = distances <= config.icp_max_correspondence_m
    if np.count_nonzero(valid) < config.min_correspondences: return 0.0
    points = points[valid] - np.asarray(transform[:2])
    normals = normals[indices[valid]]
    radius = max(1.0, float(np.sqrt(np.mean(np.sum(points*points,axis=1)))))
    jac = np.column_stack((normals,(-normals[:,0]*points[:,1]+normals[:,1]*points[:,0])/radius))
    eigen = np.linalg.eigvalsh(jac.T@jac/len(jac))
    return float(max(0,eigen[0])/max(eigen[-1],1e-12))


def register_startup(target,source,config=None,startup=None):
    config = (config or RegistrationConfig()).validated()
    startup = (startup or StartupConfig()).validated()
    started = time.perf_counter()
    def result(reason, candidates=None, **extra):
        return dict(accepted=reason=='accepted',reason=reason,candidates=candidates or [],
                    runtime_ms=(time.perf_counter()-started)*1000,**extra)
    if min(target.occupied_boundary_count,source.occupied_boundary_count) < config.min_correspondences:
        return result('insufficient_boundary_points')
    tf, sf = _boundary_distance_field(target), _boundary_distance_field(source)
    tp = sample_points(target.boundary_points,config.search_max_points)
    sp = sample_points(source.boundary_points,config.search_max_points)
    offsets = _translation_offsets(config.coarse_translation_range_m, config.coarse_translation_step_m)
    coarse = []
    # Always cover the entire circle, even when descriptor yaw is ambiguous.
    for angle in np.arange(-180.,180.,startup.coarse_yaw_step_deg):
        yaw = math.radians(float(angle))
        c,s=math.cos(yaw),math.sin(yaw)
        center = tp.mean(axis=0)-sp.mean(axis=0)@np.array([[c,s],[-s,c]])
        for origin in (np.zeros(2),center):
            pose,score = _best_raster_score_for_yaw(target,tf,source,sf,tp,sp,yaw,origin,offsets,config)
            coarse.append((score,pose))
    coarse.sort(key=lambda item:(item[0],item[1]))
    modes=[]
    for score,pose in coarse:
        if all(_distinct(pose,p, startup) for _,p in modes): modes.append((score,pose))
        if len(modes)>=startup.top_candidates:break
    refined=[]
    fine_offsets=_translation_offsets(config.fine_translation_range_m,config.fine_translation_step_m)
    for score,pose in modes:
        fine_pose, fine_score = pose,score
        # A full-circle coarse bin can be 2.5 deg away from its true mode.
        half=max(config.fine_yaw_range_rad,math.radians(startup.coarse_yaw_step_deg)/2)
        for yaw in np.arange(pose[2]-half,pose[2]+half+1e-8,config.fine_yaw_step_rad):
            p,s=_best_raster_score_for_yaw(target,tf,source,sf,tp,sp,float(yaw),np.array(pose[:2]),fine_offsets,config)
            if s<fine_score:fine_pose,fine_score=p,s
        pose=_trimmed_icp(target.boundary_points,source.boundary_points,fine_pose,config)
        f,fo,fc=_direction_metrics(target.boundary_points,source.boundary_points,pose,config)
        r,ro,rc=_direction_metrics(source.boundary_points,target.boundary_points,invert_planar(pose),config)
        distances=np.concatenate((f,r))
        rmse=float(np.sqrt(np.mean(distances**2))) if distances.size else math.inf
        overlap=min(fo,ro)
        conflict=_free_conflict_ratio(target,source,pose,config)
        observable=_observability(target,source,pose,config)
        # Evaluate competing modes with identical metrics, including rejected
        # ones. A competitor is not silently discarded merely for failing a gate.
        cost=rmse**2+config.overlap_penalty_weight*(1-overlap)**2+config.distance_clip_m**2*conflict
        reason='accepted'
        if min(fc,rc)<config.min_correspondences:reason='insufficient_correspondences'
        elif overlap<config.min_symmetric_overlap:reason='low_overlap'
        elif rmse>config.max_symmetric_rmse_m:reason='high_rmse'
        elif conflict>config.max_free_conflict_ratio:reason='free_space_conflict'
        elif observable<startup.min_observability_ratio:reason='unobservable_geometry'
        if all(_distinct(pose,entry['transform'],startup) for entry in refined):
            refined.append(dict(transform=tuple(map(float,pose)),score=float(cost),
                                rmse_m=rmse,overlap=float(overlap),free_conflict=float(conflict),
                                correspondences=int(min(fc,rc)),observability=observable,reason=reason))
    refined.sort(key=lambda entry:(entry['score'],entry['transform']))
    if not refined:return result('no_finite_candidate')
    best=refined[0]
    if best['reason']!='accepted':return result(best['reason'],refined)
    if len(refined)<2:
        # Full-circle search must retain a competing well to assess symmetry.
        return result('insufficient_distinct_candidates',refined)
    gap=(refined[1]['score']-best['score'])/max(refined[1]['score'],1e-9)
    if gap<startup.min_score_gap_ratio:return result('ambiguous_yaw_or_translation',refined,score_gap_ratio=gap)
    return result('accepted',refined,transform=best['transform'],score_gap_ratio=float(gap),
                  searched_yaw_deg=[-180,180],coarse_yaw_count=len(np.arange(-180,180,startup.coarse_yaw_step_deg)))
