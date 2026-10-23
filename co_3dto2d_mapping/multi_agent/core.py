"""ROS-independent occupancy projection, stationary consensus, and atomic saving.

All transforms named a_from_b map B coordinates into A coordinates. Source
snapshots are immutable; temporal fusion owns its common-frame evidence.
"""
from __future__ import annotations

from co_3dto2d_mapping.common.map_storage import Snapshot,atomic_write,json_safe,save_map
from io import BytesIO
from pathlib import Path
import json
import math
import os
import tempfile

import numpy as np
import yaml

from co_3dto2d_mapping.common.occupancy_submap import GridGeometry
from co_3dto2d_mapping.common.occupancy_png import (
    OccupancyGridGeometry, TrajectoryOverlay, occupancy_grid_png,
    occupancy_grid_png_with_trajectories, trajectory_pixels,
)
from co_3dto2d_mapping.common.planar_transform_utils import normalize_angle,compose_planar
from co_3dto2d_mapping.multi_agent.temporal import TemporalFusionConfig,TemporalOccupancyAccumulator




def transform_xy(points, transform):
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    x, y, yaw = transform
    c, s = math.cos(yaw), math.sin(yaw)
    return points @ np.array([[c, s], [-s, c]]) + [x, y]


def _world_cells(snapshot, transform):
    g = snapshot.geometry
    rows, cols = np.nonzero(snapshot.grid >= 0)
    centers = np.column_stack(((cols + .5) * g.resolution, (rows + .5) * g.resolution))
    centers = transform_xy(centers, (g.origin_x, g.origin_y, g.origin_yaw))
    return transform_xy(centers, transform), snapshot.grid[rows, cols]


def compose_grids(sources, resolution=.05, frame='r0/odom', max_cells=16_000_000):
    """Conservative occupied area rasterization; unknown never erases evidence.

    Separating-axis tests cover every output cell with positive area overlap
    with an occupied source cell. Free evidence is center-sampled. This avoids
    holes in rotated thin walls without filling unknown areas with free space.
    """
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('resolution must be finite and positive')
    entries = []
    low, high = [], []
    for snapshot, transform in sources:
        if not np.all(np.isfinite(transform)):
            raise ValueError('non-finite transform')
        centers, values = _world_cells(snapshot, transform)
        if not len(values):
            continue
        radius = snapshot.geometry.resolution / math.sqrt(2)
        low.append(centers.min(axis=0) - radius)
        high.append(centers.max(axis=0) + radius)
        entries.append((snapshot, transform, centers, values))
    if not entries:
        raise ValueError('no known occupancy cells')
    origin = np.floor(np.min(low, axis=0) / resolution + 1e-9) * resolution
    end = np.ceil(np.max(high, axis=0) / resolution - 1e-9) * resolution
    width, height = np.maximum(1, np.rint((end-origin)/resolution).astype(int))
    if int(width) * int(height) > max_cells:
        raise ValueError('merged grid exceeds max_cells; check transforms')
    data = np.full((height, width), -1, dtype=np.int8)
    # Write all free centers before any occupied areas (occupied wins).
    for _, _, centers, values in entries:
        free = centers[values <= 50]
        cols, rows = np.floor((free-origin)/resolution).astype(int).T
        valid = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
        data[rows[valid], cols[valid]] = 0
    for snapshot, transform, centers, values in entries:
        occupied = centers[values > 50]
        yaw = snapshot.geometry.origin_yaw + transform[2]
        c, s = math.cos(yaw), math.sin(yaw)
        hs, hd = snapshot.geometry.resolution/2, resolution/2
        reach = int(math.ceil((hs * math.sqrt(2) + hd) / resolution))
        base = np.floor((occupied-origin)/resolution).astype(int)
        for dc in range(-reach, reach+1):
            for dr in range(-reach, reach+1):
                cells = base + [dc, dr]
                valid = ((cells[:, 0] >= 0) & (cells[:, 0] < width) &
                         (cells[:, 1] >= 0) & (cells[:, 1] < height))
                delta = origin + (cells+.5)*resolution - occupied
                extent = hd + hs*(abs(c)+abs(s))
                own_extent = hs + hd*(abs(c)+abs(s))
                hit = (valid & (np.abs(delta[:, 0]) < extent-1e-9) &
                       (np.abs(delta[:, 1]) < extent-1e-9) &
                       (np.abs(delta[:, 0]*c+delta[:, 1]*s) < own_extent-1e-9) &
                       (np.abs(-delta[:, 0]*s+delta[:, 1]*c) < own_extent-1e-9))
                selected = cells[hit]
                data[selected[:, 1], selected[:, 0]] = 100
    geometry = GridGeometry(resolution, int(width), int(height), *map(float, origin))
    return Snapshot(data, geometry, max(s.stamp_ns for s, _ in sources), frame)


class StationaryConsensus:
    """Different non-overlapping observation windows, never repeated publishes."""
    def __init__(self, count=3, translation_m=.10, yaw_deg=.5):
        if count < 1 or translation_m <= 0 or yaw_deg <= 0:
            raise ValueError('invalid stationary consensus configuration')
        self.count = count
        self.translation_m = translation_m
        self.yaw_rad = math.radians(yaw_deg)
        self.evidence = []

    def reset(self):
        self.evidence.clear()

    def add(self, transform, windows):
        # Each item contains (r0_start,r0_end,r1_start,r1_end) nanoseconds.
        if not np.all(np.isfinite(transform)) or any(windows[i] >= windows[i+1] for i in (0,2)):
            return None
        if self.evidence:
            old_windows = self.evidence[-1][1]
            if windows[0] <= old_windows[1] or windows[2] <= old_windows[3]:
                return None
            if any(math.hypot(transform[0]-t[0], transform[1]-t[1]) > self.translation_m or
                   abs(normalize_angle(transform[2]-t[2])) > self.yaw_rad
                   for t, _ in self.evidence):
                self.reset()
        self.evidence.append((tuple(transform), tuple(windows)))
        self.evidence = self.evidence[-self.count:]
        if len(self.evidence) < self.count:
            return None
        poses = np.array([t for t, _ in self.evidence])
        return (float(poses[:,0].mean()), float(poses[:,1].mean()),
                float(math.atan2(np.sin(poses[:,2]).mean(), np.cos(poses[:,2]).mean())))


class SnapshotConsensus(StationaryConsensus):
    """Stability over advancing cumulative snapshots, not independent scans."""
    def add(self,transform,stamps):
        if len(stamps)!=2 or not np.all(np.isfinite(transform)):return None
        if self.evidence:
            if any(now<=old for now,old in zip(stamps,self.evidence[-1][1])):return None
            if any(math.hypot(transform[0]-t[0],transform[1]-t[1])>self.translation_m or
                   abs(normalize_angle(transform[2]-t[2]))>self.yaw_rad for t,_ in self.evidence):self.reset()
        self.evidence.append((tuple(transform),tuple(stamps)));self.evidence=self.evidence[-self.count:]
        if len(self.evidence)<self.count:return None
        poses=np.asarray([t for t,_ in self.evidence])
        return float(poses[:,0].mean()),float(poses[:,1].mean()),float(math.atan2(np.sin(poses[:,2]).mean(),np.cos(poses[:,2]).mean()))


class OccupancyAccumulator:
    """Legacy occupied-wins lattice used when central temporal clearing is OFF."""
    def __init__(self,resolution,frame='r0/odom',max_cells=16_000_000):
        if not math.isfinite(resolution) or resolution<=0:raise ValueError('invalid resolution')
        self.resolution=resolution;self.frame=frame;self.max_cells=max_cells
        self.grid=None;self.origin=None;self.stamp_ns=0

    def add(self,snapshot):
        g=snapshot.geometry
        if abs(g.resolution-self.resolution)>1e-7 or abs(g.origin_yaw)>1e-7 or snapshot.frame!=self.frame:
            raise ValueError('accumulator requires the common aligned lattice')
        if not np.any(snapshot.grid>=0):return
        corner=np.asarray([g.origin_x,g.origin_y]);end=corner+[g.width*self.resolution,g.height*self.resolution]
        if self.grid is None:
            if snapshot.grid.size>self.max_cells:raise ValueError('accumulator exceeds max_cells')
            self.origin=corner;self.grid=np.array(snapshot.grid,copy=True)
        else:
            old_end=self.origin+np.array(self.grid.shape[::-1])*self.resolution
            low=np.minimum(self.origin,corner);high=np.maximum(old_end,end)
            width,height=np.rint((high-low)/self.resolution).astype(int)
            if int(width)*int(height)>self.max_cells:raise ValueError('accumulator exceeds max_cells')
            if not np.array_equal(low,self.origin) or (height,width)!=self.grid.shape:
                expanded=np.full((height,width),-1,dtype=np.int8)
                col,row=np.rint((self.origin-low)/self.resolution).astype(int)
                expanded[row:row+self.grid.shape[0],col:col+self.grid.shape[1]]=self.grid
                self.grid=expanded;self.origin=low
            offset=(corner-self.origin)/self.resolution
            if not np.allclose(offset,np.rint(offset),atol=1e-5):raise ValueError('misaligned lattice origins')
            col,row=np.rint(offset).astype(int);view=self.grid[row:row+g.height,col:col+g.width]
            incoming=snapshot.grid;known=incoming>=0
            free=known&(incoming<=50)&(view<=50);view[free]=0;view[incoming>50]=100
        self.stamp_ns=max(self.stamp_ns,snapshot.stamp_ns)

    def snapshot(self):
        if self.grid is None:return None
        height,width=self.grid.shape
        return Snapshot(self.grid,GridGeometry(self.resolution,width,height,*map(float,self.origin)),self.stamp_ns,self.frame)


class IncrementalFusion:
    """One worker owns all state, including cross-agent temporal evidence.

    With filtering enabled the common map ingests fresh observations directly;
    it is never rebuilt as a union of stale per-agent occupied layers.
    """
    def __init__(self,resolution=.05,max_cells=16_000_000,temporal_config=None,frame='r0/odom'):
        self.resolution=resolution;self.max_cells=max_cells;self.generation=None;self.layers={}
        self.frame=frame;self.world_from_reference=None;self.alignment=None
        self.temporal_config=(temporal_config or TemporalFusionConfig()).validated()
        self.common=None;self.last_local={};self.live_agents=set();self.stats={}

    def _accumulator(self):
        if self.temporal_config.enabled:
            return TemporalOccupancyAccumulator(self.resolution,self.temporal_config,frame=self.frame,max_cells=self.max_cells)
        return OccupancyAccumulator(self.resolution,frame=self.frame,max_cells=self.max_cells)

    def diagnostics(self):
        return dict(enabled=self.temporal_config.enabled,
                    policy='timestamped_local_evidence' if self.temporal_config.enabled else 'occupied_wins',
                    input_counts=dict(self.stats),
                    latest_source_stamps_ns=dict(self.last_local),
                    common=self.common.diagnostics() if self.common is not None else {},
                    agents={rid:layer.diagnostics() for rid,layer in self.layers.items()}
                    if self.temporal_config.enabled else {})

    def update(self,generation,seeds,updates,alignment,world_from_reference=(0.,0.,0.)):
        if generation!=self.generation:
            self.generation=generation
            self.layers={rid:self._accumulator() for rid in (0,1)}
            self.common=self._accumulator() if self.temporal_config.enabled else None
            self.last_local={};self.live_agents=set();self.stats={}
            self.world_from_reference=tuple(world_from_reference);self.alignment=tuple(alignment)
        elif self.world_from_reference!=tuple(world_from_reference) or self.alignment!=tuple(alignment):
            # A transform change requires source-observation replay, not silent
            # insertion into a lattice containing old transformed evidence.
            raise ValueError('fusion transform changed: source-observation reprojection is required')
        # Global snapshots are bootstrap/recovery data, not new observations.
        # They cannot overwrite cells already observed live, even if a global
        # resync arrives late following a queue overflow.
        for rid,snapshot in sorted(seeds.items()):
            transform=world_from_reference if rid==0 else compose_planar(world_from_reference,alignment)
            if not np.any(snapshot.grid>=0):continue
            projected=compose_grids([(snapshot,transform)],self.resolution,frame=self.frame,max_cells=self.max_cells)
            if self.temporal_config.enabled:
                self.layers[rid].seed(projected);self.common.seed(projected)
            else:self.layers[rid].add(projected)
            if rid not in self.live_agents:
                self.last_local[rid]=max(self.last_local.get(rid,-1),snapshot.stamp_ns)
        # Sorting bounds arrival-order effects within a worker batch. Across
        # batches, cell bins reject stale evidence; streams never get restamped.
        for rid,snapshot in sorted(updates,key=lambda item:(item[1].stamp_ns,item[0])):
            if snapshot.stamp_ns<=self.last_local.get(rid,-1):
                key=f'r{rid}_duplicate_or_old_frames';self.stats[key]=self.stats.get(key,0)+1
                continue
            self.last_local[rid]=snapshot.stamp_ns;self.live_agents.add(rid)
            key=f'r{rid}_accepted_frames';self.stats[key]=self.stats.get(key,0)+1
            if not np.any(snapshot.grid>=0):continue
            transform=world_from_reference if rid==0 else compose_planar(world_from_reference,alignment)
            projected=compose_grids([(snapshot,transform)],self.resolution,frame=self.frame,max_cells=self.max_cells)
            if self.temporal_config.enabled:
                g=snapshot.geometry
                center=transform_xy([[g.width*g.resolution/2,g.height*g.resolution/2]],
                                    (g.origin_x,g.origin_y,g.origin_yaw))
                sensor_xy=transform_xy(center,transform)[0]
                self.layers[rid].observe(projected,sensor_xy)
                self.common.observe(projected,sensor_xy)
            else:self.layers[rid].add(projected)
        layers={rid:layer.snapshot() for rid,layer in self.layers.items()}
        if self.temporal_config.enabled:
            return self.common.snapshot(),layers
        merged=OccupancyAccumulator(self.resolution,frame=self.frame,max_cells=self.max_cells)
        for snapshot in layers.values():
            if snapshot is not None:merged.add(snapshot)
        return merged.snapshot(),layers
