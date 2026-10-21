"""ROS-independent occupancy projection, stationary consensus, and atomic saving.

All transforms named a_from_b map B coordinates into A coordinates. Source
snapshots are immutable inputs; merged output is always rebuilt from them.
"""
from __future__ import annotations

from dataclasses import dataclass
from io import BytesIO
from pathlib import Path
import json
import math
import os
import tempfile

import numpy as np
import yaml

from co_3dto2d_mapping.occupancy_submap import GridGeometry
from co_3dto2d_mapping.occupancy_png import (
    OccupancyGridGeometry, TrajectoryOverlay, occupancy_grid_png,
    occupancy_grid_png_with_trajectories, trajectory_pixels,
)
from co_3dto2d_mapping.planar_transform_utils import normalize_angle


@dataclass(frozen=True)
class Snapshot:
    grid: np.ndarray
    geometry: GridGeometry
    stamp_ns: int
    frame: str

    def __post_init__(self):
        self.geometry.validated()
        values = np.array(self.grid, dtype=np.int8, copy=True)
        if values.shape != (self.geometry.height, self.geometry.width):
            raise ValueError('snapshot dimensions do not match geometry')
        if np.any((values < -1) | (values > 100)):
            raise ValueError('invalid occupancy value')
        values.flags.writeable = False
        object.__setattr__(self, 'grid', values)


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


def atomic_write(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix='.'+path.name, dir=path.parent)
    try:
        with os.fdopen(fd, 'wb') as out:
            out.write(data)
            out.flush()
            os.fsync(out.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


def json_safe(value):
    if isinstance(value, dict): return {str(k):json_safe(v) for k,v in value.items()}
    if isinstance(value, (tuple,list)): return [json_safe(v) for v in value]
    if isinstance(value, np.generic): return json_safe(value.item())
    if isinstance(value, float) and not math.isfinite(value): return None
    return value


def save_map(directory, name, snapshot, paths=None):
    directory = Path(directory)
    g, data = snapshot.geometry, snapshot.grid
    array = BytesIO()
    np.savez_compressed(array, data=data, resolution=g.resolution,
                        origin=[g.origin_x,g.origin_y,g.origin_yaw],
                        stamp_ns=snapshot.stamp_ns, frame=snapshot.frame)
    atomic_write(directory/(name+'_data.npz'), array.getvalue())
    gray = np.where(data < 0, 205, np.where(data > 50, 0, 254)).astype(np.uint8)
    atomic_write(directory/(name+'.pgm'),
                 f'P5\n{g.width} {g.height}\n255\n'.encode()+gray[::-1].tobytes())
    metadata = dict(image=name+'.pgm', mode='trinary', resolution=g.resolution,
                    origin=[g.origin_x,g.origin_y,g.origin_yaw], negate=0,
                    occupied_thresh=.65, free_thresh=.196,
                    frame_id=snapshot.frame, source_stamp_ns=snapshot.stamp_ns)
    atomic_write(directory/(name+'.yaml'), yaml.safe_dump(metadata).encode())
    atomic_write(directory/(name+'.png'), occupancy_grid_png(
        width=g.width,height=g.height,occupancy=data.ravel().tolist()))
    if paths is not None:
        geom = OccupancyGridGeometry(g.width,g.height,g.resolution,g.origin_x,g.origin_y,g.origin_yaw)
        colors = [(230,80,40),(0,180,210),(180,100,220),(90,190,80)]
        overlays = [TrajectoryOverlay(trajectory_pixels(geometry=geom,points=points),colors[i%4])
                    for i,points in enumerate(paths.values())]
        atomic_write(directory/(name+'_with_paths.png'),occupancy_grid_png_with_trajectories(
            width=g.width,height=g.height,occupancy=data.ravel().tolist(),trajectories=overlays))
