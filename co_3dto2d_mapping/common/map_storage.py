"""Immutable map snapshots and atomic map export shared by both modes."""
from __future__ import annotations
from dataclasses import dataclass
from io import BytesIO
from pathlib import Path
import json,math,os,tempfile
import numpy as np
import yaml
from co_3dto2d_mapping.common.occupancy_submap import GridGeometry
from co_3dto2d_mapping.common.occupancy_png import OccupancyGridGeometry,TrajectoryOverlay,occupancy_grid_png,occupancy_grid_png_with_trajectories,trajectory_pixels
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

def atomic_write(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    (fd, tmp) = tempfile.mkstemp(prefix='.' + path.name, dir=path.parent)
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
    if isinstance(value, dict):
        return {str(k): json_safe(v) for (k, v) in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(v) for v in value]
    if isinstance(value, np.generic):
        return json_safe(value.item())
    if isinstance(value, float) and (not math.isfinite(value)):
        return None
    return value

def save_map(directory, name, snapshot, paths=None):
    directory = Path(directory)
    (g, data) = (snapshot.geometry, snapshot.grid)
    array = BytesIO()
    np.savez_compressed(array, data=data, resolution=g.resolution, origin=[g.origin_x, g.origin_y, g.origin_yaw], stamp_ns=snapshot.stamp_ns, frame=snapshot.frame)
    atomic_write(directory / (name + '_data.npz'), array.getvalue())
    gray = np.where(data < 0, 205, np.where(data > 50, 0, 254)).astype(np.uint8)
    atomic_write(directory / (name + '.pgm'), f'P5\n{g.width} {g.height}\n255\n'.encode() + gray[::-1].tobytes())
    metadata = dict(image=name + '.pgm', mode='trinary', resolution=g.resolution, origin=[g.origin_x, g.origin_y, g.origin_yaw], negate=0, occupied_thresh=0.65, free_thresh=0.196, frame_id=snapshot.frame, source_stamp_ns=snapshot.stamp_ns)
    atomic_write(directory / (name + '.yaml'), yaml.safe_dump(metadata).encode())
    atomic_write(directory / (name + '.png'), occupancy_grid_png(width=g.width, height=g.height, occupancy=data.ravel().tolist()))
    if paths is not None:
        geom = OccupancyGridGeometry(g.width, g.height, g.resolution, g.origin_x, g.origin_y, g.origin_yaw)
        colors = [(230, 80, 40), (0, 180, 210), (180, 100, 220), (90, 190, 80)]
        overlays = [TrajectoryOverlay(trajectory_pixels(geometry=geom, points=points), colors[i % 4]) for (i, points) in enumerate(paths.values())]
        atomic_write(directory / (name + '_with_paths.png'), occupancy_grid_png_with_trajectories(width=g.width, height=g.height, occupancy=data.ravel().tolist(), trajectories=overlays))
