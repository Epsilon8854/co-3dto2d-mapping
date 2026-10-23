"""Timestamped occupancy evidence on the common lattice (no ROS dependency)."""
from collections import Counter
from dataclasses import dataclass
import math

import numpy as np

from co_3dto2d_mapping.common.occupancy_submap import GridGeometry


@dataclass(frozen=True)
class TemporalFusionConfig:
    enabled: bool = False
    free_clear_count: int = 30
    evidence_timeout_sec: float = 5.0
    observation_bin_sec: float = 0.1
    clear_max_range_m: float = 6.0

    def validated(self):
        if type(self.enabled) is not bool:
            raise ValueError('merged_dynamic_filter_enabled must be boolean')
        if type(self.free_clear_count) is not int or not 1 <= self.free_clear_count <= 65535:
            raise ValueError('merged_dynamic_free_clear_count must be in [1, 65535]')
        for name in ('evidence_timeout_sec', 'observation_bin_sec', 'clear_max_range_m'):
            value = getattr(self, name)
            if isinstance(value, bool) or not math.isfinite(value) or value < 0:
                raise ValueError('invalid merged_dynamic_' + name)
        if round(self.observation_bin_sec * 1e9) < 1:
            raise ValueError('merged_dynamic_observation_bin_sec must be at least 1 ns')
        return self


class TemporalOccupancyAccumulator:
    """One map worker owns each grid; unknown never advances cell evidence.

    Bins use original observation timestamps, not arrival time. Each cell gains
    at most one free vote per bin across both agents. Occupied wins inside a
    bin, including when its slightly older stamp arrives after a free sample.
    Older *bins* cannot change newer evidence. A skipped bin is not free.
    """
    def __init__(self, resolution, config, frame='r0/odom', max_cells=16_000_000):
        if not math.isfinite(resolution) or resolution <= 0:
            raise ValueError('invalid temporal grid resolution')
        self.resolution = resolution
        self.config = config.validated()
        self.frame = frame
        self.max_cells = max_cells
        self.bin_ns = round(config.observation_bin_sec * 1e9)
        self.timeout_ns = round(config.evidence_timeout_sec * 1e9)
        self.grid = self.last_observed_ns = self.free_counts = self.bin_occupied = None
        self.origin = None
        self.stamp_ns = 0
        self.stats = Counter()

    def _view(self, snapshot):
        g = snapshot.geometry
        if (abs(g.resolution - self.resolution) > 1e-7 or abs(g.origin_yaw) > 1e-7
                or snapshot.frame != self.frame):
            raise ValueError('temporal accumulator requires the common aligned lattice')
        if not 0 <= snapshot.stamp_ns <= np.iinfo(np.int64).max:
            raise ValueError('invalid occupancy source timestamp')
        corner = np.asarray([g.origin_x, g.origin_y])
        end = corner + np.array([g.width, g.height]) * self.resolution
        if self.grid is None:
            low, high = corner, end
        else:
            low = np.minimum(self.origin, corner)
            high = np.maximum(self.origin + np.array(self.grid.shape[::-1]) * self.resolution, end)
        width, height = np.rint((high - low) / self.resolution).astype(int)
        if int(width) * int(height) > self.max_cells:
            raise ValueError('temporal accumulator exceeds max_cells')
        if self.grid is None or not np.array_equal(low, self.origin) or self.grid.shape != (height, width):
            arrays = []
            for old, fill, dtype in ((self.grid, -1, np.int8),
                                     (self.last_observed_ns, -1, np.int64),
                                     (self.free_counts, 0, np.uint16),
                                     (self.bin_occupied, False, bool)):
                new = np.full((height, width), fill, dtype=dtype)
                if old is not None:
                    col, row = np.rint((self.origin - low) / self.resolution).astype(int)
                    new[row:row + old.shape[0], col:col + old.shape[1]] = old
                arrays.append(new)
            self.grid, self.last_observed_ns, self.free_counts, self.bin_occupied = arrays
            self.origin = low
        offset = (corner - self.origin) / self.resolution
        if not np.allclose(offset, np.rint(offset), atol=1e-5):
            raise ValueError('misaligned temporal lattice origins')
        col, row = np.rint(offset).astype(int)
        region = np.s_[row:row + g.height, col:col + g.width]
        return tuple(a[region] for a in (self.grid, self.last_observed_ns,
                                       self.free_counts, self.bin_occupied))

    def seed(self, snapshot):
        """Bootstrap/resync supplies no votes and never replaces live evidence.

        Cumulative maps have no per-cell acquisition time. In particular a late
        r1 bootstrap cannot resurrect an obstacle already cleared by r0.
        """
        if not np.any(snapshot.grid >= 0):
            return
        grid, last, _, _ = self._view(snapshot)
        incoming = snapshot.grid
        untouched = last < 0
        occupied = untouched & (incoming > 50)
        free = untouched & (incoming >= 0) & (incoming <= 50) & (grid <= 50)
        grid[occupied] = 100
        grid[free] = 0
        self.stamp_ns = max(self.stamp_ns, snapshot.stamp_ns)
        self.stats['seed_cells'] += int(np.count_nonzero(occupied | free))

    def observe(self, snapshot, sensor_xy=None):
        if not np.any(snapshot.grid >= 0):
            return
        grid, last, counts, bin_occupied = self._view(snapshot)
        incoming = snapshot.grid
        known = incoming >= 0
        # Match the single mapper's safeguard: remote free rays cannot erase
        # an established obstacle outside the configured reliable clear range.
        if self.config.clear_max_range_m > 0:
            if sensor_xy is None or not np.all(np.isfinite(sensor_xy)):
                raise ValueError('temporal clearing requires local-grid sensor center')
            g = snapshot.geometry
            xs = g.origin_x + (np.arange(g.width) + .5) * self.resolution - sensor_xy[0]
            ys = g.origin_y + (np.arange(g.height) + .5) * self.resolution - sensor_xy[1]
            far = xs[None, :] ** 2 + ys[:, None] ** 2 > self.config.clear_max_range_m ** 2
            blocked = known & (incoming <= 50) & (grid > 50) & far
            self.stats['range_blocked_free_cells'] += int(np.count_nonzero(blocked))
            known = known & ~blocked
        stamp = snapshot.stamp_ns
        bucket = stamp // self.bin_ns
        old_bucket = last // self.bin_ns
        stale = known & (last >= 0) & (bucket < old_bucket)
        valid = known & ~stale
        fresh = valid & ((last < 0) | (bucket > old_bucket))
        same = valid & ~fresh
        timed_out = fresh & (last >= 0) & (stamp - last > self.timeout_ns) if self.timeout_ns else np.zeros_like(fresh)
        self.stats['stale_cell_observations'] += int(np.count_nonzero(stale))
        self.stats['same_bin_cell_observations'] += int(np.count_nonzero(same))
        self.stats['timeout_counter_resets'] += int(np.count_nonzero(timed_out & (counts > 0)))
        counts[timed_out] = 0
        bin_occupied[fresh] = False

        occupied = valid & (incoming > 50)
        free = valid & (incoming <= 50)
        # A later free sample in a bin that already contained occupied cannot
        # undo that occupied observation or restart its free streak.
        suppressed = free & bin_occupied
        votes = free & fresh
        old_occupied = grid > 50
        counts[votes] = np.minimum(counts[votes].astype(np.uint32) + 1,
                                  self.config.free_clear_count).astype(np.uint16)
        clear = free & ~suppressed & ((grid <= 50) | (counts >= self.config.free_clear_count))
        self.stats['cleared_cells'] += int(np.count_nonzero(clear & old_occupied))
        self.stats['free_votes'] += int(np.count_nonzero(votes))
        self.stats['same_bin_free_suppressed'] += int(np.count_nonzero(suppressed))
        grid[clear] = 0
        self.stats['occupied_counter_resets'] += int(np.count_nonzero(occupied & (counts > 0)))
        grid[occupied] = 100
        counts[occupied] = 0
        bin_occupied[occupied] = True
        last[valid] = np.maximum(last[valid], stamp)
        self.stats['accepted_cell_observations'] += int(np.count_nonzero(valid))
        self.stamp_ns = max(self.stamp_ns, stamp)

    def snapshot(self):
        if self.grid is None:
            return None
        # Lazy import avoids a dependency cycle with projection/serialization.
        from co_3dto2d_mapping.multi_agent.core import Snapshot
        height, width = self.grid.shape
        return Snapshot(self.grid, GridGeometry(self.resolution, width, height,
                        *map(float, self.origin)), self.stamp_ns, self.frame)

    def diagnostics(self):
        return dict(self.stats)
