"""Pure helpers for LiDAR-frame startup occupancy maps.

The startup map intentionally uses the raw LiDAR coordinate system.  Its height
band is symmetric about LiDAR z=0, so changing the sign convention of the
sensor z axis (for example an upside-down but otherwise identical mounting)
does not change which XY returns are retained.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Tuple

import numpy as np


_VALID_REAR_AXES = {"x", "-x", "y", "-y"}


@dataclass(frozen=True)
class LidarSliceConfig:
    """Filtering applied before projecting a raw XYZ cloud into XY."""

    slice_center_z_m: float = 0.0
    slice_half_height_m: float = 0.40
    center_box_half_extent_m: float = 0.80
    range_min_m: float = 0.80
    range_max_m: float = 12.0
    rear_filter_enabled: bool = False
    rear_filter_angle_deg: float = 120.0
    rear_filter_axis: str = "-x"
    rear_filter_min_xy_range_m: float = 0.0

    def validate(self) -> None:
        finite_values = (
            self.slice_center_z_m,
            self.slice_half_height_m,
            self.center_box_half_extent_m,
            self.range_min_m,
            self.range_max_m,
            self.rear_filter_angle_deg,
            self.rear_filter_min_xy_range_m,
        )
        if not all(math.isfinite(value) for value in finite_values):
            raise ValueError("startup LiDAR slice parameters must be finite")
        if self.slice_half_height_m <= 0.0:
            raise ValueError("slice_half_height_m must be positive")
        if self.center_box_half_extent_m < 0.0:
            raise ValueError("center_box_half_extent_m must be non-negative")
        if self.range_min_m < 0.0 or self.range_max_m < 0.0:
            raise ValueError("range limits must be non-negative")
        if self.range_max_m > 0.0 and self.range_min_m > self.range_max_m:
            raise ValueError("range_min_m must not exceed range_max_m")
        if self.rear_filter_min_xy_range_m < 0.0:
            raise ValueError("rear_filter_min_xy_range_m must be non-negative")
        if self.rear_filter_axis.lower() not in _VALID_REAR_AXES:
            raise ValueError("rear_filter_axis must be one of x, -x, y, -y")


@dataclass(frozen=True)
class SliceFilterStats:
    input_points: int
    finite_points: int
    rejected_z: int
    rejected_range: int
    rejected_center: int
    rejected_rear: int
    kept_below_center: int
    kept_at_or_above_center: int
    kept_points: int


@dataclass(frozen=True)
class OccupancyRaster:
    width: int
    height: int
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    data: np.ndarray
    occupied_cells: int
    out_of_bounds_points: int


def _rear_axis_angle(axis: str) -> float:
    normalized = axis.lower()
    if normalized == "x":
        return 0.0
    if normalized == "y":
        return 0.5 * math.pi
    if normalized == "-y":
        return -0.5 * math.pi
    if normalized == "-x":
        return math.pi
    raise ValueError("rear_filter_axis must be one of x, -x, y, -y")


def _rear_sector_mask(
    xy: np.ndarray,
    *,
    axis: str,
    angle_deg: float,
    min_xy_range_m: float,
) -> np.ndarray:
    if len(xy) == 0:
        return np.zeros(0, dtype=bool)
    half_angle = math.radians(max(0.0, min(360.0, angle_deg)) * 0.5)
    center = _rear_axis_angle(axis)
    ranges = np.hypot(xy[:, 0], xy[:, 1])
    angles = np.arctan2(xy[:, 1], xy[:, 0])
    differences = np.arctan2(np.sin(angles - center), np.cos(angles - center))
    return (ranges > min_xy_range_m) & (np.abs(differences) <= half_angle)


def filter_lidar_slice(
    points_xyz: np.ndarray,
    config: LidarSliceConfig,
) -> Tuple[np.ndarray, SliceFilterStats]:
    """Return XY points from a symmetric raw-LiDAR height slice.

    Filtering is deliberately performed before any sensor-to-base transform.
    The symmetric predicate ``abs(z - center) <= half_height`` treats positive
    and negative LiDAR z identically, which makes the slice invariant to a z-axis
    sign flip.
    """

    config.validate()
    points = np.asarray(points_xyz, dtype=np.float64)
    if points.ndim != 2 or points.shape[1:] != (3,):
        raise ValueError("points_xyz must have shape (N, 3)")

    input_points = int(len(points))
    finite = points[np.all(np.isfinite(points), axis=1)]
    finite_points = int(len(finite))
    if finite_points == 0:
        stats = SliceFilterStats(
            input_points=input_points,
            finite_points=0,
            rejected_z=0,
            rejected_range=0,
            rejected_center=0,
            rejected_rear=0,
            kept_below_center=0,
            kept_at_or_above_center=0,
            kept_points=0,
        )
        return np.empty((0, 2), dtype=np.float64), stats

    z_distance = np.abs(finite[:, 2] - config.slice_center_z_m)
    z_keep = z_distance <= config.slice_half_height_m + 1e-12
    rejected_z = int(np.count_nonzero(~z_keep))
    stage = finite[z_keep]

    ranges = np.hypot(stage[:, 0], stage[:, 1])
    range_keep = ranges >= config.range_min_m
    if config.range_max_m > 0.0:
        range_keep &= ranges <= config.range_max_m
    rejected_range = int(np.count_nonzero(~range_keep))
    stage = stage[range_keep]

    if config.center_box_half_extent_m > 0.0:
        half = config.center_box_half_extent_m
        inside_center = (np.abs(stage[:, 0]) <= half) & (
            np.abs(stage[:, 1]) <= half
        )
    else:
        inside_center = np.zeros(len(stage), dtype=bool)
    rejected_center = int(np.count_nonzero(inside_center))
    stage = stage[~inside_center]

    if config.rear_filter_enabled:
        rear = _rear_sector_mask(
            stage[:, :2],
            axis=config.rear_filter_axis,
            angle_deg=config.rear_filter_angle_deg,
            min_xy_range_m=config.rear_filter_min_xy_range_m,
        )
    else:
        rear = np.zeros(len(stage), dtype=bool)
    rejected_rear = int(np.count_nonzero(rear))
    kept = stage[~rear]

    below = int(np.count_nonzero(kept[:, 2] < config.slice_center_z_m))
    above = int(len(kept) - below)
    xy = np.ascontiguousarray(kept[:, :2], dtype=np.float64)
    stats = SliceFilterStats(
        input_points=input_points,
        finite_points=finite_points,
        rejected_z=rejected_z,
        rejected_range=rejected_range,
        rejected_center=rejected_center,
        rejected_rear=rejected_rear,
        kept_below_center=below,
        kept_at_or_above_center=above,
        kept_points=int(len(xy)),
    )
    return xy, stats


def limit_xy_points(points_xy: np.ndarray, maximum: int) -> np.ndarray:
    """Deterministically cap a point set while retaining its full span."""

    points = np.asarray(points_xy, dtype=np.float64)
    if points.ndim != 2 or points.shape[1:] != (2,):
        raise ValueError("points_xy must have shape (N, 2)")
    if maximum < 1:
        raise ValueError("maximum must be positive")
    if len(points) <= maximum:
        return points
    indices = np.linspace(0, len(points) - 1, maximum, dtype=np.int64)
    return np.ascontiguousarray(points[indices], dtype=np.float64)


def rasterize_occupancy(
    points_xy: np.ndarray,
    *,
    resolution_m: float,
    half_extent_m: float,
    occupied_threshold_points: int = 1,
) -> OccupancyRaster:
    """Rasterize XY returns into a centered, occupied/unknown grid."""

    points = np.asarray(points_xy, dtype=np.float64)
    if points.ndim != 2 or points.shape[1:] != (2,):
        raise ValueError("points_xy must have shape (N, 2)")
    if not math.isfinite(resolution_m) or resolution_m <= 0.0:
        raise ValueError("resolution_m must be positive and finite")
    if not math.isfinite(half_extent_m) or half_extent_m <= 0.0:
        raise ValueError("half_extent_m must be positive and finite")
    if occupied_threshold_points < 1:
        raise ValueError("occupied_threshold_points must be positive")

    width = max(1, int(math.ceil((2.0 * half_extent_m) / resolution_m)))
    height = width
    span = width * resolution_m
    origin_x = -0.5 * span
    origin_y = -0.5 * span

    counts = np.zeros(width * height, dtype=np.int32)
    out_of_bounds = 0
    if len(points):
        finite = points[np.all(np.isfinite(points), axis=1)]
        columns = np.floor((finite[:, 0] - origin_x) / resolution_m).astype(
            np.int64
        )
        rows = np.floor((finite[:, 1] - origin_y) / resolution_m).astype(
            np.int64
        )
        inside = (
            (columns >= 0)
            & (columns < width)
            & (rows >= 0)
            & (rows < height)
        )
        out_of_bounds = int(len(points) - np.count_nonzero(inside))
        flat = rows[inside] * width + columns[inside]
        np.add.at(counts, flat, 1)

    data = np.full(width * height, -1, dtype=np.int8)
    occupied = counts >= occupied_threshold_points
    data[occupied] = 100
    return OccupancyRaster(
        width=width,
        height=height,
        resolution_m=float(resolution_m),
        origin_x_m=float(origin_x),
        origin_y_m=float(origin_y),
        data=data,
        occupied_cells=int(np.count_nonzero(occupied)),
        out_of_bounds_points=out_of_bounds,
    )
