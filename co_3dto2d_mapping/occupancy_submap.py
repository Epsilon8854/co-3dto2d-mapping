"""Extract frozen robot-centric 2-D occupancy submaps from growing grids.

The functions in this module intentionally do not depend on ROS message types so
that the geometry can be unit tested without a ROS runtime.  A global
OccupancyGrid is represented by its dense data array plus ``GridGeometry``.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Tuple

import numpy as np

from co_3dto2d_mapping.planar_transform_utils import PlanarTransform


UNKNOWN = np.int8(-1)
FREE = np.int8(0)
OCCUPIED = np.int8(100)


@dataclass(frozen=True)
class GridGeometry:
    resolution: float
    width: int
    height: int
    origin_x: float = 0.0
    origin_y: float = 0.0
    origin_yaw: float = 0.0

    def validated(self) -> "GridGeometry":
        resolution = float(self.resolution)
        width = int(self.width)
        height = int(self.height)
        if not np.isfinite(resolution) or resolution <= 0.0:
            raise ValueError("grid resolution must be a positive finite value")
        if width <= 0 or height <= 0:
            raise ValueError("grid width and height must be positive")
        values = (self.origin_x, self.origin_y, self.origin_yaw)
        if not all(np.isfinite(value) for value in values):
            raise ValueError("grid origin must contain finite values")
        return GridGeometry(
            resolution=resolution,
            width=width,
            height=height,
            origin_x=float(self.origin_x),
            origin_y=float(self.origin_y),
            origin_yaw=float(self.origin_yaw),
        )


@dataclass(frozen=True)
class LocalOccupancyPatch:
    """A frozen circular submap expressed in a keyframe-local coordinate frame."""

    grid: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    radius_m: float
    boundary_points: np.ndarray
    free_points: np.ndarray
    known_points: np.ndarray
    known_ratio: float

    @property
    def occupied_boundary_count(self) -> int:
        return int(self.boundary_points.shape[0])

    @property
    def known_cell_count(self) -> int:
        return int(self.known_points.shape[0])


def _cell_centers(mask: np.ndarray, resolution: float, origin_x: float, origin_y: float) -> np.ndarray:
    rows, cols = np.nonzero(mask)
    if rows.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    return np.column_stack(
        (
            origin_x + (cols.astype(np.float64) + 0.5) * resolution,
            origin_y + (rows.astype(np.float64) + 0.5) * resolution,
        )
    )


def occupied_boundary_mask(grid: np.ndarray, occupied_threshold: int = 50) -> np.ndarray:
    """Return four-connected occupied boundary cells.

    An occupied cell is a boundary when at least one cardinal neighbor is not
    occupied.  The padded exterior is non-occupied, so walls touching the patch
    edge remain available for registration.
    """

    values = np.asarray(grid)
    if values.ndim != 2:
        raise ValueError("grid must be a 2-D array")
    occupied = values > int(occupied_threshold)
    padded = np.pad(occupied, 1, mode="constant", constant_values=False)
    interior = (
        padded[1:-1, :-2]
        & padded[1:-1, 2:]
        & padded[:-2, 1:-1]
        & padded[2:, 1:-1]
    )
    return occupied & ~interior


def extract_local_patch(
    data: np.ndarray,
    geometry: GridGeometry,
    map_from_keyframe: PlanarTransform,
    radius_m: float,
    *,
    output_resolution: float | None = None,
    occupied_threshold: int = 50,
) -> LocalOccupancyPatch:
    """Crop and re-rasterize a global map around one robot pose.

    ``map_from_keyframe`` maps keyframe-local coordinates to the source map.
    The returned patch is centered at the keyframe origin and aligned with the
    keyframe yaw.  Unknown cells remain unknown and occupied observations win
    only when multiple input cells quantize into the same output cell.
    """

    geometry = geometry.validated()
    radius_m = float(radius_m)
    if not np.isfinite(radius_m) or radius_m <= 0.0:
        raise ValueError("radius_m must be a positive finite value")
    resolution = (
        geometry.resolution if output_resolution is None else float(output_resolution)
    )
    if not np.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("output_resolution must be a positive finite value")

    values = np.asarray(data, dtype=np.int16)
    if values.size != geometry.width * geometry.height:
        raise ValueError("grid data length does not match width * height")
    values = values.reshape((geometry.height, geometry.width))

    half_cells = max(1, int(math.ceil(radius_m / resolution)))
    size = 2 * half_cells + 1
    patch_origin_x = -half_cells * resolution
    patch_origin_y = -half_cells * resolution
    patch = np.full((size, size), UNKNOWN, dtype=np.int8)

    # Restrict work to the input-grid bounding box of the requested circle.
    # Long-running maps can contain millions of cells; scanning the whole array
    # for every keyframe would defeat the CPU-only design.
    key_x, key_y, key_yaw = map_from_keyframe
    origin_cos = math.cos(geometry.origin_yaw)
    origin_sin = math.sin(geometry.origin_yaw)
    center_delta_x = float(key_x) - geometry.origin_x
    center_delta_y = float(key_y) - geometry.origin_y
    center_grid_x = origin_cos * center_delta_x + origin_sin * center_delta_y
    center_grid_y = -origin_sin * center_delta_x + origin_cos * center_delta_y
    center_col = center_grid_x / geometry.resolution - 0.5
    center_row = center_grid_y / geometry.resolution - 0.5
    input_radius_cells = int(math.ceil(radius_m / geometry.resolution)) + 2
    min_col = max(0, int(math.floor(center_col)) - input_radius_cells)
    max_col = min(
        geometry.width - 1, int(math.ceil(center_col)) + input_radius_cells
    )
    min_row = max(0, int(math.floor(center_row)) - input_radius_cells)
    max_row = min(
        geometry.height - 1, int(math.ceil(center_row)) + input_radius_cells
    )
    if min_col <= max_col and min_row <= max_row:
        subset = values[min_row : max_row + 1, min_col : max_col + 1]
        subset_rows, subset_cols = np.nonzero(subset >= 0)
        rows = subset_rows + min_row
        cols = subset_cols + min_col
    else:
        rows = np.empty(0, dtype=np.int64)
        cols = np.empty(0, dtype=np.int64)

    if rows.size:
        grid_x = (cols.astype(np.float64) + 0.5) * geometry.resolution
        grid_y = (rows.astype(np.float64) + 0.5) * geometry.resolution
        map_x = geometry.origin_x + origin_cos * grid_x - origin_sin * grid_y
        map_y = geometry.origin_y + origin_sin * grid_x + origin_cos * grid_y

        delta_x = map_x - float(key_x)
        delta_y = map_y - float(key_y)
        key_cos = math.cos(float(key_yaw))
        key_sin = math.sin(float(key_yaw))
        local_x = key_cos * delta_x + key_sin * delta_y
        local_y = -key_sin * delta_x + key_cos * delta_y

        inside = local_x * local_x + local_y * local_y <= radius_m * radius_m
        local_x = local_x[inside]
        local_y = local_y[inside]
        cell_values = values[rows[inside], cols[inside]]
        patch_cols = np.floor((local_x - patch_origin_x) / resolution).astype(
            np.int64
        )
        patch_rows = np.floor((local_y - patch_origin_y) / resolution).astype(
            np.int64
        )
        valid = (
            (patch_cols >= 0)
            & (patch_cols < size)
            & (patch_rows >= 0)
            & (patch_rows < size)
        )
        patch_cols = patch_cols[valid]
        patch_rows = patch_rows[valid]
        cell_values = cell_values[valid]

        free = cell_values <= int(occupied_threshold)
        patch[patch_rows[free], patch_cols[free]] = FREE
        occupied = ~free
        patch[patch_rows[occupied], patch_cols[occupied]] = OCCUPIED

    center_x = patch_origin_x + (np.arange(size, dtype=np.float64) + 0.5) * resolution
    center_y = patch_origin_y + (np.arange(size, dtype=np.float64) + 0.5) * resolution
    circle = center_y[:, None] ** 2 + center_x[None, :] ** 2 <= radius_m * radius_m
    patch[~circle] = UNKNOWN

    boundary = occupied_boundary_mask(patch, occupied_threshold)
    free_mask = (patch >= 0) & (patch <= int(occupied_threshold))
    known_mask = patch >= 0
    denominator = max(1, int(np.count_nonzero(circle)))
    known_ratio = float(np.count_nonzero(known_mask)) / float(denominator)

    return LocalOccupancyPatch(
        grid=patch,
        resolution=resolution,
        origin_x=patch_origin_x,
        origin_y=patch_origin_y,
        radius_m=radius_m,
        boundary_points=_cell_centers(
            boundary, resolution, patch_origin_x, patch_origin_y
        ),
        free_points=_cell_centers(
            free_mask, resolution, patch_origin_x, patch_origin_y
        ),
        known_points=_cell_centers(
            known_mask, resolution, patch_origin_x, patch_origin_y
        ),
        known_ratio=known_ratio,
    )


def sample_points(points: np.ndarray, maximum: int) -> np.ndarray:
    """Deterministically limit a point array without biasing to its prefix."""

    values = np.asarray(points, dtype=np.float64)
    if values.ndim != 2 or values.shape[1:] != (2,):
        raise ValueError("points must have shape (N, 2)")
    maximum = max(1, int(maximum))
    if len(values) <= maximum:
        return values
    indices = np.linspace(0, len(values) - 1, maximum, dtype=np.int64)
    return values[indices]
