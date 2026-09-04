"""Plane-relative point-height filtering helpers.

The plane is represented in a local robot frame as::

    normal.dot(point) + offset = 0

``normal`` points away from the ground (toward IMU-derived up). Therefore the
signed height above the plane is ``normal.dot(point) + offset``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class PlaneHeightFilterResult:
    points: np.ndarray
    heights_m: np.ndarray
    mask: np.ndarray


def signed_plane_heights(
    points_xyz: np.ndarray,
    normal: Sequence[float],
    offset_m: float,
) -> np.ndarray:
    """Return signed perpendicular height above a plane for each XYZ point."""

    points = np.asarray(points_xyz, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_xyz must have shape (N, 3)")
    normal_vector = np.asarray(normal, dtype=np.float64).reshape(3)
    normal_length = float(np.linalg.norm(normal_vector))
    if not np.isfinite(normal_length) or normal_length <= 1e-12:
        raise ValueError("normal must be finite and non-zero")
    if not np.isfinite(float(offset_m)):
        raise ValueError("offset_m must be finite")
    unit_normal = normal_vector / normal_length
    return points @ unit_normal + float(offset_m) / normal_length


def filter_points_by_plane_height(
    points_xyz: np.ndarray,
    normal: Sequence[float],
    offset_m: float,
    min_height_m: float,
    max_height_m: float,
) -> PlaneHeightFilterResult:
    """Keep finite points inside an inclusive plane-relative height band."""

    if not np.isfinite(float(min_height_m)) or not np.isfinite(float(max_height_m)):
        raise ValueError("plane height limits must be finite")
    if min_height_m < 0.0:
        raise ValueError("min_height_m must be non-negative")
    if max_height_m <= min_height_m:
        raise ValueError("max_height_m must be greater than min_height_m")

    points = np.asarray(points_xyz, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_xyz must have shape (N, 3)")
    finite = np.all(np.isfinite(points), axis=1)
    heights = np.full(len(points), np.nan, dtype=np.float64)
    if np.any(finite):
        heights[finite] = signed_plane_heights(points[finite], normal, offset_m)
    mask = finite & (heights >= min_height_m) & (heights <= max_height_m)
    return PlaneHeightFilterResult(
        points=np.ascontiguousarray(points[mask], dtype=np.float64),
        heights_m=heights,
        mask=mask,
    )
