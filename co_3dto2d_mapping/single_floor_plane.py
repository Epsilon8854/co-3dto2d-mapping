"""Fast single-floor plane estimation under an IMU gravity prior.

The target deployment is an approximately horizontal, single-floor environment.
The IMU-derived up vector therefore fixes the plane normal and the LiDAR only
needs to estimate the signed floor offset (robot height).  Reducing the fit to a
one-dimensional robust mode search avoids per-frame 3-point RANSAC while still
rejecting sparse wall and obstacle returns.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence

import numpy as np

from co_3dto2d_mapping.gravity_plane_utils import (
    PlaneFitConfig,
    PlaneFitResult,
    normalize_vector,
)


def _deterministic_sample(points: np.ndarray, maximum: int) -> np.ndarray:
    maximum = int(maximum)
    if maximum <= 0 or len(points) <= maximum:
        return points
    indices = np.linspace(0, len(points) - 1, maximum, dtype=np.int64)
    return points[indices]


def _mode_center(
    heights: np.ndarray,
    *,
    minimum_height_m: float,
    maximum_height_m: float,
    distance_threshold_m: float,
    bin_size_m: float,
    minimum_support: int,
    lowest_support_ratio: float,
    prior_height_m: Optional[float],
    prior_tolerance_m: float,
) -> Optional[float]:
    """Find the dominant horizontal surface, preferring the lowest valid one."""

    if heights.size == 0:
        return None

    search_heights = heights
    using_prior = False
    if prior_height_m is not None and math.isfinite(float(prior_height_m)):
        near_prior = np.abs(heights - float(prior_height_m)) <= prior_tolerance_m
        if int(np.count_nonzero(near_prior)) >= minimum_support:
            search_heights = heights[near_prior]
            using_prior = True

    bin_size_m = max(1e-3, float(bin_size_m))
    span = maximum_height_m - minimum_height_m
    bin_count = max(1, int(math.ceil(span / bin_size_m)))
    bin_indices = np.floor(
        (search_heights - minimum_height_m) / bin_size_m
    ).astype(np.int64)
    bin_indices = np.clip(bin_indices, 0, bin_count - 1)
    counts = np.bincount(bin_indices, minlength=bin_count).astype(np.int64)

    radius_bins = max(0, int(math.ceil(distance_threshold_m / bin_size_m)))
    if radius_bins > 0:
        support = np.convolve(
            counts,
            np.ones(2 * radius_bins + 1, dtype=np.int64),
            mode="same",
        )
    else:
        support = counts

    maximum_support = int(np.max(support)) if support.size else 0
    if maximum_support < minimum_support:
        return None

    centers = minimum_height_m + (
        np.arange(bin_count, dtype=np.float64) + 0.5
    ) * bin_size_m
    eligible_threshold = max(
        minimum_support,
        int(math.ceil(maximum_support * float(lowest_support_ratio))),
    )
    eligible = np.flatnonzero(support >= eligible_threshold)
    if eligible.size == 0:
        return None

    if using_prior:
        best_index = int(
            eligible[
                np.argmin(np.abs(centers[eligible] - float(prior_height_m)))
            ]
        )
    else:
        # Candidate points are already restricted to lie below the robot.
        # Under the single-floor assumption, the largest positive offset is the
        # lowest sufficiently supported horizontal surface, i.e. the floor.
        best_index = int(eligible[-1])
    return float(centers[best_index])


def estimate_single_floor_plane(
    points_xyz: np.ndarray,
    up_vector: Sequence[float],
    config: PlaneFitConfig,
    *,
    prior_height_m: Optional[float] = None,
    prior_tolerance_m: float = 0.20,
    bin_size_m: float = 0.025,
    lowest_support_ratio: float = 0.55,
) -> Optional[PlaneFitResult]:
    """Estimate one horizontal floor plane with a fixed gravity-aligned normal.

    The returned model follows ``normal.dot(point) + offset = 0``.  ``normal``
    is the normalized IMU-derived up vector and ``offset`` is the positive
    perpendicular distance from the local-frame origin to the floor.
    """

    points = np.asarray(points_xyz, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_xyz must have shape (N, 3)")
    points = points[np.all(np.isfinite(points), axis=1)]
    if len(points) < 3:
        return None

    up = normalize_vector(up_vector)
    if up is None:
        return None
    if not 0.0 < float(lowest_support_ratio) <= 1.0:
        raise ValueError("lowest_support_ratio must be in (0, 1]")
    if not math.isfinite(float(prior_tolerance_m)) or prior_tolerance_m < 0.0:
        raise ValueError("prior_tolerance_m must be finite and non-negative")
    if not math.isfinite(float(bin_size_m)) or bin_size_m <= 0.0:
        raise ValueError("bin_size_m must be positive and finite")

    points = _deterministic_sample(points, config.max_points)
    if len(points) < max(3, int(config.min_inliers)):
        return None

    # A floor point below the robot has positive height = -up.dot(point).
    heights = -(points @ up)
    valid = (
        np.isfinite(heights)
        & (heights >= float(config.min_height_m))
        & (heights <= float(config.max_height_m))
    )
    valid_heights = heights[valid]
    if valid_heights.size < int(config.min_inliers):
        return None

    center = _mode_center(
        valid_heights,
        minimum_height_m=float(config.min_height_m),
        maximum_height_m=float(config.max_height_m),
        distance_threshold_m=float(config.distance_threshold_m),
        bin_size_m=float(bin_size_m),
        minimum_support=max(3, int(config.min_inliers)),
        lowest_support_ratio=float(lowest_support_ratio),
        prior_height_m=prior_height_m,
        prior_tolerance_m=float(prior_tolerance_m),
    )
    if center is None:
        return None

    threshold = float(config.distance_threshold_m)
    inlier_mask = valid & (np.abs(heights - center) <= threshold)
    if not np.any(inlier_mask):
        return None
    height_m = float(np.median(heights[inlier_mask]))

    # One robust recentering step makes the result independent of histogram-bin
    # boundaries while preserving the fixed gravity-aligned normal.
    residuals = np.abs(heights - height_m)
    inlier_mask = valid & (residuals <= threshold)
    inlier_count = int(np.count_nonzero(inlier_mask))
    inlier_ratio = float(inlier_count) / float(len(points))
    if (
        inlier_count < int(config.min_inliers)
        or inlier_ratio < float(config.min_inlier_ratio)
    ):
        return None

    rmse_m = float(np.sqrt(np.mean(np.square(residuals[inlier_mask]))))
    return PlaneFitResult(
        normal=np.asarray(up, dtype=np.float64),
        offset=height_m,
        height_m=height_m,
        inlier_count=inlier_count,
        inlier_ratio=inlier_ratio,
        rmse_m=rmse_m,
        normal_deviation_rad=0.0,
    )
