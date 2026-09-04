"""Gravity-constrained ground-plane estimation utilities.

The plane model is expressed in the robot local frame as::

    normal.dot(point) + offset = 0

``normal`` is always oriented toward the IMU-derived up vector, so ``offset``
is the positive perpendicular height of the local-frame origin above the plane.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple

import numpy as np


_EPS = 1e-12


@dataclass(frozen=True)
class PlaneFitConfig:
    ransac_iterations: int = 120
    distance_threshold_m: float = 0.04
    max_normal_deviation_rad: float = math.radians(18.0)
    min_inliers: int = 80
    min_inlier_ratio: float = 0.08
    min_height_m: float = 0.05
    max_height_m: float = 2.5
    max_points: int = 4000
    lowest_plane_score_weight: float = 0.03
    random_seed: int = 7


@dataclass(frozen=True)
class PlaneFitResult:
    normal: np.ndarray
    offset: float
    height_m: float
    inlier_count: int
    inlier_ratio: float
    rmse_m: float
    normal_deviation_rad: float


def normalize_vector(vector: Sequence[float]) -> Optional[np.ndarray]:
    result = np.asarray(vector, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(result)):
        return None
    length = float(np.linalg.norm(result))
    if length <= _EPS:
        return None
    return result / length


def vector_angle(left: Sequence[float], right: Sequence[float]) -> float:
    left_unit = normalize_vector(left)
    right_unit = normalize_vector(right)
    if left_unit is None or right_unit is None:
        return math.inf
    cosine = float(np.clip(np.dot(left_unit, right_unit), -1.0, 1.0))
    return math.acos(cosine)


def imu_timestamp_is_usable(
    cloud_stamp_ns: int,
    imu_stamp_ns: int,
    timeout_sec: float,
) -> bool:
    """Reject only IMU samples that are older than the cloud beyond timeout."""

    if timeout_sec <= 0.0 or cloud_stamp_ns <= 0 or imu_stamp_ns <= 0:
        return True
    return cloud_stamp_ns - imu_stamp_ns <= int(timeout_sec * 1e9)


def normalize_quaternion(quaternion_xyzw: Sequence[float]) -> np.ndarray:
    q = np.asarray(quaternion_xyzw, dtype=np.float64).reshape(4)
    norm = float(np.linalg.norm(q))
    if not math.isfinite(norm) or norm <= _EPS:
        raise ValueError("quaternion must be finite and non-zero")
    return q / norm


def quaternion_rotate(
    quaternion_xyzw: Sequence[float], vector: Sequence[float]
) -> np.ndarray:
    """Rotate ``vector`` by an ``(x, y, z, w)`` quaternion."""

    x, y, z, w = normalize_quaternion(quaternion_xyzw)
    vx, vy, vz = np.asarray(vector, dtype=np.float64).reshape(3)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return np.asarray(
        [
            vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx),
        ],
        dtype=np.float64,
    )


def rotation_matrix_from_quaternion(
    quaternion_xyzw: Sequence[float],
) -> np.ndarray:
    """Return a 3x3 active rotation matrix from an ``(x, y, z, w)`` quaternion."""

    x, y, z, w = normalize_quaternion(quaternion_xyzw)
    return np.asarray(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def up_from_world_orientation(quaternion_xyzw: Sequence[float]) -> np.ndarray:
    """Return world +Z expressed in the oriented sensor frame.

    ROS REP-145 defines IMU orientation as the sensor frame orientation with
    respect to the world frame. Therefore world up in sensor coordinates is
    obtained by applying the inverse (conjugate) orientation.
    """

    q = np.asarray(quaternion_xyzw, dtype=np.float64).reshape(4)
    inverse = np.asarray([-q[0], -q[1], -q[2], q[3]], dtype=np.float64)
    up = normalize_vector(quaternion_rotate(inverse, (0.0, 0.0, 1.0)))
    if up is None:
        raise ValueError("could not derive up vector from orientation")
    return up


def roll_pitch_from_up(up_vector: Sequence[float]) -> Tuple[float, float]:
    """Recover ZYX roll and pitch from world-up expressed in body coordinates."""

    up = normalize_vector(up_vector)
    if up is None:
        raise ValueError("up vector must be finite and non-zero")
    roll = math.atan2(float(up[1]), float(up[2]))
    pitch = math.atan2(
        -float(up[0]), math.hypot(float(up[1]), float(up[2]))
    )
    return roll, pitch


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return a ROS-order ``(x, y, z, w)`` quaternion for ZYX Euler angles."""

    half_roll = 0.5 * roll
    half_pitch = 0.5 * pitch
    half_yaw = 0.5 * yaw
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    return np.asarray(
        [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ],
        dtype=np.float64,
    )


def yaw_from_quaternion(quaternion_xyzw: Sequence[float]) -> float:
    x, y, z, w = normalize_quaternion(quaternion_xyzw)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def blend_unit_vectors(
    previous: Sequence[float], current: Sequence[float], gain: float
) -> np.ndarray:
    gain = float(np.clip(gain, 0.0, 1.0))
    previous_unit = normalize_vector(previous)
    current_unit = normalize_vector(current)
    if previous_unit is None or current_unit is None:
        raise ValueError("vectors must be finite and non-zero")
    if float(np.dot(previous_unit, current_unit)) < 0.0:
        current_unit = -current_unit
    blended = normalize_vector((1.0 - gain) * previous_unit + gain * current_unit)
    if blended is None:
        return current_unit
    return blended


def pose_z_from_plane(
    height_m: float,
    *,
    mode: str,
    reference_z_m: float,
    z_offset_m: float,
    initial_height_m: Optional[float] = None,
    initial_pose_z_m: Optional[float] = None,
) -> float:
    if mode == "height_above_plane":
        return float(reference_z_m + height_m + z_offset_m)
    if mode == "relative_to_initial":
        if initial_height_m is None or initial_pose_z_m is None:
            raise ValueError("relative_to_initial requires initial height and pose z")
        return float(
            initial_pose_z_m + (height_m - initial_height_m) + z_offset_m
        )
    if mode == "passthrough":
        if initial_pose_z_m is None:
            raise ValueError("passthrough requires the input pose z")
        return float(initial_pose_z_m + z_offset_m)
    raise ValueError(
        "mode must be height_above_plane, relative_to_initial, or passthrough"
    )


def _tangent_basis(up: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    reference = (
        np.asarray([1.0, 0.0, 0.0], dtype=np.float64)
        if abs(float(up[2])) > 0.8
        else np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
    )
    first = normalize_vector(np.cross(reference, up))
    if first is None:
        reference = np.asarray([0.0, 1.0, 0.0], dtype=np.float64)
        first = normalize_vector(np.cross(reference, up))
    if first is None:
        raise ValueError("could not build a tangent basis")
    second = normalize_vector(np.cross(up, first))
    if second is None:
        raise ValueError("could not build a tangent basis")
    return first, second


def _refine_plane(
    points: np.ndarray, inlier_mask: np.ndarray, up: np.ndarray
) -> Optional[Tuple[np.ndarray, float]]:
    selected = points[inlier_mask]
    if len(selected) < 3:
        return None
    tangent_x, tangent_y = _tangent_basis(up)
    coordinate_x = selected @ tangent_x
    coordinate_y = selected @ tangent_y
    coordinate_up = selected @ up
    design = np.column_stack(
        (coordinate_x, coordinate_y, np.ones(len(selected), dtype=np.float64))
    )
    try:
        coefficients, _, rank, _ = np.linalg.lstsq(
            design, coordinate_up, rcond=None
        )
    except np.linalg.LinAlgError:
        return None
    if rank < 3:
        return None
    slope_x, slope_y, intercept = coefficients
    normal_raw = up - slope_x * tangent_x - slope_y * tangent_y
    normal_length = float(np.linalg.norm(normal_raw))
    if not math.isfinite(normal_length) or normal_length <= _EPS:
        return None
    normal = normal_raw / normal_length
    offset = -float(intercept) / normal_length
    if float(np.dot(normal, up)) < 0.0:
        normal = -normal
        offset = -offset
    return normal, offset


def _evaluate_plane(
    points: np.ndarray,
    normal: np.ndarray,
    offset: float,
    config: PlaneFitConfig,
    up: np.ndarray,
):
    deviation = vector_angle(normal, up)
    if deviation > config.max_normal_deviation_rad:
        return None
    height = float(offset)
    if height < config.min_height_m or height > config.max_height_m:
        return None
    residuals = np.abs(points @ normal + offset)
    mask = residuals <= config.distance_threshold_m
    count = int(np.count_nonzero(mask))
    ratio = float(count) / float(len(points)) if len(points) else 0.0
    if count < config.min_inliers or ratio < config.min_inlier_ratio:
        return None
    rmse = float(np.sqrt(np.mean(np.square(residuals[mask]))))
    return mask, height, count, ratio, rmse, deviation


def estimate_gravity_constrained_plane(
    points_xyz: np.ndarray,
    up_vector: Sequence[float],
    config: PlaneFitConfig,
) -> Optional[PlaneFitResult]:
    """Estimate a ground plane whose normal remains close to IMU-derived up."""

    points = np.asarray(points_xyz, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_xyz must have shape (N, 3)")
    points = points[np.all(np.isfinite(points), axis=1)]
    if len(points) < 3:
        return None
    up = normalize_vector(up_vector)
    if up is None:
        return None

    if config.max_points > 0 and len(points) > config.max_points:
        indices = np.linspace(
            0, len(points) - 1, config.max_points, dtype=np.int64
        )
        points = points[indices]
    if len(points) < max(3, config.min_inliers):
        return None

    rng = np.random.default_rng(config.random_seed)
    best = None
    best_rank = None
    for _ in range(max(1, int(config.ransac_iterations))):
        sample_indices = rng.choice(len(points), size=3, replace=False)
        first, second, third = points[sample_indices]
        normal = normalize_vector(np.cross(second - first, third - first))
        if normal is None:
            continue
        if float(np.dot(normal, up)) < 0.0:
            normal = -normal
        offset = -float(np.dot(normal, first))
        evaluated = _evaluate_plane(points, normal, offset, config, up)
        if evaluated is None:
            continue
        mask, height, count, ratio, rmse, deviation = evaluated
        score = float(count) + (
            config.lowest_plane_score_weight * height * float(len(points))
        )
        rank = (score, count, ratio, -rmse, -deviation)
        if best_rank is None or rank > best_rank:
            best_rank = rank
            best = (normal, offset, mask)

    if best is None:
        return None

    normal, offset, mask = best
    refined = _refine_plane(points, mask, up)
    if refined is not None:
        refined_normal, refined_offset = refined
        evaluated = _evaluate_plane(
            points, refined_normal, refined_offset, config, up
        )
        if evaluated is not None:
            normal, offset = refined_normal, refined_offset
            mask, height, count, ratio, rmse, deviation = evaluated
        else:
            evaluated = _evaluate_plane(points, normal, offset, config, up)
            if evaluated is None:
                return None
            mask, height, count, ratio, rmse, deviation = evaluated
    else:
        evaluated = _evaluate_plane(points, normal, offset, config, up)
        if evaluated is None:
            return None
        mask, height, count, ratio, rmse, deviation = evaluated

    return PlaneFitResult(
        normal=normal,
        offset=float(offset),
        height_m=float(height),
        inlier_count=int(count),
        inlier_ratio=float(ratio),
        rmse_m=float(rmse),
        normal_deviation_rad=float(deviation),
    )
