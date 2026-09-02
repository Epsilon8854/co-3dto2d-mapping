"""Numerical helpers for rigid point-cloud registration.

The cross-robot aligner uses full XYZ correspondences while the published map
alignment remains planar.  These helpers intentionally have no ROS dependency
so the registration math can be regression-tested in isolation.
"""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np


def _as_points(points: np.ndarray, *, name: str) -> np.ndarray:
    array = np.asarray(points, dtype=np.float64)
    if array.ndim != 2 or array.shape[1] not in (2, 3):
        raise ValueError("%s must have shape (N, 2) or (N, 3)" % name)
    if len(array) == 0:
        return array
    if not np.all(np.isfinite(array)):
        raise ValueError("%s contains non-finite coordinates" % name)
    return array


def estimate_rigid_transform(
    source: np.ndarray,
    target: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Estimate the least-squares rigid transform from matched points.

    Points are represented as row vectors and transformed with
    ``source @ rotation.T + translation``.  Both 2D and 3D correspondences are
    supported.
    """

    source_points = _as_points(source, name="source")
    target_points = _as_points(target, name="target")
    if source_points.shape != target_points.shape:
        raise ValueError("source and target correspondences must have equal shape")
    if len(source_points) < source_points.shape[1]:
        raise ValueError("not enough correspondences to estimate a rigid transform")

    source_mean = np.mean(source_points, axis=0)
    target_mean = np.mean(target_points, axis=0)
    source_centered = source_points - source_mean
    target_centered = target_points - target_mean

    u, _, vt = np.linalg.svd(source_centered.T @ target_centered)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T

    translation = target_mean - rotation @ source_mean
    return rotation, translation


def yaw_rotation_matrix(yaw: float, dimension: int) -> np.ndarray:
    """Return a 2D rotation or a 3D rotation about +Z."""

    if dimension not in (2, 3):
        raise ValueError("dimension must be 2 or 3")
    cos_yaw = math.cos(float(yaw))
    sin_yaw = math.sin(float(yaw))
    rotation = np.eye(dimension, dtype=np.float64)
    rotation[0, 0] = cos_yaw
    rotation[0, 1] = -sin_yaw
    rotation[1, 0] = sin_yaw
    rotation[1, 1] = cos_yaw
    return rotation


def transform_points(
    points: np.ndarray,
    rotation: np.ndarray,
    translation: np.ndarray,
) -> np.ndarray:
    """Apply a rigid transform to row-vector points."""

    point_array = _as_points(points, name="points")
    rotation_array = np.asarray(rotation, dtype=np.float64)
    translation_array = np.asarray(translation, dtype=np.float64)
    dimension = point_array.shape[1]
    if rotation_array.shape != (dimension, dimension):
        raise ValueError("rotation shape does not match point dimension")
    if translation_array.shape != (dimension,):
        raise ValueError("translation shape does not match point dimension")
    return point_array @ rotation_array.T + translation_array


def invert_transform(
    rotation: np.ndarray,
    translation: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Invert a rigid transform represented by rotation and translation."""

    rotation_array = np.asarray(rotation, dtype=np.float64)
    translation_array = np.asarray(translation, dtype=np.float64)
    if rotation_array.ndim != 2 or rotation_array.shape[0] != rotation_array.shape[1]:
        raise ValueError("rotation must be square")
    if translation_array.shape != (rotation_array.shape[0],):
        raise ValueError("translation shape does not match rotation")
    inverse_rotation = rotation_array.T
    inverse_translation = -(inverse_rotation @ translation_array)
    return inverse_rotation, inverse_translation


def rotation_yaw(rotation: np.ndarray) -> float:
    """Extract ZYX yaw from a 2D or 3D rotation matrix."""

    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape not in ((2, 2), (3, 3)):
        raise ValueError("rotation must be 2x2 or 3x3")
    return math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))


def rotation_angle(rotation: np.ndarray) -> float:
    """Return the unsigned angle represented by a 2D or 3D rotation."""

    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape == (2, 2):
        return abs(rotation_yaw(matrix))
    if matrix.shape != (3, 3):
        raise ValueError("rotation must be 2x2 or 3x3")
    cosine = float(np.clip((np.trace(matrix) - 1.0) * 0.5, -1.0, 1.0))
    return math.acos(cosine)


def rotation_tilt(rotation: np.ndarray) -> float:
    """Return the angle between transformed +Z and +Z.

    For 2D rotations the tilt is always zero.
    """

    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape == (2, 2):
        return 0.0
    if matrix.shape != (3, 3):
        raise ValueError("rotation must be 2x2 or 3x3")
    cosine = float(np.clip(matrix[2, 2], -1.0, 1.0))
    return math.acos(cosine)


def rotation_rpy(rotation: np.ndarray) -> Tuple[float, float, float]:
    """Return roll, pitch, yaw using the ZYX convention."""

    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape == (2, 2):
        return 0.0, 0.0, rotation_yaw(matrix)
    if matrix.shape != (3, 3):
        raise ValueError("rotation must be 2x2 or 3x3")

    pitch = math.asin(float(np.clip(-matrix[2, 0], -1.0, 1.0)))
    cos_pitch = math.cos(pitch)
    if abs(cos_pitch) > 1e-9:
        roll = math.atan2(float(matrix[2, 1]), float(matrix[2, 2]))
        yaw = math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))
    else:
        roll = 0.0
        yaw = math.atan2(float(-matrix[0, 1]), float(matrix[1, 1]))
    return roll, pitch, yaw


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Average points falling in the same 2D or 3D voxel."""

    point_array = _as_points(points, name="points")
    if voxel_size <= 0.0 or len(point_array) == 0:
        return point_array
    keys = np.floor(point_array / float(voxel_size)).astype(np.int64)
    unique_keys, inverse = np.unique(keys, axis=0, return_inverse=True)
    sums = np.zeros((len(unique_keys), point_array.shape[1]), dtype=np.float64)
    counts = np.zeros(len(unique_keys), dtype=np.float64)
    np.add.at(sums, inverse, point_array)
    np.add.at(counts, inverse, 1.0)
    return sums / counts[:, None]
