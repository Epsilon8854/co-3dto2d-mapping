import math

import numpy as np

from co_3dto2d_mapping.pointcloud_registration import (
    estimate_rigid_transform,
    invert_transform,
    rotation_angle,
    rotation_rpy,
    rotation_tilt,
    transform_points,
    voxel_downsample,
    yaw_rotation_matrix,
)


def _rotation_xyz(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.asarray([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.asarray([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.asarray([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def test_estimate_rigid_transform_recovers_xyz_registration():
    rng = np.random.default_rng(7)
    source = rng.normal(size=(200, 3))
    expected_rotation = _rotation_xyz(
        math.radians(3.0), math.radians(-4.0), math.radians(18.0)
    )
    expected_translation = np.asarray([1.2, -0.7, 0.15])
    target = transform_points(source, expected_rotation, expected_translation)

    rotation, translation = estimate_rigid_transform(source, target)

    assert np.allclose(rotation, expected_rotation, atol=1e-10)
    assert np.allclose(translation, expected_translation, atol=1e-10)
    assert rotation_angle(rotation @ expected_rotation.T) < 1e-7


def test_transform_inverse_round_trip_for_xyz_points():
    points = np.asarray(
        [[0.0, 0.0, 0.0], [1.0, -2.0, 0.5], [-0.4, 0.2, 3.0]]
    )
    rotation = _rotation_xyz(
        math.radians(2.0), math.radians(5.0), math.radians(-25.0)
    )
    translation = np.asarray([2.0, 0.5, -0.3])

    inverse_rotation, inverse_translation = invert_transform(
        rotation, translation
    )
    transformed = transform_points(points, rotation, translation)
    restored = transform_points(
        transformed, inverse_rotation, inverse_translation
    )

    assert np.allclose(restored, points, atol=1e-10)


def test_yaw_rotation_and_tilt_diagnostics():
    rotation = yaw_rotation_matrix(math.radians(30.0), 3)
    roll, pitch, yaw = rotation_rpy(rotation)

    assert abs(roll) < 1e-12
    assert abs(pitch) < 1e-12
    assert math.isclose(yaw, math.radians(30.0), abs_tol=1e-12)
    assert rotation_tilt(rotation) < 1e-12


def test_voxel_downsample_keeps_xyz_dimension_and_averages():
    points = np.asarray(
        [
            [0.01, 0.01, 0.01],
            [0.02, 0.02, 0.02],
            [1.01, 1.01, 1.01],
        ]
    )

    downsampled = voxel_downsample(points, 0.1)

    assert downsampled.shape == (2, 3)
    assert any(
        np.allclose(point, np.asarray([0.015, 0.015, 0.015]))
        for point in downsampled
    )


def test_estimate_rigid_transform_remains_compatible_with_2d_points():
    source = np.asarray(
        [[-1.0, -0.5], [0.0, 1.0], [2.0, 0.5], [0.4, -1.2]]
    )
    expected_rotation = yaw_rotation_matrix(math.radians(-20.0), 2)
    expected_translation = np.asarray([0.8, -1.3])
    target = transform_points(source, expected_rotation, expected_translation)

    rotation, translation = estimate_rigid_transform(source, target)

    assert np.allclose(rotation, expected_rotation, atol=1e-10)
    assert np.allclose(translation, expected_translation, atol=1e-10)
