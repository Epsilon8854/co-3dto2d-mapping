import math

import numpy as np
import pytest

from co_3dto2d_mapping.gravity_plane_utils import (
    PlaneFitConfig,
    blend_unit_vectors,
    estimate_gravity_constrained_plane,
    pose_z_from_plane,
    quaternion_from_rpy,
    roll_pitch_from_up,
    rotation_matrix_from_quaternion,
    up_from_world_orientation,
    vector_angle,
)


def _world_from_body(roll: float, pitch: float, yaw: float) -> np.ndarray:
    return rotation_matrix_from_quaternion(quaternion_from_rpy(roll, pitch, yaw))


def test_imu_orientation_recovers_world_up_in_sensor_frame() -> None:
    roll = math.radians(11.0)
    pitch = math.radians(-7.0)
    yaw = math.radians(63.0)
    orientation = quaternion_from_rpy(roll, pitch, yaw)
    expected = _world_from_body(roll, pitch, yaw).T @ np.asarray([0.0, 0.0, 1.0])
    measured = up_from_world_orientation(orientation)
    assert np.allclose(measured, expected, atol=1e-9)


def test_roll_pitch_are_recovered_from_plane_normal_independent_of_yaw() -> None:
    roll = math.radians(8.0)
    pitch = math.radians(-5.0)
    for yaw in (0.0, math.radians(90.0), math.radians(-140.0)):
        up_body = _world_from_body(roll, pitch, yaw).T @ np.asarray([0.0, 0.0, 1.0])
        recovered_roll, recovered_pitch = roll_pitch_from_up(up_body)
        assert recovered_roll == pytest.approx(roll, abs=1e-9)
        assert recovered_pitch == pytest.approx(pitch, abs=1e-9)


def test_gravity_constrained_ransac_selects_floor_over_outliers() -> None:
    rng = np.random.default_rng(41)
    roll = math.radians(6.0)
    pitch = math.radians(-4.0)
    up = _world_from_body(roll, pitch, math.radians(30.0)).T @ np.asarray([0.0, 0.0, 1.0])
    tangent_x = np.cross([1.0, 0.0, 0.0], up)
    tangent_x /= np.linalg.norm(tangent_x)
    tangent_y = np.cross(up, tangent_x)
    height = 0.58
    coordinates = rng.uniform(-3.0, 3.0, size=(1200, 2))
    floor = (
        coordinates[:, :1] * tangent_x
        + coordinates[:, 1:] * tangent_y
        - height * up
    )
    floor += rng.normal(0.0, 0.008, size=(len(floor), 1)) * up
    vertical_wall = np.column_stack(
        (
            np.full(350, 2.0),
            rng.uniform(-2.5, 2.5, 350),
            rng.uniform(-1.5, 1.5, 350),
        )
    )
    random_outliers = rng.uniform(-3.0, 3.0, size=(200, 3))
    points = np.vstack((floor, vertical_wall, random_outliers))

    result = estimate_gravity_constrained_plane(
        points,
        up,
        PlaneFitConfig(
            ransac_iterations=160,
            distance_threshold_m=0.035,
            max_normal_deviation_rad=math.radians(15.0),
            min_inliers=300,
            min_inlier_ratio=0.35,
            min_height_m=0.2,
            max_height_m=1.2,
            random_seed=9,
        ),
    )
    assert result is not None
    assert result.height_m == pytest.approx(height, abs=0.02)
    assert vector_angle(result.normal, up) < math.radians(0.5)
    assert result.inlier_count > 1100
    assert result.rmse_m < 0.015


def test_vertical_plane_is_rejected_by_gravity_constraint() -> None:
    rng = np.random.default_rng(8)
    wall = np.column_stack(
        (
            np.full(500, 1.5),
            rng.uniform(-2.0, 2.0, 500),
            rng.uniform(-1.0, 1.0, 500),
        )
    )
    result = estimate_gravity_constrained_plane(
        wall,
        [0.0, 0.0, 1.0],
        PlaneFitConfig(
            min_inliers=100,
            min_inlier_ratio=0.2,
            max_normal_deviation_rad=math.radians(10.0),
        ),
    )
    assert result is None


def test_height_to_z_modes_are_explicit() -> None:
    assert pose_z_from_plane(
        0.6,
        mode="height_above_plane",
        reference_z_m=0.1,
        z_offset_m=0.02,
    ) == pytest.approx(0.72)
    assert pose_z_from_plane(
        0.7,
        mode="relative_to_initial",
        reference_z_m=0.0,
        z_offset_m=0.0,
        initial_height_m=0.5,
        initial_pose_z_m=0.0,
    ) == pytest.approx(0.2)


def test_unit_vector_filter_does_not_flip_normal_sign() -> None:
    blended = blend_unit_vectors([0.0, 0.0, 1.0], [0.0, 0.0, -1.0], 0.5)
    assert np.allclose(blended, [0.0, 0.0, 1.0])
