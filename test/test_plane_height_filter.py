import numpy as np
import pytest

from co_3dto2d_mapping.plane_height_filter import (
    filter_points_by_fixed_sensor_height,
    filter_points_by_plane_height,
    signed_plane_heights,
)


def test_signed_height_uses_plane_normal_and_offset():
    points = np.asarray(
        [
            [0.0, 0.0, -1.0],
            [1.0, 0.0, -0.5],
            [0.0, 2.0, 0.0],
        ]
    )
    heights = signed_plane_heights(points, [0.0, 0.0, 1.0], 1.0)
    assert np.allclose(heights, [0.0, 0.5, 1.0])


def test_default_one_meter_band_removes_ground_and_tall_points():
    points = np.asarray(
        [
            [0.0, 0.0, -1.00],  # ground: 0.00 m
            [0.0, 0.0, -0.94],  # 0.06 m
            [0.0, 0.0, -0.20],  # 0.80 m
            [0.0, 0.0, 0.00],   # 1.00 m, inclusive
            [0.0, 0.0, 0.01],   # 1.01 m
        ]
    )
    result = filter_points_by_plane_height(
        points,
        [0.0, 0.0, 1.0],
        1.0,
        0.05,
        1.00,
    )
    assert np.allclose(result.points, points[[1, 2, 3]])
    assert result.mask.tolist() == [False, True, True, True, False]


def test_filter_is_rotation_invariant_in_plane_coordinates():
    normal = np.asarray([0.0, np.sqrt(0.5), np.sqrt(0.5)])
    ground = -normal
    points = np.vstack(
        [
            ground,
            ground + 0.20 * normal,
            ground + 0.95 * normal,
            ground + 1.20 * normal,
        ]
    )
    result = filter_points_by_plane_height(
        points,
        normal,
        1.0,
        0.05,
        1.0,
    )
    assert np.allclose(result.points, points[[1, 2]])


def test_invalid_height_band_is_rejected():
    with pytest.raises(ValueError):
        filter_points_by_plane_height(
            np.empty((0, 3)),
            [0.0, 0.0, 1.0],
            0.0,
            1.0,
            1.0,
        )


def test_fixed_sensor_height_keeps_the_requested_floor_relative_band():
    """A virtual floor must work even when no observed floor points exist."""

    up = np.asarray([0.0, np.sqrt(0.5), np.sqrt(0.5)])
    sensor_height_m = 0.30
    points = np.vstack(
        [
            -0.30 * up,  # virtual floor, 0.00 m above floor
            0.00 * up,  # sensor height, 0.30 m above floor
            0.90 * up,  # 1.20 m above floor
            0.91 * up,  # 1.21 m above floor
        ]
    )

    result = filter_points_by_fixed_sensor_height(
        points,
        up,
        sensor_height_m,
        0.30,
        1.20,
    )

    assert np.allclose(result.points, points[[1, 2]])
    assert result.heights_m == pytest.approx([0.0, 0.30, 1.20, 1.21])
