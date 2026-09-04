import numpy as np
import pytest

from co_3dto2d_mapping.gravity_plane_utils import PlaneFitConfig
from co_3dto2d_mapping.single_floor_plane import estimate_single_floor_plane


def _config(**overrides):
    values = dict(
        distance_threshold_m=0.04,
        min_inliers=80,
        min_inlier_ratio=0.06,
        min_height_m=0.10,
        max_height_m=1.50,
        max_points=5000,
    )
    values.update(overrides)
    return PlaneFitConfig(**values)


def test_single_floor_estimator_prefers_lowest_supported_horizontal_surface():
    rng = np.random.default_rng(11)
    floor_xy = rng.uniform(-4.0, 4.0, size=(900, 2))
    floor = np.column_stack(
        (floor_xy, rng.normal(-0.62, 0.008, size=len(floor_xy)))
    )
    table_xy = rng.uniform(-1.5, 1.5, size=(650, 2))
    table = np.column_stack(
        (table_xy, rng.normal(-0.28, 0.006, size=len(table_xy)))
    )
    wall = np.column_stack(
        (
            np.full(500, 2.0),
            rng.uniform(-3.0, 3.0, 500),
            rng.uniform(-1.2, -0.1, 500),
        )
    )
    outliers = rng.uniform(-4.0, 4.0, size=(250, 3))

    result = estimate_single_floor_plane(
        np.vstack((floor, table, wall, outliers)),
        [0.0, 0.0, 1.0],
        _config(),
        lowest_support_ratio=0.50,
    )

    assert result is not None
    assert result.height_m == pytest.approx(0.62, abs=0.02)
    assert result.rmse_m < 0.02
    assert result.inlier_count > 800


def test_prior_keeps_tracker_on_the_same_floor_when_a_table_becomes_denser():
    rng = np.random.default_rng(23)
    floor_xy = rng.uniform(-3.0, 3.0, size=(260, 2))
    floor = np.column_stack(
        (floor_xy, rng.normal(-0.58, 0.008, size=len(floor_xy)))
    )
    table_xy = rng.uniform(-2.0, 2.0, size=(1200, 2))
    table = np.column_stack(
        (table_xy, rng.normal(-0.24, 0.006, size=len(table_xy)))
    )

    result = estimate_single_floor_plane(
        np.vstack((floor, table)),
        [0.0, 0.0, 1.0],
        _config(min_inliers=120, min_inlier_ratio=0.05),
        prior_height_m=0.58,
        prior_tolerance_m=0.12,
    )

    assert result is not None
    assert result.height_m == pytest.approx(0.58, abs=0.02)


def test_vertical_only_structure_is_rejected():
    rng = np.random.default_rng(7)
    wall = np.column_stack(
        (
            np.full(600, 2.0),
            rng.uniform(-3.0, 3.0, 600),
            rng.uniform(-1.4, -0.1, 600),
        )
    )

    result = estimate_single_floor_plane(
        wall,
        [0.0, 0.0, 1.0],
        _config(min_inliers=100, min_inlier_ratio=0.20),
    )

    assert result is None


def test_input_validation_is_explicit():
    with pytest.raises(ValueError):
        estimate_single_floor_plane(
            np.zeros((5, 2)), [0.0, 0.0, 1.0], _config()
        )
    with pytest.raises(ValueError):
        estimate_single_floor_plane(
            np.zeros((100, 3)),
            [0.0, 0.0, 1.0],
            _config(),
            lowest_support_ratio=0.0,
        )
