import math

from co_3dto2d_mapping.heading_constraint import (
    angular_distance,
    initial_yaw_candidates,
    registration_rank,
    yaw_within_prior,
)


def test_opposite_heading_is_rejected_by_default_hemisphere_prior():
    assert yaw_within_prior(0.4, 0.0, math.pi / 2.0)
    assert not yaw_within_prior(math.pi, 0.0, math.pi / 2.0)
    assert not yaw_within_prior(-math.pi, 0.0, math.pi / 2.0)


def test_heading_prior_handles_wraparound():
    expected = math.radians(179.0)
    candidate = math.radians(-179.0)
    assert math.isclose(angular_distance(candidate, expected), math.radians(2.0))
    assert yaw_within_prior(candidate, expected, math.radians(5.0))


def test_initial_candidates_stay_inside_allowed_heading_region():
    expected = math.radians(10.0)
    limit = math.radians(45.0)
    candidates = initial_yaw_candidates(
        expected,
        limit,
        [0.0, math.radians(-30.0), math.radians(30.0), math.pi],
    )
    assert math.isclose(candidates[0], expected)
    assert len(candidates) == 3
    assert all(yaw_within_prior(value, expected, limit) for value in candidates)


def test_higher_fitness_upside_down_result_cannot_win():
    valid = registration_rank(
        fitness=0.62,
        rmse=0.18,
        correspondences=500,
        yaw=math.radians(8.0),
        expected_yaw=0.0,
        max_deviation=math.pi / 2.0,
        heading_prior_weight=0.05,
        enforce_prior=True,
    )
    upside_down = registration_rank(
        fitness=0.95,
        rmse=0.04,
        correspondences=900,
        yaw=math.pi,
        expected_yaw=0.0,
        max_deviation=math.pi / 2.0,
        heading_prior_weight=0.05,
        enforce_prior=True,
    )
    assert valid is not None
    assert upside_down is None
