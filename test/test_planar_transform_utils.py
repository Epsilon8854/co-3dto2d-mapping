import math

import pytest

from co_3dto2d_mapping.planar_transform_utils import (
    compose_planar,
    invert_planar,
    transform_pose,
    world_from_source_odom,
)


def assert_pose(actual, expected, tol=1e-9):
    assert actual[0] == pytest.approx(expected[0], abs=tol)
    assert actual[1] == pytest.approx(expected[1], abs=tol)
    assert math.atan2(
        math.sin(actual[2] - expected[2]),
        math.cos(actual[2] - expected[2]),
    ) == pytest.approx(0.0, abs=tol)


def test_compose_with_inverse_returns_identity():
    transform = (2.3, -1.7, math.radians(37.0))

    assert_pose(
        compose_planar(transform, invert_planar(transform)),
        (0.0, 0.0, 0.0),
    )
    assert_pose(
        compose_planar(invert_planar(transform), transform),
        (0.0, 0.0, 0.0),
    )


def test_world_alignment_compensates_both_non_identity_odometry_poses():
    target_odom_from_target_base = (4.0, 1.0, math.radians(30.0))
    source_odom_from_source_base = (-2.0, 3.0, math.radians(-20.0))
    target_base_from_source_base = (0.8, -0.2, math.radians(5.0))

    world_from_source = world_from_source_odom(
        target_odom_from_target_base,
        target_base_from_source_base,
        source_odom_from_source_base,
    )

    source_base_in_world = transform_pose(
        world_from_source, source_odom_from_source_base
    )
    expected_source_base_in_world = transform_pose(
        target_odom_from_target_base, target_base_from_source_base
    )
    assert_pose(source_base_in_world, expected_source_base_in_world)


def test_identity_odometry_reduces_to_base_registration():
    base_match = (1.2, -0.6, math.radians(12.0))

    assert_pose(
        world_from_source_odom(
            (0.0, 0.0, 0.0), base_match, (0.0, 0.0, 0.0)
        ),
        base_match,
    )
