import math

from co_3dto2d_mapping.alignment_utils import (
    angular_distance,
    candidate_is_consistent,
    mean_planar_candidate,
    normalize_angle,
)


def test_normalize_angle_wraps_to_shortest_representation():
    assert math.isclose(normalize_angle(3.0 * math.pi), math.pi, abs_tol=1e-12)
    assert math.isclose(normalize_angle(-3.0 * math.pi), -math.pi, abs_tol=1e-12)


def test_circular_mean_handles_wraparound():
    _, _, yaw = mean_planar_candidate(
        [(1.0, 2.0, math.radians(179.0)), (3.0, 4.0, math.radians(-179.0))]
    )
    assert math.isclose(abs(yaw), math.pi, abs_tol=math.radians(1.0))


def test_candidate_consistency_checks_translation_and_rotation():
    history = [(1.0, -2.0, math.radians(10.0))]
    assert candidate_is_consistent(
        (1.1, -2.1, math.radians(12.0)),
        history,
        max_translation_delta_m=0.2,
        max_rotation_delta_rad=math.radians(5.0),
    )
    assert not candidate_is_consistent(
        (1.4, -2.0, math.radians(12.0)),
        history,
        max_translation_delta_m=0.2,
        max_rotation_delta_rad=math.radians(5.0),
    )
    assert not candidate_is_consistent(
        (1.0, -2.0, math.radians(20.0)),
        history,
        max_translation_delta_m=0.2,
        max_rotation_delta_rad=math.radians(5.0),
    )
