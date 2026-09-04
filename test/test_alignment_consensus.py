import math

import pytest

from co_3dto2d_mapping.alignment_consensus import (
    AlignmentMeasurement,
    estimate_alignment_consensus,
)


def test_pcm_clique_rejects_weighted_outlier():
    good = [
        AlignmentMeasurement(0, 0, (1.00, -0.50, math.radians(5.0)), 1.0),
        AlignmentMeasurement(1, 0, (1.04, -0.48, math.radians(5.4)), 0.9),
        AlignmentMeasurement(1, 1, (0.98, -0.53, math.radians(4.7)), 1.1),
        AlignmentMeasurement(2, 2, (1.02, -0.51, math.radians(5.1)), 1.0),
    ]
    measurements = good + [
        AlignmentMeasurement(9, 9, (-4.0, 3.0, math.radians(90.0)), 50.0)
    ]

    result = estimate_alignment_consensus(
        measurements,
        translation_cluster_m=0.25,
        yaw_cluster_rad=math.radians(4.0),
        min_measurements=3,
        min_distinct_keyframes_per_robot=2,
    )

    assert result is not None and result.accepted
    assert result.measurement_count == 4
    assert result.transform[0] == pytest.approx(1.01, abs=0.05)
    assert result.transform[1] == pytest.approx(-0.505, abs=0.05)
    assert result.target_keyframe_count == 3
    assert result.source_keyframe_count == 3


def test_pcm_does_not_accept_a_transitive_but_inconsistent_chain():
    # A-B and B-C pass the 0.15 m gate, but A-C does not. A connected-component
    # method would incorrectly count all three; a pairwise clique may count two.
    measurements = [
        AlignmentMeasurement(0, 0, (0.00, 0.0, 0.0), 1.0),
        AlignmentMeasurement(1, 1, (0.10, 0.0, 0.0), 1.0),
        AlignmentMeasurement(2, 2, (0.20, 0.0, 0.0), 1.0),
    ]
    result = estimate_alignment_consensus(
        measurements,
        translation_cluster_m=0.15,
        yaw_cluster_rad=math.radians(2.0),
        min_measurements=3,
        min_distinct_keyframes_per_robot=2,
    )

    assert result is not None
    assert result.measurement_count == 2
    assert not result.accepted


def test_consensus_requires_distinct_keyframes_from_both_robots():
    measurements = [
        AlignmentMeasurement(index, 0, (0.2, 0.1, 0.0), 1.0)
        for index in range(4)
    ]
    result = estimate_alignment_consensus(
        measurements,
        translation_cluster_m=0.2,
        yaw_cluster_rad=math.radians(3.0),
        min_measurements=3,
        min_distinct_keyframes_per_robot=2,
    )

    assert result is not None
    assert not result.accepted
    assert result.source_keyframe_count == 1


def test_yaw_consistency_wraps_across_pi():
    measurements = [
        AlignmentMeasurement(0, 0, (0.0, 0.0, math.radians(179.0)), 1.0),
        AlignmentMeasurement(1, 1, (0.0, 0.0, math.radians(-179.5)), 1.0),
        AlignmentMeasurement(2, 2, (0.0, 0.0, math.radians(178.5)), 1.0),
    ]
    result = estimate_alignment_consensus(
        measurements,
        translation_cluster_m=0.1,
        yaw_cluster_rad=math.radians(3.0),
        min_measurements=3,
        min_distinct_keyframes_per_robot=2,
    )

    assert result is not None and result.accepted
    assert abs(abs(result.transform[2]) - math.pi) < math.radians(2.0)
