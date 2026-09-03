import math

from co_3dto2d_mapping.runtime_stability import (
    RuntimePoseSample,
    choose_fusion_reference,
    motion_keyframe_allowed,
    stationarity_metrics,
)


def samples(jump=False):
    result = []
    for index in range(21):
        x = 0.004 * math.sin(index)
        y = 0.003 * math.cos(index)
        if jump and index > 15:
            x += 0.4
        result.append(
            RuntimePoseSample(
                receipt_ns=index * 100_000_000,
                stamp_ns=index * 100_000_000,
                x=x,
                y=y,
                yaw=math.radians(0.2) * math.sin(index),
                linear_speed_mps=0.005,
                angular_speed_rps=math.radians(0.2),
            )
        )
    return result


def test_stationary_window_accepts_small_jitter():
    metrics = stationarity_metrics(
        samples(), 2_000_000_000, 2.0, 0.03, math.radians(1.0),
        0.03, math.radians(2.0),
    )
    assert metrics.ready and metrics.stable
    assert metrics.translation_span_m < 0.01


def test_stationary_window_rejects_real_motion():
    metrics = stationarity_metrics(
        samples(jump=True), 2_000_000_000, 2.0, 0.03,
        math.radians(1.0), 0.03, math.radians(2.0),
    )
    assert metrics.ready and not metrics.stable


def test_stationary_random_walk_does_not_create_keyframe():
    metrics = stationarity_metrics(
        samples(), 2_000_000_000, 2.0, 0.03, math.radians(1.0),
        0.03, math.radians(2.0),
    )
    assert not motion_keyframe_allowed(
        0.0, 0.0, 0.0, 1.05, 0.0, 0.0, metrics,
        translation_threshold_m=1.0,
        yaw_threshold_rad=math.radians(10.0),
        large_motion_factor=1.75,
    )
    assert motion_keyframe_allowed(
        0.0, 0.0, 0.0, 2.0, 0.0, 0.0, metrics,
        translation_threshold_m=1.0,
        yaw_threshold_rad=math.radians(10.0),
        large_motion_factor=1.75,
    )


def test_reference_selection_supports_robot1_only_but_prefers_canonical_r0():
    assert choose_fusion_reference(False, False, True, None) == 1
    assert choose_fusion_reference(False, True, True, 1) == 0
    assert choose_fusion_reference(True, True, True, 1) == 0
    assert choose_fusion_reference(False, False, False, None) is None
