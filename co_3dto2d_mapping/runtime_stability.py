"""Pure-Python runtime gates shared by alignment and map fusion nodes.

The functions in this module deliberately avoid ROS imports so startup/stationary
behaviour and single-robot fallback can be unit tested without a ROS workspace.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Optional, Sequence

import numpy as np


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def angular_distance(left: float, right: float) -> float:
    return abs(normalize_angle(left - right))


@dataclass(frozen=True)
class RuntimePoseSample:
    receipt_ns: int
    stamp_ns: int
    x: float
    y: float
    yaw: float
    linear_speed_mps: float = 0.0
    angular_speed_rps: float = 0.0


@dataclass(frozen=True)
class StationarityMetrics:
    ready: bool
    stable: bool
    duration_sec: float
    center_x: float
    center_y: float
    center_yaw: float
    translation_span_m: float
    yaw_span_rad: float
    max_linear_speed_mps: float
    max_angular_speed_rps: float
    sample_count: int


def _circular_mean(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    return math.atan2(
        sum(math.sin(value) for value in values),
        sum(math.cos(value) for value in values),
    )


def stationarity_metrics(
    samples: Iterable[RuntimePoseSample],
    now_ns: int,
    window_sec: float,
    max_translation_span_m: float,
    max_yaw_span_rad: float,
    max_linear_speed_mps: float,
    max_angular_speed_rps: float,
    minimum_samples: int = 5,
) -> StationarityMetrics:
    """Measure robust pose/twist stability over a receipt-time window.

    Receipt time is used instead of message time so two computers with imperfect
    clock synchronisation cannot make the local stationarity gate oscillate.
    The center is a median/circular mean and spans are measured about it, which
    makes one timestamp or odometry spike much less influential.
    """

    window_ns = int(max(0.0, float(window_sec)) * 1e9)
    selected = [
        sample
        for sample in samples
        if int(now_ns) - int(sample.receipt_ns) <= window_ns
    ]
    selected.sort(key=lambda sample: sample.receipt_ns)
    if not selected:
        return StationarityMetrics(
            False,
            False,
            0.0,
            0.0,
            0.0,
            0.0,
            math.inf,
            math.inf,
            math.inf,
            math.inf,
            0,
        )

    duration_sec = max(
        0.0,
        (selected[-1].receipt_ns - selected[0].receipt_ns) / 1e9,
    )
    center_x = float(np.median([sample.x for sample in selected]))
    center_y = float(np.median([sample.y for sample in selected]))
    center_yaw = _circular_mean([sample.yaw for sample in selected])
    translation_span = max(
        math.hypot(sample.x - center_x, sample.y - center_y)
        for sample in selected
    )
    yaw_span = max(
        angular_distance(sample.yaw, center_yaw) for sample in selected
    )
    max_linear = max(abs(sample.linear_speed_mps) for sample in selected)
    max_angular = max(abs(sample.angular_speed_rps) for sample in selected)
    ready = (
        len(selected) >= max(2, int(minimum_samples))
        and duration_sec >= max(0.0, float(window_sec)) * 0.8
    )
    stable = (
        ready
        and translation_span <= max(0.0, float(max_translation_span_m))
        and yaw_span <= max(0.0, float(max_yaw_span_rad))
        and max_linear <= max(0.0, float(max_linear_speed_mps))
        and max_angular <= max(0.0, float(max_angular_speed_rps))
    )
    return StationarityMetrics(
        ready=ready,
        stable=stable,
        duration_sec=duration_sec,
        center_x=center_x,
        center_y=center_y,
        center_yaw=center_yaw,
        translation_span_m=translation_span,
        yaw_span_rad=yaw_span,
        max_linear_speed_mps=max_linear,
        max_angular_speed_rps=max_angular,
        sample_count=len(selected),
    )


def motion_keyframe_allowed(
    previous_x: float,
    previous_y: float,
    previous_yaw: float,
    current_x: float,
    current_y: float,
    current_yaw: float,
    recent_stationarity: Optional[StationarityMetrics],
    translation_threshold_m: float,
    yaw_threshold_rad: float,
    large_motion_factor: float = 1.75,
) -> bool:
    """Reject odometry-only keyframes while the independent motion gate is stable.

    A large displacement still passes even when twist fields are stuck at zero;
    this preserves slow-motion operation on odometry sources that do not report
    twist while blocking small stationary random walk.
    """

    translation = math.hypot(current_x - previous_x, current_y - previous_y)
    yaw = angular_distance(current_yaw, previous_yaw)
    translation_threshold = max(0.0, float(translation_threshold_m))
    yaw_threshold = max(0.0, float(yaw_threshold_rad))
    moved = translation >= translation_threshold or yaw >= yaw_threshold
    if not moved:
        return False
    if recent_stationarity is None or not recent_stationarity.stable:
        return True
    factor = max(1.0, float(large_motion_factor))
    return (
        translation >= factor * max(translation_threshold, 1e-6)
        or yaw >= factor * max(yaw_threshold, 1e-6)
    )


def choose_fusion_reference(
    alignment_available: bool,
    robot0_usable: bool,
    robot1_usable: bool,
    previous_reference: Optional[int],
    allow_robot1_single_fallback: bool = True,
) -> Optional[int]:
    """Choose the common-frame anchor without publishing a phantom r0 frame.

    Once an inter-robot alignment exists, robot 0 is the canonical map frame.
    Before that, the previous usable reference is retained to prevent repeated
    frame jumps.  If only robot 1 is running, robot 1 becomes an identity anchor.
    """

    if alignment_available:
        return 0
    usable = {0: bool(robot0_usable), 1: bool(robot1_usable)}
    if previous_reference in usable and usable[previous_reference]:
        return previous_reference
    if usable[0]:
        return 0
    if allow_robot1_single_fallback and usable[1]:
        return 1
    return None
