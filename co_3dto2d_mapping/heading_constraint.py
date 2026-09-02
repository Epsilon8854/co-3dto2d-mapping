"""Helpers for rejecting physically impossible cross-robot heading solutions."""

from __future__ import annotations

import math
from typing import Iterable, List, Optional, Tuple


RegistrationRank = Tuple[float, float, int, float]


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""

    return math.atan2(math.sin(angle), math.cos(angle))


def angular_distance(left: float, right: float) -> float:
    """Return the absolute wrapped angular distance in radians."""

    return abs(normalize_angle(left - right))


def yaw_within_prior(
    yaw: float,
    expected_yaw: float,
    max_deviation: float,
    *,
    tolerance: float = 1e-9,
) -> bool:
    """Return whether ``yaw`` is compatible with the physical heading prior."""

    if max_deviation < 0.0:
        raise ValueError("max_deviation must be non-negative")
    return angular_distance(yaw, expected_yaw) <= max_deviation + tolerance


def initial_yaw_candidates(
    expected_yaw: float,
    max_deviation: float,
    offsets: Iterable[float],
) -> List[float]:
    """Build unique initial yaw candidates, nearest to the prior first."""

    if max_deviation < 0.0:
        raise ValueError("max_deviation must be non-negative")

    candidates = [normalize_angle(expected_yaw)]
    for offset in offsets:
        candidate = normalize_angle(expected_yaw + float(offset))
        if not yaw_within_prior(candidate, expected_yaw, max_deviation):
            continue
        if any(angular_distance(candidate, existing) < 1e-9 for existing in candidates):
            continue
        candidates.append(candidate)

    candidates.sort(key=lambda value: angular_distance(value, expected_yaw))
    return candidates


def registration_rank(
    *,
    fitness: float,
    rmse: float,
    correspondences: int,
    yaw: float,
    expected_yaw: float,
    max_deviation: float,
    heading_prior_weight: float,
    enforce_prior: bool,
) -> Optional[RegistrationRank]:
    """Return a sortable rank, or ``None`` for a heading-invalid result.

    Fitness remains the dominant term. The heading prior only breaks close
    geometric ties in favor of the physically expected orientation.
    """

    if heading_prior_weight < 0.0:
        raise ValueError("heading_prior_weight must be non-negative")
    deviation = angular_distance(yaw, expected_yaw)
    if enforce_prior and deviation > max_deviation + 1e-9:
        return None

    normalization = max(max_deviation if enforce_prior else math.pi, 1e-9)
    adjusted_fitness = float(fitness) - heading_prior_weight * deviation / normalization
    return (
        adjusted_fitness,
        -float(rmse),
        int(correspondences),
        -deviation,
    )
