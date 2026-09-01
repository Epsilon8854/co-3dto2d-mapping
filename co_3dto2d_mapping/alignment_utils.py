import math
from typing import Sequence, Tuple


PlanarCandidate = Tuple[float, float, float]


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    return math.atan2(math.sin(angle), math.cos(angle))


def angular_distance(left: float, right: float) -> float:
    """Return the absolute shortest angular distance between two angles."""
    return abs(normalize_angle(left - right))


def mean_planar_candidate(candidates: Sequence[PlanarCandidate]) -> PlanarCandidate:
    if not candidates:
        raise ValueError("at least one planar candidate is required")

    count = float(len(candidates))
    x = sum(candidate[0] for candidate in candidates) / count
    y = sum(candidate[1] for candidate in candidates) / count
    sin_yaw = sum(math.sin(candidate[2]) for candidate in candidates)
    cos_yaw = sum(math.cos(candidate[2]) for candidate in candidates)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return x, y, yaw


def candidate_is_consistent(
    candidate: PlanarCandidate,
    history: Sequence[PlanarCandidate],
    max_translation_delta_m: float,
    max_rotation_delta_rad: float,
) -> bool:
    if not history:
        return True

    reference = mean_planar_candidate(history)
    translation_delta = math.hypot(
        candidate[0] - reference[0],
        candidate[1] - reference[1],
    )
    rotation_delta = angular_distance(candidate[2], reference[2])
    return (
        translation_delta <= max_translation_delta_m
        and rotation_delta <= max_rotation_delta_rad
    )
