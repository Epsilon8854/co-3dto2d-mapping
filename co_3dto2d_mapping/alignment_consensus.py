"""Robust consensus for repeated inter-robot SE(2) map-transform estimates."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from co_3dto2d_mapping.planar_transform_utils import (
    PlanarTransform,
    normalize_angle,
)


@dataclass(frozen=True)
class AlignmentMeasurement:
    target_keyframe_id: int
    source_keyframe_id: int
    transform: PlanarTransform
    weight: float
    stamp_ns: int = 0
    descriptor_distance: float = 0.0
    rmse_m: float = 0.0
    overlap: float = 0.0
    free_conflict_ratio: float = 0.0


@dataclass(frozen=True)
class ConsensusResult:
    transform: PlanarTransform
    measurement_count: int
    target_keyframe_count: int
    source_keyframe_count: int
    total_weight: float
    translation_rms_m: float
    yaw_rms_rad: float
    accepted: bool


def _angle_distance(left: float, right: float) -> float:
    return abs(normalize_angle(left - right))


def _weighted_mean(
    measurements: Sequence[AlignmentMeasurement],
    translation_scale: float,
    yaw_scale: float,
) -> tuple[PlanarTransform, list[float]]:
    if not measurements:
        raise ValueError("at least one measurement is required")
    base_weights = [max(1e-9, float(item.weight)) for item in measurements]
    weights = list(base_weights)
    estimate = measurements[0].transform

    for _ in range(8):
        total = sum(weights)
        x = sum(weight * item.transform[0] for weight, item in zip(weights, measurements)) / total
        y = sum(weight * item.transform[1] for weight, item in zip(weights, measurements)) / total
        sin_yaw = sum(weight * math.sin(item.transform[2]) for weight, item in zip(weights, measurements))
        cos_yaw = sum(weight * math.cos(item.transform[2]) for weight, item in zip(weights, measurements))
        yaw = math.atan2(sin_yaw, cos_yaw)
        updated = (x, y, yaw)

        robust = []
        for base, item in zip(base_weights, measurements):
            translation = math.hypot(item.transform[0] - x, item.transform[1] - y)
            angular = _angle_distance(item.transform[2], yaw)
            normalized = math.sqrt(
                (translation / max(translation_scale, 1e-9)) ** 2
                + (angular / max(yaw_scale, 1e-9)) ** 2
            )
            huber = 1.0 if normalized <= 1.0 else 1.0 / normalized
            robust.append(base * huber)
        if (
            math.hypot(updated[0] - estimate[0], updated[1] - estimate[1]) < 1e-6
            and _angle_distance(updated[2], estimate[2]) < 1e-6
        ):
            estimate = updated
            weights = robust
            break
        estimate = updated
        weights = robust
    return estimate, weights


def estimate_alignment_consensus(
    measurements: Sequence[AlignmentMeasurement],
    *,
    translation_cluster_m: float,
    yaw_cluster_rad: float,
    min_measurements: int,
    min_distinct_keyframes_per_robot: int,
) -> ConsensusResult | None:
    """Return the highest-weight mutually consistent transform cluster."""

    if not measurements:
        return None
    translation_cluster_m = max(1e-9, float(translation_cluster_m))
    yaw_cluster_rad = max(1e-9, float(yaw_cluster_rad))
    min_measurements = max(1, int(min_measurements))
    min_distinct = max(1, int(min_distinct_keyframes_per_robot))

    best = None
    best_rank = None
    for seed in measurements:
        cluster = [
            item
            for item in measurements
            if math.hypot(
                item.transform[0] - seed.transform[0],
                item.transform[1] - seed.transform[1],
            )
            <= translation_cluster_m
            and _angle_distance(item.transform[2], seed.transform[2])
            <= yaw_cluster_rad
        ]
        estimate, robust_weights = _weighted_mean(
            cluster, translation_cluster_m, yaw_cluster_rad
        )
        # Re-evaluate around the robust mean to remove seed-edge inclusions.
        cluster = [
            item
            for item in cluster
            if math.hypot(
                item.transform[0] - estimate[0],
                item.transform[1] - estimate[1],
            )
            <= translation_cluster_m
            and _angle_distance(item.transform[2], estimate[2])
            <= yaw_cluster_rad
        ]
        estimate, robust_weights = _weighted_mean(
            cluster, translation_cluster_m, yaw_cluster_rad
        )
        total_weight = float(sum(robust_weights))
        target_count = len({item.target_keyframe_id for item in cluster})
        source_count = len({item.source_keyframe_id for item in cluster})
        translation_rms = math.sqrt(
            sum(
                weight
                * (
                    (item.transform[0] - estimate[0]) ** 2
                    + (item.transform[1] - estimate[1]) ** 2
                )
                for weight, item in zip(robust_weights, cluster)
            )
            / max(total_weight, 1e-9)
        )
        yaw_rms = math.sqrt(
            sum(
                weight * _angle_distance(item.transform[2], estimate[2]) ** 2
                for weight, item in zip(robust_weights, cluster)
            )
            / max(total_weight, 1e-9)
        )
        accepted = (
            len(cluster) >= min_measurements
            and target_count >= min_distinct
            and source_count >= min_distinct
        )
        result = ConsensusResult(
            transform=estimate,
            measurement_count=len(cluster),
            target_keyframe_count=target_count,
            source_keyframe_count=source_count,
            total_weight=total_weight,
            translation_rms_m=translation_rms,
            yaw_rms_rad=yaw_rms,
            accepted=accepted,
        )
        rank = (
            int(accepted),
            min(target_count, source_count),
            len(cluster),
            total_weight,
            -translation_rms,
            -yaw_rms,
        )
        if best_rank is None or rank > best_rank:
            best_rank = rank
            best = result
    return best
