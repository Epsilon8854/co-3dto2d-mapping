"""Pairwise-consistent consensus for fixed inter-robot SE(2) alignment.

Every geometrically verified place pair estimates the same map-to-map transform.
We therefore form a pairwise-consistency graph: two measurements share an edge
only when both translation and yaw differences satisfy the configured gates.
A PCM-style maximal-clique search selects a mutually consistent measurement set,
then a robust weighted mean produces the fixed transform.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence

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


def _consistent(
    left: AlignmentMeasurement,
    right: AlignmentMeasurement,
    translation_gate_m: float,
    yaw_gate_rad: float,
) -> bool:
    return (
        math.hypot(
            left.transform[0] - right.transform[0],
            left.transform[1] - right.transform[1],
        )
        <= translation_gate_m
        and _angle_distance(left.transform[2], right.transform[2])
        <= yaw_gate_rad
    )


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

    # Iteratively reweighted mean limits the influence of a measurement near a
    # clique boundary without violating the pairwise-consistency requirement.
    for _ in range(8):
        total = max(sum(weights), 1e-9)
        x = sum(
            weight * item.transform[0]
            for weight, item in zip(weights, measurements)
        ) / total
        y = sum(
            weight * item.transform[1]
            for weight, item in zip(weights, measurements)
        ) / total
        sin_yaw = sum(
            weight * math.sin(item.transform[2])
            for weight, item in zip(weights, measurements)
        )
        cos_yaw = sum(
            weight * math.cos(item.transform[2])
            for weight, item in zip(weights, measurements)
        )
        yaw = math.atan2(sin_yaw, cos_yaw)
        updated = (x, y, yaw)

        robust_weights = []
        for base, item in zip(base_weights, measurements):
            translation = math.hypot(
                item.transform[0] - x,
                item.transform[1] - y,
            )
            angular = _angle_distance(item.transform[2], yaw)
            normalized = math.sqrt(
                (translation / max(translation_scale, 1e-9)) ** 2
                + (angular / max(yaw_scale, 1e-9)) ** 2
            )
            huber = 1.0 if normalized <= 1.0 else 1.0 / normalized
            robust_weights.append(base * huber)

        converged = (
            math.hypot(updated[0] - estimate[0], updated[1] - estimate[1])
            < 1e-6
            and _angle_distance(updated[2], estimate[2]) < 1e-6
        )
        estimate = updated
        weights = robust_weights
        if converged:
            break
    return estimate, weights


def _popcount(mask: int) -> int:
    # int.bit_count() is unavailable in the Python 3.8 used by ROS 2 Foxy.
    return bin(mask).count("1")


def _indices(mask: int) -> Iterable[int]:
    while mask:
        bit = mask & -mask
        yield bit.bit_length() - 1
        mask ^= bit


def _connected_components(adjacency: Sequence[int]) -> list[int]:
    unseen = (1 << len(adjacency)) - 1
    components: list[int] = []
    while unseen:
        seed = unseen & -unseen
        frontier = seed
        component = 0
        while frontier:
            component |= frontier
            unseen &= ~frontier
            neighbours = 0
            for index in _indices(frontier):
                neighbours |= adjacency[index]
            frontier = neighbours & unseen
        components.append(component)
    return components


def _greedy_cliques(
    component: int,
    adjacency: Sequence[int],
    measurements: Sequence[AlignmentMeasurement],
) -> list[int]:
    """Deterministic bounded fallback if maximal-clique enumeration explodes."""

    vertices = sorted(
        _indices(component),
        key=lambda index: (
            -max(0.0, float(measurements[index].weight)),
            -_popcount(adjacency[index] & component),
            index,
        ),
    )
    cliques: list[int] = []
    for seed in vertices:
        clique = 1 << seed
        candidates = component & adjacency[seed]
        for candidate in vertices:
            bit = 1 << candidate
            if not candidates & bit:
                continue
            if all(
                adjacency[candidate] & (1 << member)
                for member in _indices(clique)
            ):
                clique |= bit
                candidates &= adjacency[candidate]
        cliques.append(clique)
    return cliques


def _maximal_cliques(
    component: int,
    adjacency: Sequence[int],
    measurements: Sequence[AlignmentMeasurement],
    *,
    search_node_limit: int = 200_000,
) -> list[int]:
    """Enumerate maximal cliques with Bron-Kerbosch and a deterministic pivot."""

    cliques: list[int] = []
    visited = 0
    aborted = False

    def visit(clique: int, candidates: int, excluded: int) -> None:
        nonlocal visited, aborted
        if aborted:
            return
        visited += 1
        if visited > search_node_limit:
            aborted = True
            return
        if candidates == 0 and excluded == 0:
            cliques.append(clique)
            return

        union = candidates | excluded
        if union:
            pivot = max(
                _indices(union),
                key=lambda index: (
                    _popcount(candidates & adjacency[index]),
                    max(0.0, float(measurements[index].weight)),
                    -index,
                ),
            )
            extension = candidates & ~adjacency[pivot]
        else:
            extension = candidates

        ordered = sorted(
            _indices(extension),
            key=lambda index: (
                -max(0.0, float(measurements[index].weight)),
                -_popcount(adjacency[index] & candidates),
                index,
            ),
        )
        for vertex in ordered:
            bit = 1 << vertex
            visit(
                clique | bit,
                candidates & adjacency[vertex],
                excluded & adjacency[vertex],
            )
            candidates &= ~bit
            excluded |= bit
            if aborted:
                return

    visit(0, component, 0)
    if aborted or not cliques:
        return _greedy_cliques(component, adjacency, measurements)
    return cliques


def _result_for_clique(
    clique: int,
    measurements: Sequence[AlignmentMeasurement],
    *,
    translation_scale: float,
    yaw_scale: float,
    min_measurements: int,
    min_distinct: int,
) -> ConsensusResult:
    cluster = [measurements[index] for index in _indices(clique)]
    estimate, robust_weights = _weighted_mean(
        cluster,
        translation_scale,
        yaw_scale,
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
    return ConsensusResult(
        transform=estimate,
        measurement_count=len(cluster),
        target_keyframe_count=target_count,
        source_keyframe_count=source_count,
        total_weight=total_weight,
        translation_rms_m=translation_rms,
        yaw_rms_rad=yaw_rms,
        accepted=(
            len(cluster) >= min_measurements
            and target_count >= min_distinct
            and source_count >= min_distinct
        ),
    )


def estimate_alignment_consensus(
    measurements: Sequence[AlignmentMeasurement],
    *,
    translation_cluster_m: float,
    yaw_cluster_rad: float,
    min_measurements: int,
    min_distinct_keyframes_per_robot: int,
) -> ConsensusResult | None:
    """Return the best mutually pairwise-consistent fixed-transform estimate."""

    if not measurements:
        return None
    translation_cluster_m = max(1e-9, float(translation_cluster_m))
    yaw_cluster_rad = max(1e-9, float(yaw_cluster_rad))
    min_measurements = max(1, int(min_measurements))
    min_distinct = max(1, int(min_distinct_keyframes_per_robot))

    count = len(measurements)
    adjacency = [0] * count
    for left_index in range(count):
        for right_index in range(left_index + 1, count):
            if _consistent(
                measurements[left_index],
                measurements[right_index],
                translation_cluster_m,
                yaw_cluster_rad,
            ):
                adjacency[left_index] |= 1 << right_index
                adjacency[right_index] |= 1 << left_index

    best: ConsensusResult | None = None
    best_rank = None
    for component in _connected_components(adjacency):
        for clique in _maximal_cliques(component, adjacency, measurements):
            result = _result_for_clique(
                clique,
                measurements,
                translation_scale=translation_cluster_m,
                yaw_scale=yaw_cluster_rad,
                min_measurements=min_measurements,
                min_distinct=min_distinct,
            )
            rank = (
                int(result.accepted),
                min(
                    result.target_keyframe_count,
                    result.source_keyframe_count,
                ),
                result.measurement_count,
                result.total_weight,
                -result.translation_rms_m,
                -result.yaw_rms_rad,
            )
            if best_rank is None or rank > best_rank:
                best_rank = rank
                best = result
    return best
