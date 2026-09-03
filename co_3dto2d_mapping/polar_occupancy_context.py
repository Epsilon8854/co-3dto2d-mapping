"""Rotation-aware polar place descriptors for 2-D occupancy submaps.

This is a CPU-only adaptation of the Scan Context retrieval idea. Instead of
maximum point height, each ring-sector bin stores occupied-boundary density,
free-space ratio and observed-area ratio. The ring key is rotation invariant;
the full descriptor is circularly shifted to estimate relative yaw.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Tuple

import numpy as np

from co_3dto2d_mapping.occupancy_submap import LocalOccupancyPatch
from co_3dto2d_mapping.planar_transform_utils import normalize_angle


@dataclass(frozen=True)
class PolarContextConfig:
    max_radius_m: float = 15.0
    num_rings: int = 20
    num_sectors: int = 60
    occupied_channel_weight: float = 2.0
    free_channel_weight: float = 1.0
    minimum_common_weight: float = 1.0
    valid_difference_weight: float = 0.10
    common_area_penalty_weight: float = 0.10
    recenter_on_occupied_centroid: bool = False

    def validated(self) -> "PolarContextConfig":
        max_radius_m = float(self.max_radius_m)
        num_rings = int(self.num_rings)
        num_sectors = int(self.num_sectors)
        if not np.isfinite(max_radius_m) or max_radius_m <= 0.0:
            raise ValueError("max_radius_m must be positive and finite")
        if num_rings < 2 or num_sectors < 8:
            raise ValueError(
                "polar descriptor needs at least 2 rings and 8 sectors"
            )
        weights = (
            self.occupied_channel_weight,
            self.free_channel_weight,
            self.minimum_common_weight,
            self.valid_difference_weight,
            self.common_area_penalty_weight,
        )
        if not all(
            np.isfinite(value) and value >= 0.0 for value in weights
        ):
            raise ValueError(
                "descriptor weights must be finite and non-negative"
            )
        return PolarContextConfig(
            max_radius_m=max_radius_m,
            num_rings=num_rings,
            num_sectors=num_sectors,
            occupied_channel_weight=float(self.occupied_channel_weight),
            free_channel_weight=float(self.free_channel_weight),
            minimum_common_weight=float(self.minimum_common_weight),
            valid_difference_weight=float(self.valid_difference_weight),
            common_area_penalty_weight=float(
                self.common_area_penalty_weight
            ),
            recenter_on_occupied_centroid=bool(
                self.recenter_on_occupied_centroid
            ),
        )


@dataclass(frozen=True)
class PolarContext:
    descriptor: np.ndarray
    ring_key: np.ndarray

    @property
    def num_sectors(self) -> int:
        return int(self.descriptor.shape[1])


@dataclass(frozen=True)
class ContextMatch:
    distance: float
    yaw_rad: float
    sector_shift: int
    common_weight: float


def _polar_bins(
    points: np.ndarray, config: PolarContextConfig
) -> Tuple[np.ndarray, np.ndarray]:
    values = np.asarray(points, dtype=np.float64)
    if values.ndim != 2 or values.shape[1:] != (2,):
        raise ValueError("points must have shape (N, 2)")
    if not len(values):
        empty = np.empty(0, dtype=np.int64)
        return empty, empty.copy()
    radius = np.hypot(values[:, 0], values[:, 1])
    angle = np.mod(
        np.arctan2(values[:, 1], values[:, 0]), 2.0 * math.pi
    )
    valid = (radius >= 0.0) & (radius < config.max_radius_m)
    ring = np.floor(
        radius[valid] / config.max_radius_m * config.num_rings
    ).astype(np.int64)
    sector = np.floor(
        angle[valid] / (2.0 * math.pi) * config.num_sectors
    ).astype(np.int64)
    return ring, sector


def _counts(points: np.ndarray, config: PolarContextConfig) -> np.ndarray:
    result = np.zeros(
        (config.num_rings, config.num_sectors), dtype=np.float64
    )
    ring, sector = _polar_bins(points, config)
    if ring.size:
        np.add.at(result, (ring, sector), 1.0)
    return result


def build_polar_context(
    patch: LocalOccupancyPatch,
    config: PolarContextConfig = PolarContextConfig(),
) -> PolarContext:
    config = config.validated()
    if patch.resolution <= 0.0:
        raise ValueError("patch resolution must be positive")

    center = np.zeros(2, dtype=np.float64)
    if config.recenter_on_occupied_centroid and len(patch.boundary_points):
        center = np.median(np.asarray(patch.boundary_points), axis=0)
    occupied_counts = _counts(
        np.asarray(patch.boundary_points) - center, config
    )
    free_counts = _counts(np.asarray(patch.free_points) - center, config)
    known_counts = _counts(np.asarray(patch.known_points) - center, config)

    ring_edges = np.linspace(
        0.0, config.max_radius_m, config.num_rings + 1
    )
    ring_area = math.pi * (
        ring_edges[1:] ** 2 - ring_edges[:-1] ** 2
    )
    expected_cells = (
        ring_area[:, None]
        / float(config.num_sectors)
        / float(patch.resolution * patch.resolution)
    )
    expected_cells = np.maximum(expected_cells, 1.0)

    occupied_density = np.log1p(occupied_counts) / np.log1p(
        expected_cells
    )
    occupied_density = np.clip(occupied_density, 0.0, 1.0)
    free_ratio = np.divide(
        free_counts,
        known_counts,
        out=np.zeros_like(free_counts),
        where=known_counts > 0.0,
    )
    valid_ratio = np.clip(known_counts / expected_cells, 0.0, 1.0)
    descriptor = np.stack(
        (occupied_density, free_ratio, valid_ratio), axis=-1
    ).astype(np.float32)

    valid_sum = np.sum(valid_ratio, axis=1)
    occupied_ring = np.divide(
        np.sum(occupied_density * valid_ratio, axis=1),
        valid_sum,
        out=np.zeros(config.num_rings, dtype=np.float64),
        where=valid_sum > 0.0,
    )
    free_ring = np.divide(
        np.sum(free_ratio * valid_ratio, axis=1),
        valid_sum,
        out=np.zeros(config.num_rings, dtype=np.float64),
        where=valid_sum > 0.0,
    )
    observed_ring = np.mean(valid_ratio, axis=1)
    ring_key = np.stack(
        (occupied_ring, free_ring, observed_ring), axis=-1
    )
    return PolarContext(
        descriptor=descriptor,
        ring_key=ring_key.astype(np.float32).reshape(-1),
    )


def ring_key_distance(left: PolarContext, right: PolarContext) -> float:
    left_key = np.asarray(left.ring_key, dtype=np.float64).reshape(-1)
    right_key = np.asarray(right.ring_key, dtype=np.float64).reshape(-1)
    if left_key.shape != right_key.shape:
        raise ValueError("ring keys must have the same shape")
    return float(
        np.linalg.norm(left_key - right_key)
        / math.sqrt(max(1, left_key.size))
    )


def match_polar_context(
    target: PolarContext,
    source: PolarContext,
    config: PolarContextConfig = PolarContextConfig(),
) -> ContextMatch:
    """Match ``source`` to ``target`` and estimate target-from-source yaw."""

    config = config.validated()
    target_descriptor = np.asarray(target.descriptor, dtype=np.float64)
    source_descriptor = np.asarray(source.descriptor, dtype=np.float64)
    if target_descriptor.shape != source_descriptor.shape:
        raise ValueError("polar descriptors must have the same shape")
    if target_descriptor.shape != (
        config.num_rings,
        config.num_sectors,
        3,
    ):
        raise ValueError(
            "descriptor shape does not match the supplied config"
        )

    channel_scale = np.asarray(
        [config.occupied_channel_weight, config.free_channel_weight],
        dtype=np.float64,
    )
    target_features = target_descriptor[..., :2] * channel_scale
    target_valid = target_descriptor[..., 2]
    best = ContextMatch(
        distance=float("inf"),
        yaw_rad=0.0,
        sector_shift=0,
        common_weight=0.0,
    )

    for shift in range(config.num_sectors):
        shifted = np.roll(source_descriptor, shift=shift, axis=1)
        source_features = shifted[..., :2] * channel_scale
        source_valid = shifted[..., 2]
        weights = np.minimum(target_valid, source_valid)
        common_weight = float(np.sum(weights))
        if common_weight < config.minimum_common_weight:
            continue

        numerator = float(
            np.sum(
                weights[..., None]
                * target_features
                * source_features
            )
        )
        target_energy = float(
            np.sum(
                weights[..., None]
                * target_features
                * target_features
            )
        )
        source_energy = float(
            np.sum(
                weights[..., None]
                * source_features
                * source_features
            )
        )
        denominator = math.sqrt(
            max(0.0, target_energy * source_energy)
        )
        cosine = numerator / denominator if denominator > 1e-12 else 0.0
        cosine = min(1.0, max(-1.0, cosine))
        valid_difference = float(
            np.sum(weights * np.abs(target_valid - source_valid))
            / common_weight
        )
        common_fraction = common_weight / float(
            config.num_rings * config.num_sectors
        )
        distance = (
            1.0
            - cosine
            + config.valid_difference_weight * valid_difference
            + config.common_area_penalty_weight
            * (1.0 - min(1.0, common_fraction))
        )
        if distance < best.distance:
            yaw = normalize_angle(
                shift * 2.0 * math.pi / float(config.num_sectors)
            )
            best = ContextMatch(
                distance=float(distance),
                yaw_rad=float(yaw),
                sector_shift=shift,
                common_weight=common_weight,
            )
    return best


def rank_ring_candidates(
    query: PolarContext,
    database: Iterable[PolarContext],
    top_k: int,
) -> list[Tuple[int, float]]:
    distances = [
        (index, ring_key_distance(query, candidate))
        for index, candidate in enumerate(database)
    ]
    distances.sort(key=lambda item: item[1])
    return distances[: max(0, int(top_k))]
