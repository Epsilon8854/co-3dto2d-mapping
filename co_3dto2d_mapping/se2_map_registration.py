"""CPU-only geometric verification for two frozen 2-D occupancy submaps."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Tuple

import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree

from co_3dto2d_mapping.occupancy_submap import (
    LocalOccupancyPatch,
    occupied_boundary_mask,
    sample_points,
)
from co_3dto2d_mapping.planar_transform_utils import (
    PlanarTransform,
    invert_planar,
    normalize_angle,
)


@dataclass(frozen=True)
class RegistrationConfig:
    coarse_translation_range_m: float = 2.0
    coarse_translation_step_m: float = 0.25
    coarse_yaw_range_rad: float = math.radians(12.0)
    coarse_yaw_step_rad: float = math.radians(2.0)
    fine_translation_range_m: float = 0.35
    fine_translation_step_m: float = 0.05
    fine_yaw_range_rad: float = math.radians(2.0)
    fine_yaw_step_rad: float = math.radians(0.5)
    search_max_points: int = 700
    search_batch_size: int = 256
    distance_clip_m: float = 0.80
    overlap_distance_m: float = 0.25
    overlap_penalty_weight: float = 0.20
    icp_max_iterations: int = 35
    icp_max_correspondence_m: float = 0.50
    icp_trim_ratio: float = 0.75
    icp_max_points: int = 2500
    convergence_translation_m: float = 1e-3
    convergence_rotation_rad: float = math.radians(0.05)
    free_conflict_clearance_m: float = 0.15
    max_free_samples: int = 1200
    min_correspondences: int = 80
    min_symmetric_overlap: float = 0.30
    max_symmetric_rmse_m: float = 0.25
    max_free_conflict_ratio: float = 0.14

    def validated(self) -> "RegistrationConfig":
        positive = {
            "coarse_translation_step_m": self.coarse_translation_step_m,
            "coarse_yaw_step_rad": self.coarse_yaw_step_rad,
            "fine_translation_step_m": self.fine_translation_step_m,
            "fine_yaw_step_rad": self.fine_yaw_step_rad,
            "distance_clip_m": self.distance_clip_m,
            "overlap_distance_m": self.overlap_distance_m,
            "icp_max_correspondence_m": self.icp_max_correspondence_m,
            "convergence_translation_m": self.convergence_translation_m,
            "convergence_rotation_rad": self.convergence_rotation_rad,
        }
        for name, value in positive.items():
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
        non_negative = (
            self.coarse_translation_range_m,
            self.coarse_yaw_range_rad,
            self.fine_translation_range_m,
            self.fine_yaw_range_rad,
            self.overlap_penalty_weight,
            self.free_conflict_clearance_m,
        )
        if not all(
            np.isfinite(value) and value >= 0.0
            for value in non_negative
        ):
            raise ValueError(
                "registration ranges and penalties must be non-negative"
            )
        integer_positive = {
            "search_max_points": self.search_max_points,
            "search_batch_size": self.search_batch_size,
            "icp_max_iterations": self.icp_max_iterations,
            "icp_max_points": self.icp_max_points,
            "max_free_samples": self.max_free_samples,
            "min_correspondences": self.min_correspondences,
        }
        for name, value in integer_positive.items():
            if int(value) < 1:
                raise ValueError(f"{name} must be positive")
        if not 0.0 < self.icp_trim_ratio <= 1.0:
            raise ValueError("icp_trim_ratio must be in (0, 1]")
        if (
            not np.isfinite(self.max_symmetric_rmse_m)
            or self.max_symmetric_rmse_m <= 0.0
        ):
            raise ValueError(
                "max_symmetric_rmse_m must be positive and finite"
            )
        if not 0.0 <= self.min_symmetric_overlap <= 1.0:
            raise ValueError("min_symmetric_overlap must be in [0, 1]")
        if not 0.0 <= self.max_free_conflict_ratio <= 1.0:
            raise ValueError("max_free_conflict_ratio must be in [0, 1]")
        return RegistrationConfig(
            **{
                field: getattr(self, field)
                for field in self.__dataclass_fields__
            }
        )


@dataclass(frozen=True)
class RegistrationResult:
    transform: PlanarTransform
    search_score: float
    symmetric_rmse_m: float
    symmetric_overlap: float
    forward_overlap: float
    reverse_overlap: float
    free_conflict_ratio: float
    correspondences: int
    accepted: bool
    reason: str


def transform_points(
    points: np.ndarray, transform: PlanarTransform
) -> np.ndarray:
    values = np.asarray(points, dtype=np.float64)
    if values.ndim != 2 or values.shape[1:] != (2,):
        raise ValueError("points must have shape (N, 2)")
    x, y, yaw = transform
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    rotation = np.asarray(
        [[cosine, -sine], [sine, cosine]], dtype=np.float64
    )
    return values @ rotation.T + np.asarray([x, y], dtype=np.float64)


def _estimate_rigid(
    source: np.ndarray, target: np.ndarray
) -> PlanarTransform:
    source_mean = np.mean(source, axis=0)
    target_mean = np.mean(target, axis=0)
    centered_source = source - source_mean
    centered_target = target - target_mean
    u, _, vt = np.linalg.svd(centered_source.T @ centered_target)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = target_mean - rotation @ source_mean
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return float(translation[0]), float(translation[1]), float(yaw)


def _compose(
    left: PlanarTransform, right: PlanarTransform
) -> PlanarTransform:
    lx, ly, lyaw = left
    rx, ry, ryaw = right
    cosine = math.cos(lyaw)
    sine = math.sin(lyaw)
    return (
        lx + cosine * rx - sine * ry,
        ly + sine * rx + cosine * ry,
        normalize_angle(lyaw + ryaw),
    )


def _boundary_distance_field(
    patch: LocalOccupancyPatch,
) -> np.ndarray:
    boundary = occupied_boundary_mask(patch.grid)
    if not np.any(boundary):
        raise ValueError("occupancy patch has no occupied boundary")
    return distance_transform_edt(~boundary) * float(patch.resolution)


def _grid_indices(
    patch: LocalOccupancyPatch, points: np.ndarray
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    columns = np.floor(
        (points[..., 0] - patch.origin_x) / patch.resolution
    ).astype(np.int64)
    rows = np.floor(
        (points[..., 1] - patch.origin_y) / patch.resolution
    ).astype(np.int64)
    valid = (
        (columns >= 0)
        & (columns < patch.grid.shape[1])
        & (rows >= 0)
        & (rows < patch.grid.shape[0])
    )
    return rows, columns, valid


def _grid_values(
    patch: LocalOccupancyPatch, points: np.ndarray
) -> np.ndarray:
    rows, columns, valid = _grid_indices(patch, points)
    values = np.full(points.shape[:-1], -1, dtype=np.int16)
    values[valid] = patch.grid[rows[valid], columns[valid]]
    return values


def _range_values(
    center: float, half_range: float, step: float
) -> np.ndarray:
    if half_range <= 0.0:
        return np.asarray([center], dtype=np.float64)
    count = max(1, int(math.ceil(half_range / step)))
    offsets = np.arange(-count, count + 1, dtype=np.float64) * step
    return center + offsets


def _translation_offsets(
    half_range: float, step: float
) -> np.ndarray:
    values = _range_values(0.0, half_range, step)
    x, y = np.meshgrid(values, values, indexing="xy")
    return np.column_stack((x.reshape(-1), y.reshape(-1)))


def _best_raster_score_for_yaw(
    target: LocalOccupancyPatch,
    target_field: np.ndarray,
    source: LocalOccupancyPatch,
    source_field: np.ndarray,
    target_points: np.ndarray,
    source_points: np.ndarray,
    yaw: float,
    translation_center: np.ndarray,
    translation_offsets: np.ndarray,
    config: RegistrationConfig,
) -> Tuple[PlanarTransform, float]:
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    rotation = np.asarray(
        [[cosine, -sine], [sine, cosine]], dtype=np.float64
    )
    rotated = source_points @ rotation.T
    translations = translation_center[None, :] + translation_offsets
    best_transform = (
        float(translations[0, 0]),
        float(translations[0, 1]),
        yaw,
    )
    best_score = float("inf")

    for start in range(0, len(translations), config.search_batch_size):
        batch = translations[start : start + config.search_batch_size]
        transformed = rotated[None, :, :] + batch[:, None, :]
        rows, columns, valid = _grid_indices(target, transformed)
        forward = np.full(
            valid.shape, config.distance_clip_m, dtype=np.float64
        )
        forward[valid] = target_field[rows[valid], columns[valid]]
        forward = np.minimum(forward, config.distance_clip_m)

        # Symmetric scoring prevents a small fragment from being rewarded for
        # matching only one repeated wall in the target map.
        inverse_transformed = (
            target_points[None, :, :] - batch[:, None, :]
        ) @ rotation
        reverse_rows, reverse_columns, reverse_valid = _grid_indices(
            source, inverse_transformed
        )
        reverse = np.full(
            reverse_valid.shape,
            config.distance_clip_m,
            dtype=np.float64,
        )
        reverse[reverse_valid] = source_field[
            reverse_rows[reverse_valid], reverse_columns[reverse_valid]
        ]
        reverse = np.minimum(reverse, config.distance_clip_m)

        forward_overlap = np.mean(
            forward <= config.overlap_distance_m, axis=1
        )
        reverse_overlap = np.mean(
            reverse <= config.overlap_distance_m, axis=1
        )
        overlap = np.minimum(forward_overlap, reverse_overlap)
        mean_square = 0.5 * (
            np.mean(forward * forward, axis=1)
            + np.mean(reverse * reverse, axis=1)
        )
        scores = mean_square + config.overlap_penalty_weight * np.square(
            1.0 - overlap
        )
        index = int(np.argmin(scores))
        score = float(scores[index])
        if score < best_score:
            best_score = score
            best = batch[index]
            best_transform = (
                float(best[0]),
                float(best[1]),
                float(yaw),
            )
    return best_transform, best_score


def _search_transform(
    target: LocalOccupancyPatch,
    source: LocalOccupancyPatch,
    initial_yaw: float,
    config: RegistrationConfig,
) -> Tuple[PlanarTransform, float]:
    target_points = sample_points(
        target.boundary_points, config.search_max_points
    )
    source_points = sample_points(
        source.boundary_points, config.search_max_points
    )
    target_field = _boundary_distance_field(target)
    source_field = _boundary_distance_field(source)
    target_centroid = np.mean(target_points, axis=0)
    source_centroid = np.mean(source_points, axis=0)

    best_transform: Optional[PlanarTransform] = None
    best_score = float("inf")
    coarse_offsets = _translation_offsets(
        config.coarse_translation_range_m,
        config.coarse_translation_step_m,
    )
    for yaw in _range_values(
        initial_yaw,
        config.coarse_yaw_range_rad,
        config.coarse_yaw_step_rad,
    ):
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        rotation = np.asarray([[cosine, -sine], [sine, cosine]])
        centroid_translation = (
            target_centroid - source_centroid @ rotation.T
        )
        translation_centers = [
            centroid_translation,
            np.zeros(2, dtype=np.float64),
        ]
        for translation_center in translation_centers:
            transform, score = _best_raster_score_for_yaw(
                target,
                target_field,
                source,
                source_field,
                target_points,
                source_points,
                normalize_angle(float(yaw)),
                translation_center,
                coarse_offsets,
                config,
            )
            if score < best_score:
                best_transform, best_score = transform, score

    if best_transform is None:
        raise ValueError("coarse registration produced no candidate")

    fine_offsets = _translation_offsets(
        config.fine_translation_range_m,
        config.fine_translation_step_m,
    )
    fine_center = np.asarray(best_transform[:2], dtype=np.float64)
    coarse_yaw = best_transform[2]
    for yaw in _range_values(
        coarse_yaw,
        config.fine_yaw_range_rad,
        config.fine_yaw_step_rad,
    ):
        transform, score = _best_raster_score_for_yaw(
            target,
            target_field,
            source,
            source_field,
            target_points,
            source_points,
            normalize_angle(float(yaw)),
            fine_center,
            fine_offsets,
            config,
        )
        if score < best_score:
            best_transform, best_score = transform, score
    return best_transform, best_score


def _trimmed_icp(
    target_points: np.ndarray,
    source_points: np.ndarray,
    initial: PlanarTransform,
    config: RegistrationConfig,
) -> PlanarTransform:
    target = sample_points(target_points, config.icp_max_points)
    source = sample_points(source_points, config.icp_max_points)
    tree = cKDTree(target)
    transform = initial

    for _ in range(config.icp_max_iterations):
        transformed = transform_points(source, transform)
        distances, indices = tree.query(transformed, k=1)
        valid = distances <= config.icp_max_correspondence_m
        count = int(np.count_nonzero(valid))
        if count < config.min_correspondences:
            break
        valid_indices = np.flatnonzero(valid)
        keep_count = max(
            config.min_correspondences,
            int(math.floor(count * config.icp_trim_ratio)),
        )
        if keep_count < count:
            order = np.argpartition(
                distances[valid_indices], keep_count - 1
            )
            valid_indices = valid_indices[order[:keep_count]]
        delta = _estimate_rigid(
            transformed[valid_indices], target[indices[valid_indices]]
        )
        transform = _compose(delta, transform)
        if (
            math.hypot(delta[0], delta[1])
            <= config.convergence_translation_m
            and abs(delta[2]) <= config.convergence_rotation_rad
        ):
            break
    return transform


def _direction_metrics(
    target_points: np.ndarray,
    source_points: np.ndarray,
    target_from_source: PlanarTransform,
    config: RegistrationConfig,
) -> Tuple[np.ndarray, float, int]:
    tree = cKDTree(target_points)
    distances, _ = tree.query(
        transform_points(source_points, target_from_source), k=1
    )
    overlap = float(
        np.mean(distances <= config.overlap_distance_m)
    )
    valid = distances <= config.icp_max_correspondence_m
    return distances[valid], overlap, int(np.count_nonzero(valid))


def _interior_free_points(
    patch: LocalOccupancyPatch,
    config: RegistrationConfig,
) -> np.ndarray:
    points = sample_points(patch.free_points, config.max_free_samples)
    if not len(points) or config.free_conflict_clearance_m <= 0.0:
        return points
    field = _boundary_distance_field(patch)
    rows, columns, valid = _grid_indices(patch, points)
    keep = np.zeros(len(points), dtype=bool)
    keep[valid] = (
        field[rows[valid], columns[valid]]
        >= config.free_conflict_clearance_m
    )
    return points[keep]


def _free_conflict_ratio(
    target: LocalOccupancyPatch,
    source: LocalOccupancyPatch,
    target_from_source: PlanarTransform,
    config: RegistrationConfig,
) -> float:
    inverse = invert_planar(target_from_source)
    conflict = 0
    comparable = 0

    source_boundary = sample_points(
        source.boundary_points, config.max_free_samples
    )
    target_boundary = sample_points(
        target.boundary_points, config.max_free_samples
    )
    for patch, points in (
        (target, transform_points(source_boundary, target_from_source)),
        (source, transform_points(target_boundary, inverse)),
    ):
        values = _grid_values(patch, points)
        known = values >= 0
        comparable += int(np.count_nonzero(known))
        conflict += int(np.count_nonzero(known & (values <= 50)))

    source_free = _interior_free_points(source, config)
    target_free = _interior_free_points(target, config)
    for patch, points in (
        (target, transform_points(source_free, target_from_source)),
        (source, transform_points(target_free, inverse)),
    ):
        values = _grid_values(patch, points)
        known = values >= 0
        comparable += int(np.count_nonzero(known))
        conflict += int(np.count_nonzero(known & (values > 50)))

    return (
        float(conflict) / float(comparable) if comparable else 1.0
    )


def register_submaps(
    target: LocalOccupancyPatch,
    source: LocalOccupancyPatch,
    initial_yaw_rad: float,
    config: RegistrationConfig = RegistrationConfig(),
) -> RegistrationResult:
    """Estimate and verify ``target_keyframe <- source_keyframe``."""

    config = config.validated()
    if (
        target.occupied_boundary_count < config.min_correspondences
        or source.occupied_boundary_count < config.min_correspondences
    ):
        return RegistrationResult(
            transform=(
                0.0,
                0.0,
                normalize_angle(initial_yaw_rad),
            ),
            search_score=float("inf"),
            symmetric_rmse_m=float("inf"),
            symmetric_overlap=0.0,
            forward_overlap=0.0,
            reverse_overlap=0.0,
            free_conflict_ratio=1.0,
            correspondences=0,
            accepted=False,
            reason="insufficient_boundary_points",
        )

    try:
        initial, search_score = _search_transform(
            target,
            source,
            normalize_angle(initial_yaw_rad),
            config,
        )
        refined = _trimmed_icp(
            target.boundary_points,
            source.boundary_points,
            initial,
            config,
        )
        (
            forward_distances,
            forward_overlap,
            forward_count,
        ) = _direction_metrics(
            target.boundary_points,
            source.boundary_points,
            refined,
            config,
        )
        (
            reverse_distances,
            reverse_overlap,
            reverse_count,
        ) = _direction_metrics(
            source.boundary_points,
            target.boundary_points,
            invert_planar(refined),
            config,
        )
    except (ValueError, np.linalg.LinAlgError):
        return RegistrationResult(
            transform=(
                0.0,
                0.0,
                normalize_angle(initial_yaw_rad),
            ),
            search_score=float("inf"),
            symmetric_rmse_m=float("inf"),
            symmetric_overlap=0.0,
            forward_overlap=0.0,
            reverse_overlap=0.0,
            free_conflict_ratio=1.0,
            correspondences=0,
            accepted=False,
            reason="registration_failed",
        )

    distances = np.concatenate(
        (forward_distances, reverse_distances)
    )
    rmse = (
        float(np.sqrt(np.mean(distances * distances)))
        if distances.size
        else float("inf")
    )
    overlap = min(forward_overlap, reverse_overlap)
    correspondences = min(forward_count, reverse_count)
    conflict = _free_conflict_ratio(
        target, source, refined, config
    )

    if correspondences < config.min_correspondences:
        accepted, reason = False, "insufficient_correspondences"
    elif overlap < config.min_symmetric_overlap:
        accepted, reason = False, "low_symmetric_overlap"
    elif rmse > config.max_symmetric_rmse_m:
        accepted, reason = False, "high_symmetric_rmse"
    elif conflict > config.max_free_conflict_ratio:
        accepted, reason = False, "free_space_conflict"
    else:
        accepted, reason = True, "accepted"

    return RegistrationResult(
        transform=refined,
        search_score=float(search_score),
        symmetric_rmse_m=rmse,
        symmetric_overlap=float(overlap),
        forward_overlap=float(forward_overlap),
        reverse_overlap=float(reverse_overlap),
        free_conflict_ratio=float(conflict),
        correspondences=int(correspondences),
        accepted=accepted,
        reason=reason,
    )
