"""CPU-only 2D occupancy place recognition and SE(2) map alignment.

The module deliberately has no ROS dependency.  It provides the geometric core
used by :mod:`inter_robot_place_alignment` and can therefore be unit tested on
ordinary NumPy arrays.

Coordinate convention
---------------------
``Pose2`` always denotes ``T_parent_child``: it maps a point expressed in the
child frame into the parent frame.  A registration result maps source-submap
coordinates into target-submap coordinates.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Hashable, Iterable, List, Optional, Sequence, Tuple

import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree


_TWO_PI = 2.0 * math.pi


@dataclass(frozen=True)
class Pose2:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class GridSnapshot:
    """Immutable occupancy-grid snapshot.

    ``data`` uses the ROS occupancy convention: -1 unknown, 0 free, and values
    above ``occupied_threshold`` occupied.  The array is indexed ``[row, col]``.
    """

    resolution: float
    origin: Pose2
    data: np.ndarray
    frame_id: str = ""
    stamp_ns: int = 0

    def __post_init__(self) -> None:
        array = np.asarray(self.data, dtype=np.int16)
        if array.ndim != 2:
            raise ValueError("GridSnapshot.data must be a 2-D array")
        if self.resolution <= 0.0:
            raise ValueError("GridSnapshot.resolution must be positive")
        object.__setattr__(self, "data", array.copy())

    @property
    def height(self) -> int:
        return int(self.data.shape[0])

    @property
    def width(self) -> int:
        return int(self.data.shape[1])


@dataclass(frozen=True)
class LocalSubmap:
    resolution: float
    radius_m: float
    data: np.ndarray
    known: np.ndarray
    free: np.ndarray
    occupied: np.ndarray
    boundary: np.ndarray
    boundary_points: np.ndarray
    known_ratio: float

    @property
    def size(self) -> int:
        return int(self.data.shape[0])

    @property
    def half_index(self) -> int:
        return self.size // 2


@dataclass(frozen=True)
class PolarDescriptor:
    """Two-channel rotation-searchable occupancy descriptor.

    Channel 0 contains occupied-boundary strength.  Channel 1 contains the
    fraction of known cells that are free.  ``valid`` marks bins with enough
    observed cells; unknown space is never treated as free.
    """

    values: np.ndarray  # [2, rings, sectors]
    valid: np.ndarray  # [rings, sectors]
    ring_key: np.ndarray
    known_ratio: float

    @property
    def rings(self) -> int:
        return int(self.values.shape[1])

    @property
    def sectors(self) -> int:
        return int(self.values.shape[2])


@dataclass(frozen=True)
class DescriptorMatch:
    item: Hashable
    similarity: float
    yaw_source_to_target: float
    ring_key_distance: float


@dataclass(frozen=True)
class RegistrationOptions:
    coarse_translation_range_m: float = 4.0
    coarse_translation_step_m: float = 0.25
    coarse_yaw_range_rad: float = math.radians(30.0)
    coarse_yaw_step_rad: float = math.radians(2.0)
    fine_translation_range_m: float = 0.50
    fine_translation_step_m: float = 0.05
    fine_yaw_range_rad: float = math.radians(2.0)
    fine_yaw_step_rad: float = math.radians(0.5)
    search_max_distance_m: float = 0.75
    search_max_points: int = 600
    search_translation_chunk: int = 256
    icp_max_iterations: int = 30
    icp_max_correspondence_m: float = 0.40
    icp_trim_ratio: float = 0.75
    min_correspondences: int = 60
    min_symmetric_overlap: float = 0.35
    max_symmetric_rmse_m: float = 0.20
    max_free_space_conflict_ratio: float = 0.10
    free_space_conflict_clearance_m: float = 0.20
    convergence_translation_m: float = 1e-3
    convergence_rotation_rad: float = math.radians(0.05)
    free_space_conflict_weight: float = 2.0
    unknown_space_weight: float = 0.35
    overlap_reward: float = 0.50

    def validate(self) -> None:
        positive = {
            "coarse_translation_step_m": self.coarse_translation_step_m,
            "coarse_yaw_step_rad": self.coarse_yaw_step_rad,
            "fine_translation_step_m": self.fine_translation_step_m,
            "fine_yaw_step_rad": self.fine_yaw_step_rad,
            "search_max_distance_m": self.search_max_distance_m,
            "icp_max_correspondence_m": self.icp_max_correspondence_m,
        }
        for name, value in positive.items():
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
        if self.search_max_points < 3:
            raise ValueError("search_max_points must be at least 3")
        if self.search_translation_chunk < 1:
            raise ValueError("search_translation_chunk must be positive")
        if self.icp_max_iterations < 1:
            raise ValueError("icp_max_iterations must be positive")
        if self.min_correspondences < 3:
            raise ValueError("min_correspondences must be at least 3")
        if not 0.05 <= self.icp_trim_ratio <= 1.0:
            raise ValueError("icp_trim_ratio must be in [0.05, 1.0]")
        if not 0.0 <= self.min_symmetric_overlap <= 1.0:
            raise ValueError("min_symmetric_overlap must be in [0, 1]")
        if not 0.0 <= self.max_free_space_conflict_ratio <= 1.0:
            raise ValueError("max_free_space_conflict_ratio must be in [0, 1]")
        if self.free_space_conflict_clearance_m < 0.0:
            raise ValueError("free_space_conflict_clearance_m must be non-negative")


@dataclass(frozen=True)
class RegistrationResult:
    success: bool
    transform_source_to_target: Pose2
    symmetric_rmse_m: float
    symmetric_overlap: float
    source_overlap: float
    target_overlap: float
    free_space_conflict_ratio: float
    correspondences: int
    score: float
    reason: str = ""


@dataclass(frozen=True)
class AlignmentMeasurement:
    transform_map1_to_map0: Pose2
    robot0_keyframe_id: int
    robot1_keyframe_id: int
    descriptor_similarity: float
    symmetric_overlap: float
    symmetric_rmse_m: float
    free_space_conflict_ratio: float
    stamp_ns: int = 0

    def weight(self, rmse_scale_m: float = 0.20) -> float:
        scale = max(1e-6, rmse_scale_m)
        descriptor = float(np.clip(self.descriptor_similarity, 0.0, 1.0))
        overlap = float(np.clip(self.symmetric_overlap, 0.0, 1.0))
        conflict = float(np.clip(self.free_space_conflict_ratio, 0.0, 1.0))
        return max(
            1e-9,
            descriptor
            * overlap
            * overlap
            * math.exp(-((self.symmetric_rmse_m / scale) ** 2))
            * (1.0 - conflict),
        )


@dataclass(frozen=True)
class ConsensusResult:
    transform_map1_to_map0: Pose2
    measurements: Tuple[AlignmentMeasurement, ...]
    total_weight: float
    translation_spread_m: float
    yaw_spread_rad: float

    @property
    def support_count(self) -> int:
        return len(self.measurements)

    @property
    def distinct_robot0_keyframes(self) -> int:
        return len({m.robot0_keyframe_id for m in self.measurements})

    @property
    def distinct_robot1_keyframes(self) -> int:
        return len({m.robot1_keyframe_id for m in self.measurements})


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def angular_distance(left: float, right: float) -> float:
    return abs(normalize_angle(left - right))


def compose_pose(parent_from_middle: Pose2, middle_from_child: Pose2) -> Pose2:
    c = math.cos(parent_from_middle.yaw)
    s = math.sin(parent_from_middle.yaw)
    return Pose2(
        parent_from_middle.x
        + c * middle_from_child.x
        - s * middle_from_child.y,
        parent_from_middle.y
        + s * middle_from_child.x
        + c * middle_from_child.y,
        normalize_angle(parent_from_middle.yaw + middle_from_child.yaw),
    )


def inverse_pose(parent_from_child: Pose2) -> Pose2:
    c = math.cos(parent_from_child.yaw)
    s = math.sin(parent_from_child.yaw)
    return Pose2(
        -c * parent_from_child.x - s * parent_from_child.y,
        s * parent_from_child.x - c * parent_from_child.y,
        normalize_angle(-parent_from_child.yaw),
    )


def between_pose(parent_from_a: Pose2, parent_from_b: Pose2) -> Pose2:
    return compose_pose(inverse_pose(parent_from_a), parent_from_b)


def transform_points(transform: Pose2, points: np.ndarray) -> np.ndarray:
    array = np.asarray(points, dtype=np.float64)
    if array.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    c = math.cos(transform.yaw)
    s = math.sin(transform.yaw)
    rotation = np.asarray([[c, -s], [s, c]], dtype=np.float64)
    return array @ rotation.T + np.asarray([transform.x, transform.y])


def _boundary_mask(occupied: np.ndarray) -> np.ndarray:
    padded = np.pad(occupied.astype(bool), 1, mode="constant", constant_values=False)
    interior = (
        padded[1:-1, :-2]
        & padded[1:-1, 2:]
        & padded[:-2, 1:-1]
        & padded[2:, 1:-1]
    )
    return occupied & ~interior


def extract_local_submap(
    grid: GridSnapshot,
    keyframe_pose: Pose2,
    radius_m: float,
    output_resolution_m: float,
    occupied_threshold: int = 50,
    max_boundary_points: int = 3000,
) -> LocalSubmap:
    """Resample a circular occupancy patch around ``keyframe_pose``.

    The returned patch is expressed in the keyframe/body frame, not the map
    frame.  This is what makes its polar descriptor independent of global
    translation and makes the descriptor yaw directly useful for registration.
    """

    if radius_m <= 0.0:
        raise ValueError("radius_m must be positive")
    if output_resolution_m <= 0.0:
        raise ValueError("output_resolution_m must be positive")

    half = int(math.ceil(radius_m / output_resolution_m))
    size = 2 * half + 1
    axis = (np.arange(size, dtype=np.float64) - half) * output_resolution_m
    local_x, local_y = np.meshgrid(axis, axis)
    circular = local_x * local_x + local_y * local_y <= radius_m * radius_m

    ck = math.cos(keyframe_pose.yaw)
    sk = math.sin(keyframe_pose.yaw)
    world_x = keyframe_pose.x + ck * local_x - sk * local_y
    world_y = keyframe_pose.y + sk * local_x + ck * local_y

    dx = world_x - grid.origin.x
    dy = world_y - grid.origin.y
    co = math.cos(grid.origin.yaw)
    so = math.sin(grid.origin.yaw)
    grid_x = co * dx + so * dy
    grid_y = -so * dx + co * dy
    cols = np.floor(grid_x / grid.resolution).astype(np.int64)
    rows = np.floor(grid_y / grid.resolution).astype(np.int64)
    inside = (
        circular
        & (cols >= 0)
        & (cols < grid.width)
        & (rows >= 0)
        & (rows < grid.height)
    )

    patch = np.full((size, size), -1, dtype=np.int16)
    patch[inside] = grid.data[rows[inside], cols[inside]]
    known = patch >= 0
    occupied = patch > occupied_threshold
    free = known & ~occupied
    boundary = _boundary_mask(occupied)

    boundary_rows, boundary_cols = np.nonzero(boundary)
    boundary_points = np.column_stack(
        (
            (boundary_cols.astype(np.float64) - half) * output_resolution_m,
            (boundary_rows.astype(np.float64) - half) * output_resolution_m,
        )
    )
    if max_boundary_points > 0 and len(boundary_points) > max_boundary_points:
        indices = np.linspace(
            0, len(boundary_points) - 1, max_boundary_points, dtype=np.int64
        )
        boundary_points = boundary_points[indices]

    circular_count = max(1, int(np.count_nonzero(circular)))
    known_ratio = float(np.count_nonzero(known & circular)) / float(circular_count)
    return LocalSubmap(
        resolution=output_resolution_m,
        radius_m=radius_m,
        data=patch,
        known=known,
        free=free,
        occupied=occupied,
        boundary=boundary,
        boundary_points=boundary_points,
        known_ratio=known_ratio,
    )


def build_polar_descriptor(
    submap: LocalSubmap,
    num_rings: int = 20,
    num_sectors: int = 60,
    min_known_cells_per_bin: int = 2,
) -> PolarDescriptor:
    if num_rings < 1 or num_sectors < 4:
        raise ValueError("descriptor requires at least 1 ring and 4 sectors")

    half = submap.half_index
    axis = (np.arange(submap.size, dtype=np.float64) - half) * submap.resolution
    x, y = np.meshgrid(axis, axis)
    radius = np.hypot(x, y)
    theta = np.mod(np.arctan2(y, x), _TWO_PI)
    inside = radius <= submap.radius_m

    ring = np.minimum(
        num_rings - 1,
        np.floor(radius / submap.radius_m * num_rings).astype(np.int64),
    )
    sector = np.minimum(
        num_sectors - 1,
        np.floor(theta / _TWO_PI * num_sectors).astype(np.int64),
    )
    flat_bin = ring * num_sectors + sector
    bin_count = num_rings * num_sectors

    known_mask = inside & submap.known
    free_mask = inside & submap.free
    boundary_mask = inside & submap.boundary
    known_count = np.bincount(
        flat_bin[known_mask].ravel(), minlength=bin_count
    ).astype(np.float64)
    free_count = np.bincount(
        flat_bin[free_mask].ravel(), minlength=bin_count
    ).astype(np.float64)
    boundary_count = np.bincount(
        flat_bin[boundary_mask].ravel(), minlength=bin_count
    ).astype(np.float64)

    occupied_strength = np.log1p(boundary_count)
    maximum = float(np.max(occupied_strength)) if occupied_strength.size else 0.0
    if maximum > 0.0:
        occupied_strength /= maximum
    free_ratio = np.divide(
        free_count,
        known_count,
        out=np.zeros_like(free_count),
        where=known_count > 0.0,
    )
    valid = known_count >= float(max(1, min_known_cells_per_bin))

    values = np.stack(
        (
            occupied_strength.reshape(num_rings, num_sectors),
            free_ratio.reshape(num_rings, num_sectors),
        ),
        axis=0,
    )
    valid_2d = valid.reshape(num_rings, num_sectors)

    ring_key_channels: List[np.ndarray] = []
    for channel in values:
        weighted_sum = np.sum(channel * valid_2d, axis=1)
        valid_count = np.sum(valid_2d, axis=1)
        ring_key_channels.append(
            np.divide(
                weighted_sum,
                valid_count,
                out=np.zeros(num_rings, dtype=np.float64),
                where=valid_count > 0,
            )
        )
    ring_key = np.concatenate(ring_key_channels)
    return PolarDescriptor(
        values=values,
        valid=valid_2d,
        ring_key=ring_key,
        known_ratio=submap.known_ratio,
    )


def _cosine_similarity(left: np.ndarray, right: np.ndarray) -> float:
    left_norm = float(np.linalg.norm(left))
    right_norm = float(np.linalg.norm(right))
    if left_norm <= 1e-12 or right_norm <= 1e-12:
        return 0.0
    return float(np.dot(left, right) / (left_norm * right_norm))


def match_polar_descriptors(
    target: PolarDescriptor,
    source: PolarDescriptor,
    occupied_weight: float = 0.70,
    free_weight: float = 0.30,
    minimum_common_bins: int = 20,
) -> Tuple[float, float]:
    """Return similarity and yaw mapping source coordinates into target.

    A positive circular shift moves source sector content toward increasing
    target angles.  The same positive angle is therefore the SE(2) yaw used to
    rotate source points into the target frame.
    """

    if target.values.shape != source.values.shape:
        raise ValueError("descriptor shapes must match")
    sectors = target.sectors
    best_similarity = -1.0
    best_shift = 0
    target_valid_count = int(np.count_nonzero(target.valid))
    source_valid_count = int(np.count_nonzero(source.valid))
    denominator = max(1, max(target_valid_count, source_valid_count))

    weight_sum = max(1e-9, occupied_weight + free_weight)
    occ_weight = occupied_weight / weight_sum
    fr_weight = free_weight / weight_sum

    for shift in range(sectors):
        shifted_values = np.roll(source.values, shift, axis=2)
        shifted_valid = np.roll(source.valid, shift, axis=1)
        common = target.valid & shifted_valid
        common_count = int(np.count_nonzero(common))
        if common_count < minimum_common_bins:
            continue

        target_occ = target.values[0][common]
        source_occ = shifted_values[0][common]
        occupied_similarity = max(0.0, _cosine_similarity(target_occ, source_occ))
        free_similarity = 1.0 - float(
            np.mean(np.abs(target.values[1][common] - shifted_values[1][common]))
        )
        free_similarity = float(np.clip(free_similarity, 0.0, 1.0))
        common_fraction = float(common_count) / float(denominator)
        coverage_factor = 0.5 + 0.5 * common_fraction
        similarity = (
            occ_weight * occupied_similarity + fr_weight * free_similarity
        ) * coverage_factor
        if similarity > best_similarity:
            best_similarity = similarity
            best_shift = shift

    if best_similarity < 0.0:
        return 0.0, 0.0
    yaw = normalize_angle(float(best_shift) * _TWO_PI / float(sectors))
    return float(best_similarity), yaw


def rank_descriptor_matches(
    target: PolarDescriptor,
    database: Sequence[Tuple[Hashable, PolarDescriptor]],
    top_k: int = 5,
    candidate_multiplier: int = 3,
) -> List[DescriptorMatch]:
    if not database or top_k <= 0:
        return []
    ring_distances = np.asarray(
        [np.linalg.norm(target.ring_key - descriptor.ring_key) for _, descriptor in database],
        dtype=np.float64,
    )
    candidate_count = min(
        len(database), max(top_k, top_k * max(1, candidate_multiplier))
    )
    candidate_indices = np.argsort(ring_distances)[:candidate_count]
    matches: List[DescriptorMatch] = []
    for index in candidate_indices:
        item, descriptor = database[int(index)]
        similarity, yaw = match_polar_descriptors(target, descriptor)
        matches.append(
            DescriptorMatch(
                item=item,
                similarity=similarity,
                yaw_source_to_target=yaw,
                ring_key_distance=float(ring_distances[int(index)]),
            )
        )
    matches.sort(key=lambda match: (match.similarity, -match.ring_key_distance), reverse=True)
    return matches[: min(top_k, len(matches))]


def _axis_values(center: float, radius: float, step: float) -> np.ndarray:
    count = int(math.floor(max(0.0, radius) / step + 1e-9))
    return center + np.arange(-count, count + 1, dtype=np.float64) * step


def _sample_submap_mask(mask: np.ndarray, submap: LocalSubmap, points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    points = np.asarray(points, dtype=np.float64)
    cols = np.rint(points[:, 0] / submap.resolution + submap.half_index).astype(np.int64)
    rows = np.rint(points[:, 1] / submap.resolution + submap.half_index).astype(np.int64)
    inside = (
        (cols >= 0)
        & (cols < submap.size)
        & (rows >= 0)
        & (rows < submap.size)
    )
    values = np.zeros(len(points), dtype=bool)
    values[inside] = mask[rows[inside], cols[inside]]
    return values, inside


def _subsample_points(points: np.ndarray, maximum: int) -> np.ndarray:
    if len(points) <= maximum:
        return points
    indices = np.linspace(0, len(points) - 1, maximum, dtype=np.int64)
    return points[indices]


def _evaluate_translation_candidates(
    rotated_source: np.ndarray,
    translations: np.ndarray,
    target: LocalSubmap,
    target_distance_m: np.ndarray,
    options: RegistrationOptions,
) -> Tuple[int, np.ndarray]:
    n_points = len(rotated_source)
    costs = np.full(len(translations), np.inf, dtype=np.float64)
    max_distance = options.search_max_distance_m
    max_distance_sq = max_distance * max_distance
    half = target.half_index
    size = target.size
    resolution = target.resolution

    for begin in range(0, len(translations), options.search_translation_chunk):
        end = min(len(translations), begin + options.search_translation_chunk)
        translation_chunk = translations[begin:end]
        x = rotated_source[None, :, 0] + translation_chunk[:, None, 0]
        y = rotated_source[None, :, 1] + translation_chunk[:, None, 1]
        cols = np.rint(x / resolution + half).astype(np.int64)
        rows = np.rint(y / resolution + half).astype(np.int64)
        inside = (cols >= 0) & (cols < size) & (rows >= 0) & (rows < size)
        safe_cols = np.clip(cols, 0, size - 1)
        safe_rows = np.clip(rows, 0, size - 1)
        known = inside & target.known[safe_rows, safe_cols]
        target_free = known & target.free[safe_rows, safe_cols]
        distances = np.full_like(x, max_distance, dtype=np.float64)
        distances[inside] = target_distance_m[safe_rows[inside], safe_cols[inside]]
        free = target_free & (distances > options.free_space_conflict_clearance_m)
        clipped_sq = np.minimum(distances * distances, max_distance_sq)
        geometry_cost = np.mean(clipped_sq, axis=1)
        overlap = np.count_nonzero(known & (distances <= max_distance), axis=1) / float(n_points)
        conflict = np.count_nonzero(free, axis=1) / float(n_points)
        unknown = 1.0 - np.count_nonzero(known, axis=1) / float(n_points)
        costs[begin:end] = (
            geometry_cost
            + options.free_space_conflict_weight * conflict
            + options.unknown_space_weight * unknown
            - options.overlap_reward * overlap
        )
    best_index = int(np.argmin(costs))
    return best_index, costs


def _correlative_search(
    target: LocalSubmap,
    source: LocalSubmap,
    initial_yaw: float,
    center_translation: Tuple[float, float],
    translation_range_m: float,
    translation_step_m: float,
    yaw_range_rad: float,
    yaw_step_rad: float,
    options: RegistrationOptions,
) -> Optional[Pose2]:
    if len(target.boundary_points) < 3 or len(source.boundary_points) < 3:
        return None
    sampled_source = _subsample_points(source.boundary_points, options.search_max_points)
    target_distance_m = distance_transform_edt(~target.boundary) * target.resolution

    x_values = _axis_values(center_translation[0], translation_range_m, translation_step_m)
    y_values = _axis_values(center_translation[1], translation_range_m, translation_step_m)
    grid_x, grid_y = np.meshgrid(x_values, y_values)
    translations = np.column_stack((grid_x.ravel(), grid_y.ravel()))
    yaw_values = _axis_values(initial_yaw, yaw_range_rad, yaw_step_rad)

    best_pose: Optional[Pose2] = None
    best_cost = math.inf
    for yaw in yaw_values:
        c = math.cos(float(yaw))
        s = math.sin(float(yaw))
        rotation = np.asarray([[c, -s], [s, c]], dtype=np.float64)
        rotated = sampled_source @ rotation.T
        best_index, costs = _evaluate_translation_candidates(
            rotated, translations, target, target_distance_m, options
        )
        cost = float(costs[best_index])
        if cost < best_cost:
            best_cost = cost
            best_pose = Pose2(
                float(translations[best_index, 0]),
                float(translations[best_index, 1]),
                normalize_angle(float(yaw)),
            )
    return best_pose


def _estimate_rigid_transform(source: np.ndarray, target: np.ndarray) -> Optional[Pose2]:
    if len(source) < 2 or len(target) != len(source):
        return None
    source_mean = np.mean(source, axis=0)
    target_mean = np.mean(target, axis=0)
    source_centered = source - source_mean
    target_centered = target - target_mean
    covariance = source_centered.T @ target_centered
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = target_mean - rotation @ source_mean
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return Pose2(float(translation[0]), float(translation[1]), normalize_angle(yaw))


def _trimmed_icp(
    target_points: np.ndarray,
    source_points: np.ndarray,
    initial_pose: Pose2,
    options: RegistrationOptions,
) -> Optional[Pose2]:
    if (
        len(target_points) < options.min_correspondences
        or len(source_points) < options.min_correspondences
    ):
        return None
    tree = cKDTree(target_points)
    pose = initial_pose
    for _ in range(options.icp_max_iterations):
        transformed = transform_points(pose, source_points)
        distances, indices = tree.query(transformed, k=1)
        valid_indices = np.flatnonzero(distances <= options.icp_max_correspondence_m)
        if len(valid_indices) < options.min_correspondences:
            return None
        keep_count = max(
            options.min_correspondences,
            int(math.floor(options.icp_trim_ratio * len(valid_indices))),
        )
        order = valid_indices[np.argsort(distances[valid_indices])[:keep_count]]
        delta = _estimate_rigid_transform(transformed[order], target_points[indices[order]])
        if delta is None:
            return None
        pose = compose_pose(delta, pose)
        if (
            math.hypot(delta.x, delta.y) <= options.convergence_translation_m
            and abs(delta.yaw) <= options.convergence_rotation_rad
        ):
            break
    return pose


def _one_way_registration_metrics(
    target: LocalSubmap,
    source: LocalSubmap,
    transform_source_to_target: Pose2,
    max_correspondence_m: float,
    conflict_clearance_m: float,
) -> Tuple[float, float, float, int]:
    transformed = transform_points(transform_source_to_target, source.boundary_points)
    tree = cKDTree(target.boundary_points)
    distances, _ = tree.query(transformed, k=1)
    inliers = distances <= max_correspondence_m
    count = int(np.count_nonzero(inliers))
    overlap = float(count) / float(max(1, len(source.boundary_points)))
    rmse = (
        float(np.sqrt(np.mean(np.square(distances[inliers]))))
        if count > 0
        else math.inf
    )
    free_values, inside = _sample_submap_mask(target.free, target, transformed)
    conflict_mask = free_values & inside & (distances > conflict_clearance_m)
    conflict = float(np.count_nonzero(conflict_mask)) / float(
        max(1, len(source.boundary_points))
    )
    return overlap, rmse, conflict, count


def register_submaps(
    target: LocalSubmap,
    source: LocalSubmap,
    descriptor_yaw_source_to_target: float,
    options: RegistrationOptions = RegistrationOptions(),
) -> RegistrationResult:
    """Correlative-search + trimmed-ICP registration on occupied boundaries."""

    options.validate()
    failure_pose = Pose2(yaw=normalize_angle(descriptor_yaw_source_to_target))
    if (
        len(target.boundary_points) < options.min_correspondences
        or len(source.boundary_points) < options.min_correspondences
    ):
        return RegistrationResult(
            False,
            failure_pose,
            math.inf,
            0.0,
            0.0,
            0.0,
            1.0,
            0,
            0.0,
            "insufficient boundary points",
        )

    coarse = _correlative_search(
        target=target,
        source=source,
        initial_yaw=descriptor_yaw_source_to_target,
        center_translation=(0.0, 0.0),
        translation_range_m=options.coarse_translation_range_m,
        translation_step_m=options.coarse_translation_step_m,
        yaw_range_rad=options.coarse_yaw_range_rad,
        yaw_step_rad=options.coarse_yaw_step_rad,
        options=options,
    )
    if coarse is None:
        return RegistrationResult(
            False,
            failure_pose,
            math.inf,
            0.0,
            0.0,
            0.0,
            1.0,
            0,
            0.0,
            "coarse correlative search failed",
        )

    fine = _correlative_search(
        target=target,
        source=source,
        initial_yaw=coarse.yaw,
        center_translation=(coarse.x, coarse.y),
        translation_range_m=options.fine_translation_range_m,
        translation_step_m=options.fine_translation_step_m,
        yaw_range_rad=options.fine_yaw_range_rad,
        yaw_step_rad=options.fine_yaw_step_rad,
        options=options,
    )
    initial = fine if fine is not None else coarse
    refined = _trimmed_icp(
        target.boundary_points,
        source.boundary_points,
        initial,
        options,
    )
    if refined is None:
        return RegistrationResult(
            False,
            initial,
            math.inf,
            0.0,
            0.0,
            0.0,
            1.0,
            0,
            0.0,
            "trimmed ICP failed",
        )

    source_overlap, forward_rmse, forward_conflict, forward_count = (
        _one_way_registration_metrics(
            target,
            source,
            refined,
            options.icp_max_correspondence_m,
            options.free_space_conflict_clearance_m,
        )
    )
    reverse = inverse_pose(refined)
    target_overlap, reverse_rmse, reverse_conflict, reverse_count = (
        _one_way_registration_metrics(
            source,
            target,
            reverse,
            options.icp_max_correspondence_m,
            options.free_space_conflict_clearance_m,
        )
    )
    symmetric_overlap = min(source_overlap, target_overlap)
    finite_rmses = [value for value in (forward_rmse, reverse_rmse) if math.isfinite(value)]
    symmetric_rmse = max(finite_rmses) if finite_rmses else math.inf
    conflict = max(forward_conflict, reverse_conflict)
    correspondences = min(forward_count, reverse_count)
    score = (
        symmetric_overlap
        * math.exp(-((symmetric_rmse / max(1e-6, options.max_symmetric_rmse_m)) ** 2))
        * max(0.0, 1.0 - conflict)
        if math.isfinite(symmetric_rmse)
        else 0.0
    )

    reasons: List[str] = []
    if correspondences < options.min_correspondences:
        reasons.append("too few symmetric correspondences")
    if symmetric_overlap < options.min_symmetric_overlap:
        reasons.append("symmetric overlap below threshold")
    if symmetric_rmse > options.max_symmetric_rmse_m:
        reasons.append("symmetric RMSE above threshold")
    if conflict > options.max_free_space_conflict_ratio:
        reasons.append("free-space conflict above threshold")
    success = not reasons
    return RegistrationResult(
        success=success,
        transform_source_to_target=refined,
        symmetric_rmse_m=symmetric_rmse,
        symmetric_overlap=symmetric_overlap,
        source_overlap=source_overlap,
        target_overlap=target_overlap,
        free_space_conflict_ratio=conflict,
        correspondences=correspondences,
        score=score,
        reason="; ".join(reasons),
    )


def map_alignment_from_keyframes(
    map0_from_keyframe0: Pose2,
    keyframe0_from_keyframe1: Pose2,
    map1_from_keyframe1: Pose2,
) -> Pose2:
    """Compute ``T_map0_map1`` from a verified inter-robot place match."""

    return compose_pose(
        compose_pose(map0_from_keyframe0, keyframe0_from_keyframe1),
        inverse_pose(map1_from_keyframe1),
    )


def weighted_mean_pose(
    measurements: Sequence[AlignmentMeasurement],
) -> Tuple[Pose2, float]:
    if not measurements:
        raise ValueError("at least one measurement is required")
    weights = np.asarray([measurement.weight() for measurement in measurements])
    weight_sum = float(np.sum(weights))
    if weight_sum <= 0.0:
        weights = np.ones(len(measurements), dtype=np.float64)
        weight_sum = float(len(measurements))
    x = float(
        np.dot(weights, [m.transform_map1_to_map0.x for m in measurements]) / weight_sum
    )
    y = float(
        np.dot(weights, [m.transform_map1_to_map0.y for m in measurements]) / weight_sum
    )
    sin_yaw = float(
        np.dot(weights, [math.sin(m.transform_map1_to_map0.yaw) for m in measurements])
    )
    cos_yaw = float(
        np.dot(weights, [math.cos(m.transform_map1_to_map0.yaw) for m in measurements])
    )
    return Pose2(x, y, math.atan2(sin_yaw, cos_yaw)), weight_sum


def _measurement_close(
    measurement: AlignmentMeasurement,
    reference: Pose2,
    translation_threshold_m: float,
    yaw_threshold_rad: float,
) -> bool:
    transform = measurement.transform_map1_to_map0
    return (
        math.hypot(transform.x - reference.x, transform.y - reference.y)
        <= translation_threshold_m
        and angular_distance(transform.yaw, reference.yaw) <= yaw_threshold_rad
    )


def estimate_se2_consensus(
    measurements: Sequence[AlignmentMeasurement],
    translation_threshold_m: float = 0.40,
    yaw_threshold_rad: float = math.radians(4.0),
    min_supports: int = 2,
    min_distinct_keyframes_per_robot: int = 2,
) -> Optional[ConsensusResult]:
    """Robustly cluster repeated measurements of the single map-to-map SE(2)."""

    if len(measurements) < min_supports:
        return None
    best_cluster: Optional[List[AlignmentMeasurement]] = None
    best_weight = -1.0

    for seed in measurements:
        reference = seed.transform_map1_to_map0
        cluster = [
            measurement
            for measurement in measurements
            if _measurement_close(
                measurement,
                reference,
                translation_threshold_m,
                yaw_threshold_rad,
            )
        ]
        if not cluster:
            continue
        # One refinement around the weighted circular mean removes seed-order
        # dependence close to the clustering boundary.
        refined_mean, _ = weighted_mean_pose(cluster)
        refined_cluster = [
            measurement
            for measurement in measurements
            if _measurement_close(
                measurement,
                refined_mean,
                translation_threshold_m,
                yaw_threshold_rad,
            )
        ]
        mean, total_weight = weighted_mean_pose(refined_cluster)
        distinct0 = len({m.robot0_keyframe_id for m in refined_cluster})
        distinct1 = len({m.robot1_keyframe_id for m in refined_cluster})
        if (
            len(refined_cluster) < min_supports
            or distinct0 < min_distinct_keyframes_per_robot
            or distinct1 < min_distinct_keyframes_per_robot
        ):
            continue
        if total_weight > best_weight:
            best_weight = total_weight
            best_cluster = refined_cluster

    if best_cluster is None:
        return None
    mean, total_weight = weighted_mean_pose(best_cluster)
    translation_spread = max(
        math.hypot(
            m.transform_map1_to_map0.x - mean.x,
            m.transform_map1_to_map0.y - mean.y,
        )
        for m in best_cluster
    )
    yaw_spread = max(
        angular_distance(m.transform_map1_to_map0.yaw, mean.yaw)
        for m in best_cluster
    )
    return ConsensusResult(
        transform_map1_to_map0=mean,
        measurements=tuple(best_cluster),
        total_weight=total_weight,
        translation_spread_m=translation_spread,
        yaw_spread_rad=yaw_spread,
    )
