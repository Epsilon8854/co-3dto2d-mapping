import math

import numpy as np

from co_3dto2d_mapping.occupancy_place_recognition import (
    AlignmentMeasurement,
    GridSnapshot,
    LocalSubmap,
    PolarDescriptor,
    Pose2,
    RegistrationOptions,
    angular_distance,
    build_polar_descriptor,
    compose_pose,
    estimate_se2_consensus,
    extract_local_submap,
    inverse_pose,
    map_alignment_from_keyframes,
    match_polar_descriptors,
    register_submaps,
    transform_points,
)


def make_submap(points, *, size=201, resolution=0.10):
    data = np.zeros((size, size), dtype=np.int16)
    half = size // 2
    points = np.asarray(points, dtype=np.float64)
    cols = np.rint(points[:, 0] / resolution + half).astype(int)
    rows = np.rint(points[:, 1] / resolution + half).astype(int)
    valid = (cols >= 0) & (cols < size) & (rows >= 0) & (rows < size)
    data[rows[valid], cols[valid]] = 100
    occupied = data > 50
    known = np.ones_like(occupied, dtype=bool)
    free = known & ~occupied
    boundary = occupied.copy()
    rr, cc = np.nonzero(boundary)
    boundary_points = np.column_stack(
        ((cc - half) * resolution, (rr - half) * resolution)
    ).astype(np.float64)
    radius = half * resolution
    return LocalSubmap(
        resolution=resolution,
        radius_m=radius,
        data=data,
        known=known,
        free=free,
        occupied=occupied,
        boundary=boundary,
        boundary_points=boundary_points,
        known_ratio=1.0,
    )


def asymmetric_shape():
    points = []
    # Long L-shape plus offset shelves: deliberately asymmetric under 90/180 deg.
    for x in np.arange(-4.0, 3.01, 0.10):
        points.append((x, -2.0))
    for y in np.arange(-2.0, 3.01, 0.10):
        points.append((-4.0, y))
    for x in np.arange(-1.0, 3.01, 0.10):
        points.append((x, 1.0))
    for y in np.arange(1.0, 3.01, 0.10):
        points.append((2.5, y))
    for t in np.arange(0.0, 1.01, 0.05):
        points.append((-1.5 + 1.5 * t, 2.5 - 0.7 * t))
    return np.asarray(points, dtype=np.float64)


def test_extract_local_submap_uses_keyframe_coordinates():
    resolution = 0.10
    data = np.zeros((100, 100), dtype=np.int16)
    # Grid origin is (-5, -5); world point (1, 0) is cell (60, 50).
    data[50, 60] = 100
    grid = GridSnapshot(
        resolution=resolution,
        origin=Pose2(-5.0, -5.0, 0.0),
        data=data,
    )
    submap = extract_local_submap(
        grid,
        Pose2(0.0, 0.0, math.radians(90.0)),
        radius_m=2.0,
        output_resolution_m=resolution,
        max_boundary_points=100,
    )
    assert len(submap.boundary_points) >= 1
    # A +x world point appears at -y in a frame rotated +90 degrees.
    centroid = np.mean(submap.boundary_points, axis=0)
    assert np.linalg.norm(centroid - np.asarray([0.0, -1.0])) < 0.16


def test_descriptor_shift_reports_source_to_target_yaw():
    rings = 4
    sectors = 12
    values = np.zeros((2, rings, sectors), dtype=np.float64)
    values[0, 0, 1] = 1.0
    values[0, 2, 4] = 0.6
    values[1, :, :] = np.linspace(0.1, 0.9, rings * sectors).reshape(rings, sectors)
    valid = np.ones((rings, sectors), dtype=bool)
    ring_key = np.concatenate([np.mean(values[0], axis=1), np.mean(values[1], axis=1)])
    target = PolarDescriptor(values, valid, ring_key, 1.0)

    shift = 3
    source_values = np.roll(values, -shift, axis=2)
    source_valid = np.roll(valid, -shift, axis=1)
    source = PolarDescriptor(source_values, source_valid, ring_key.copy(), 1.0)

    similarity, yaw = match_polar_descriptors(target, source, minimum_common_bins=1)
    assert similarity > 0.99
    assert angular_distance(yaw, shift * 2.0 * math.pi / sectors) < 1e-9


def test_correlative_registration_and_trimmed_icp_recover_se2():
    target_points = asymmetric_shape()
    truth = Pose2(0.80, -0.55, math.radians(14.0))
    source_points = transform_points(inverse_pose(truth), target_points)

    # Add a small set of outlier occupied cells to exercise trimming.
    outliers = np.asarray([(3.8, 3.7), (3.7, 3.5), (-2.0, 4.0), (0.0, -4.0)])
    source = make_submap(np.vstack((source_points, outliers)))
    target = make_submap(target_points)

    options = RegistrationOptions(
        coarse_translation_range_m=1.5,
        coarse_translation_step_m=0.20,
        coarse_yaw_range_rad=math.radians(30.0),
        coarse_yaw_step_rad=math.radians(2.0),
        fine_translation_range_m=0.25,
        fine_translation_step_m=0.05,
        fine_yaw_range_rad=math.radians(2.0),
        fine_yaw_step_rad=math.radians(0.5),
        search_max_distance_m=0.60,
        search_max_points=400,
        icp_max_correspondence_m=0.30,
        icp_trim_ratio=0.80,
        min_correspondences=30,
        min_symmetric_overlap=0.75,
        max_symmetric_rmse_m=0.16,
        max_free_space_conflict_ratio=0.05,
    )
    target_descriptor = build_polar_descriptor(target)
    source_descriptor = build_polar_descriptor(source)
    descriptor_similarity, descriptor_yaw = match_polar_descriptors(
        target_descriptor, source_descriptor
    )
    assert descriptor_similarity > 0.40

    result = register_submaps(
        target,
        source,
        descriptor_yaw_source_to_target=descriptor_yaw,
        options=options,
    )

    assert result.success, result.reason
    estimate = result.transform_source_to_target
    assert math.hypot(estimate.x - truth.x, estimate.y - truth.y) < 0.12
    assert angular_distance(estimate.yaw, truth.yaw) < math.radians(1.5)
    assert result.symmetric_overlap > 0.80
    assert result.symmetric_rmse_m < 0.12


def test_map_alignment_composition_has_expected_direction():
    map0_from_k0 = Pose2(5.0, 1.0, math.radians(20.0))
    map1_from_k1 = Pose2(-2.0, 4.0, math.radians(-10.0))
    map0_from_map1_truth = Pose2(7.0, -3.0, math.radians(35.0))
    k0_from_k1 = compose_pose(
        inverse_pose(map0_from_k0),
        compose_pose(map0_from_map1_truth, map1_from_k1),
    )

    recovered = map_alignment_from_keyframes(map0_from_k0, k0_from_k1, map1_from_k1)
    assert math.hypot(recovered.x - map0_from_map1_truth.x, recovered.y - map0_from_map1_truth.y) < 1e-9
    assert angular_distance(recovered.yaw, map0_from_map1_truth.yaw) < 1e-9


def measurement(transform, k0, k1, rmse=0.08):
    return AlignmentMeasurement(
        transform_map1_to_map0=transform,
        robot0_keyframe_id=k0,
        robot1_keyframe_id=k1,
        descriptor_similarity=0.90,
        symmetric_overlap=0.70,
        symmetric_rmse_m=rmse,
        free_space_conflict_ratio=0.02,
    )


def test_se2_consensus_rejects_outlier_and_requires_distinct_keyframes():
    truth = Pose2(2.0, -1.0, math.radians(8.0))
    measurements = [
        measurement(Pose2(2.04, -1.02, math.radians(8.4)), 1, 5),
        measurement(Pose2(1.97, -0.96, math.radians(7.7)), 2, 6),
        measurement(Pose2(2.01, -1.03, math.radians(8.1)), 3, 7),
        measurement(Pose2(-5.0, 6.0, math.radians(170.0)), 4, 8),
    ]
    result = estimate_se2_consensus(
        measurements,
        translation_threshold_m=0.30,
        yaw_threshold_rad=math.radians(3.0),
        min_supports=3,
        min_distinct_keyframes_per_robot=2,
    )
    assert result is not None
    assert result.support_count == 3
    assert math.hypot(
        result.transform_map1_to_map0.x - truth.x,
        result.transform_map1_to_map0.y - truth.y,
    ) < 0.08
    assert angular_distance(result.transform_map1_to_map0.yaw, truth.yaw) < math.radians(0.6)

    repeated_same_pair = [measurement(Pose2(2.0, -1.0, truth.yaw), 1, 5) for _ in range(3)]
    assert (
        estimate_se2_consensus(
            repeated_same_pair,
            min_supports=3,
            min_distinct_keyframes_per_robot=2,
        )
        is None
    )
