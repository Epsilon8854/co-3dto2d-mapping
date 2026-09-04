import math

import numpy as np
import pytest

from co_3dto2d_mapping.occupancy_submap import (
    GridGeometry,
    extract_local_patch,
    occupied_boundary_mask,
)
from co_3dto2d_mapping.planar_transform_utils import compose_planar, invert_planar
from co_3dto2d_mapping.polar_occupancy_context import (
    PolarContextConfig,
    build_polar_context,
    match_polar_context,
    rank_ring_candidates,
)
from co_3dto2d_mapping.se2_map_registration import (
    RegistrationConfig,
    register_submaps,
)


def make_world(resolution=0.1, size=260):
    grid = np.full((size, size), -1, dtype=np.int8)
    grid[20:240, 20:240] = 0
    grid[55:60, 35:205] = 100
    grid[55:190, 35:40] = 100
    grid[125:130, 35:160] = 100
    grid[125:205, 155:160] = 100
    grid[185:190, 90:230] = 100
    grid[80:150, 205:210] = 100
    grid[90:100, 90:100] = 100
    grid[155:168, 65:78] = 100
    geometry = GridGeometry(
        resolution=resolution,
        width=size,
        height=size,
        origin_x=-13.0,
        origin_y=-13.0,
    )
    return grid, geometry


def registration_config(**overrides):
    values = dict(
        coarse_translation_range_m=2.0,
        coarse_translation_step_m=0.25,
        coarse_yaw_range_rad=math.radians(8.0),
        coarse_yaw_step_rad=math.radians(2.0),
        fine_translation_range_m=0.25,
        fine_translation_step_m=0.05,
        fine_yaw_range_rad=math.radians(2.0),
        fine_yaw_step_rad=math.radians(0.5),
        search_max_points=500,
        min_correspondences=60,
        min_symmetric_overlap=0.35,
        max_symmetric_rmse_m=0.22,
        max_free_conflict_ratio=0.18,
    )
    values.update(overrides)
    return RegistrationConfig(**values)


def test_extract_patch_respects_keyframe_pose_and_boundary():
    grid, geometry = make_world()
    patch = extract_local_patch(grid, geometry, (0.0, 0.0, 0.0), 10.0)

    assert patch.known_ratio > 0.8
    assert patch.occupied_boundary_count > 100
    assert occupied_boundary_mask(patch.grid).sum() == patch.occupied_boundary_count


def test_polar_context_estimates_target_from_source_yaw_for_shared_place_center():
    grid, geometry = make_world()
    target_pose = (0.0, 0.0, math.radians(-8.0))
    source_pose = (0.0, 0.0, math.radians(16.0))
    target = extract_local_patch(grid, geometry, target_pose, 10.0)
    source = extract_local_patch(grid, geometry, source_pose, 10.0)
    config = PolarContextConfig(max_radius_m=10.0, num_rings=20, num_sectors=72)

    target_context = build_polar_context(target, config)
    source_context = build_polar_context(source, config)
    match = match_polar_context(target_context, source_context, config)
    expected = compose_planar(invert_planar(target_pose), source_pose)[2]
    error = abs(
        math.atan2(
            math.sin(match.yaw_rad - expected),
            math.cos(match.yaw_rad - expected),
        )
    )

    assert error <= math.radians(6.0)
    assert match.distance < 0.15


def test_ring_key_retrieval_prefers_same_place_over_unrelated_patch():
    grid, geometry = make_world()
    query = extract_local_patch(grid, geometry, (0.0, 0.0, 0.0), 9.0)
    same_place = extract_local_patch(
        grid, geometry, (0.4, -0.2, math.radians(4.0)), 9.0
    )

    unrelated_grid = np.full_like(grid, -1)
    unrelated_grid[20:240, 20:240] = 0
    unrelated_grid[40:45, 40:220] = 100
    unrelated_grid[210:215, 40:220] = 100
    unrelated_grid[40:215, 125:130] = 100
    unrelated = extract_local_patch(
        unrelated_grid, geometry, (0.0, 0.0, 0.0), 9.0
    )
    config = PolarContextConfig(max_radius_m=9.0, num_rings=18, num_sectors=60)

    query_context = build_polar_context(query, config)
    ranked = rank_ring_candidates(
        query_context,
        [build_polar_context(unrelated, config), build_polar_context(same_place, config)],
        2,
    )

    assert ranked[0][0] == 1
    assert ranked[0][1] < ranked[1][1]


def test_registration_recovers_keyframe_transform_from_descriptor_and_odom_yaws():
    grid, geometry = make_world()
    target_pose = (-0.8, 0.5, math.radians(-5.0))
    source_pose = (0.9, -0.3, math.radians(14.0))
    target = extract_local_patch(grid, geometry, target_pose, 9.0)
    source = extract_local_patch(grid, geometry, source_pose, 9.0)
    descriptor_config = PolarContextConfig(
        max_radius_m=9.0, num_rings=18, num_sectors=72
    )
    descriptor_match = match_polar_context(
        build_polar_context(target, descriptor_config),
        build_polar_context(source, descriptor_config),
        descriptor_config,
    )
    expected = compose_planar(invert_planar(target_pose), source_pose)

    hypotheses = [descriptor_match.yaw_rad, expected[2]]
    results = [
        register_submaps(target, source, yaw, registration_config())
        for yaw in hypotheses
    ]
    result = max(
        results,
        key=lambda item: (
            int(item.accepted),
            item.symmetric_overlap,
            -item.symmetric_rmse_m,
            -item.free_conflict_ratio,
        ),
    )

    assert result.accepted, result
    assert result.transform[0] == pytest.approx(expected[0], abs=0.16)
    assert result.transform[1] == pytest.approx(expected[1], abs=0.16)
    angle_error = abs(
        math.atan2(
            math.sin(result.transform[2] - expected[2]),
            math.cos(result.transform[2] - expected[2]),
        )
    )
    assert angle_error < math.radians(2.0)
    assert result.free_conflict_ratio < 0.10


def test_geometric_verification_rejects_incompatible_free_space():
    grid, geometry = make_world()
    target = extract_local_patch(grid, geometry, (0.0, 0.0, 0.0), 8.0)

    incompatible = np.full_like(grid, -1)
    incompatible[20:240, 20:240] = 0
    incompatible[70:190, 70:190] = 100
    incompatible[100:160, 100:160] = 0
    source = extract_local_patch(
        incompatible, geometry, (0.0, 0.0, 0.0), 8.0
    )

    result = register_submaps(
        target,
        source,
        0.0,
        registration_config(
            coarse_translation_range_m=1.0,
            min_symmetric_overlap=0.45,
            max_free_conflict_ratio=0.12,
        ),
    )

    assert not result.accepted
    assert result.reason in {
        "low_symmetric_overlap",
        "high_symmetric_rmse",
        "free_space_conflict",
        "insufficient_correspondences",
    }
