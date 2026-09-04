import numpy as np

from co_3dto2d_mapping.temporal_fusion import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    TemporalFusionConfig,
    TemporalFusionGrid,
    expand_free_observations,
)


def value_at(grid, col, row):
    return int(grid.data[row - grid.origin_row, col - grid.origin_col])


def test_seed_occupied_is_cleared_by_four_new_free_frames():
    grid = TemporalFusionGrid(
        0.05,
        TemporalFusionConfig(free_clear_count=4, occupied_confirm_count=3),
        padding_m=0.0,
    )
    grid.seed([10], [20], [OCCUPIED])

    for _ in range(3):
        stats = grid.observe([10], [20], [FREE])
        assert stats.cleared_cells == 0
        assert value_at(grid, 10, 20) == OCCUPIED

    stats = grid.observe([10], [20], [FREE])
    assert stats.cleared_cells == 1
    assert value_at(grid, 10, 20) == FREE


def test_stale_global_occupied_is_not_reapplied_after_live_clear():
    grid = TemporalFusionGrid(0.05, padding_m=0.0)
    grid.seed([0], [0], [OCCUPIED])
    for _ in range(4):
        grid.observe([0], [0], [FREE])

    assert value_at(grid, 0, 0) == FREE
    # Persistent source global maps are intentionally seeded only once by the
    # runtime wrapper; a later live frame, not the stale map, owns the state.
    assert grid.frame_index == 4


def test_new_obstacle_requires_repeated_live_frames():
    grid = TemporalFusionGrid(
        0.05,
        TemporalFusionConfig(free_clear_count=4, occupied_confirm_count=3),
        padding_m=0.0,
    )
    grid.seed([1], [2], [FREE])

    grid.observe([1], [2], [OCCUPIED])
    assert value_at(grid, 1, 2) == FREE
    grid.observe([1], [2], [OCCUPIED])
    assert value_at(grid, 1, 2) == FREE
    stats = grid.observe([1], [2], [OCCUPIED])
    assert stats.committed_occupied_cells == 1
    assert value_at(grid, 1, 2) == OCCUPIED


def test_occupied_wins_only_inside_one_observation_frame():
    grid = TemporalFusionGrid(
        1.0,
        TemporalFusionConfig(free_clear_count=2, occupied_confirm_count=1),
        padding_m=0.0,
    )
    grid.observe([3, 3], [4, 4], [FREE, OCCUPIED])
    assert value_at(grid, 3, 4) == OCCUPIED

    grid.observe([3], [4], [FREE])
    assert value_at(grid, 3, 4) == OCCUPIED
    grid.observe([3], [4], [FREE])
    assert value_at(grid, 3, 4) == FREE


def test_expansion_preserves_values_and_evidence():
    grid = TemporalFusionGrid(
        0.1,
        TemporalFusionConfig(free_clear_count=2, occupied_confirm_count=1),
        padding_m=0.0,
    )
    grid.observe([5], [5], [OCCUPIED])
    grid.observe([-20], [30], [FREE])

    assert value_at(grid, 5, 5) == OCCUPIED
    assert value_at(grid, -20, 30) == FREE
    assert grid.data.dtype == np.int8


def test_unknown_stays_unknown_until_observed():
    grid = TemporalFusionGrid(1.0, padding_m=1.0)
    grid.observe([0], [0], [FREE])

    assert value_at(grid, 0, 0) == FREE
    assert UNKNOWN in grid.data


def test_free_observation_expansion_uses_manhattan_footprint():
    cols, rows, values = expand_free_observations(
        [10, 20], [10, 20], [FREE, OCCUPIED], 1
    )
    samples = set(zip(cols.tolist(), rows.tolist(), values.tolist()))

    assert (10, 10, FREE) in samples
    assert (9, 10, FREE) in samples
    assert (11, 10, FREE) in samples
    assert (10, 9, FREE) in samples
    assert (10, 11, FREE) in samples
    assert (9, 9, FREE) not in samples
    assert (20, 20, OCCUPIED) in samples


def test_expanded_free_does_not_beat_same_frame_occupied_endpoint():
    grid = TemporalFusionGrid(1.0, TemporalFusionConfig(occupied_confirm_count=1), 0.0)
    cols, rows, values = expand_free_observations(
        [0, 1], [0, 0], [FREE, OCCUPIED], 1
    )
    grid.observe(cols, rows, values)

    assert value_at(grid, 1, 0) == OCCUPIED
