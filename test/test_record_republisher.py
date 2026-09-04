import math

import pytest
from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid

from co_3dto2d_mapping.record_republisher import (
    apply_planar_alignment,
    known_cell_centers,
    merge_global_grids,
)
from co_3dto2d_mapping.record_republisher_world import (
    transform_grid_to_common_frame,
)


def make_grid(
    width,
    height,
    data,
    *,
    resolution=1.0,
    origin_x=0.0,
    origin_y=0.0,
    origin_yaw=0.0,
):
    msg = OccupancyGrid()
    msg.info.resolution = resolution
    msg.info.width = width
    msg.info.height = height
    msg.info.origin.position.x = origin_x
    msg.info.origin.position.y = origin_y
    msg.info.origin.orientation.z = math.sin(0.5 * origin_yaw)
    msg.info.origin.orientation.w = math.cos(0.5 * origin_yaw)
    msg.data = data
    return msg


def quaternion_yaw(quaternion):
    return math.atan2(
        2.0
        * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0
        - 2.0
        * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


def test_known_cell_centers_preserve_free_and_occupied():
    msg = make_grid(width=3, height=1, data=[-1, 0, 100])

    assert known_cell_centers(msg, 50) == [
        (1.5, 0.5, 0),
        (2.5, 0.5, 100),
    ]


def test_known_cell_centers_apply_grid_origin_yaw():
    msg = make_grid(
        width=1,
        height=1,
        data=[0],
        origin_x=10.0,
        origin_y=20.0,
        origin_yaw=math.pi / 2.0,
    )

    cells = known_cell_centers(msg, 50)

    assert cells[0][0] == pytest.approx(9.5)
    assert cells[0][1] == pytest.approx(20.5)
    assert cells[0][2] == 0


def test_common_frame_grid_transform_preserves_odom_applied_cell_positions():
    grid = make_grid(
        width=2,
        height=1,
        data=[0, 100],
        resolution=0.5,
        origin_x=3.0,
        origin_y=-2.0,
        origin_yaw=math.radians(30.0),
    )
    alignment = (1.0, 4.0, math.radians(90.0))
    stamp = Time(sec=8, nanosec=9)

    original_cells = known_cell_centers(grid, 50)
    transformed = transform_grid_to_common_frame(
        grid,
        alignment,
        "map",
        stamp,
    )
    transformed_cells = known_cell_centers(transformed, 50)

    assert transformed.header.frame_id == "map"
    assert transformed.header.stamp.sec == 8
    assert transformed.header.stamp.nanosec == 9
    assert list(transformed.data) == list(grid.data)
    assert transformed.info.origin.position.x == pytest.approx(3.0)
    assert transformed.info.origin.position.y == pytest.approx(7.0)
    assert quaternion_yaw(transformed.info.origin.orientation) == pytest.approx(
        math.radians(120.0)
    )
    for original, common in zip(original_cells, transformed_cells):
        expected_x, expected_y = apply_planar_alignment(
            (original[0], original[1]),
            alignment,
        )
        assert common[0] == pytest.approx(expected_x)
        assert common[1] == pytest.approx(expected_y)
        assert common[2] == original[2]


def test_merge_keeps_unknown_background_and_both_known_values():
    grid = make_grid(width=2, height=1, data=[0, 100])

    merged = merge_global_grids(
        {0: grid},
        alignment=None,
        common_frame_id="map",
        merged_padding_m=1.0,
        occupied_threshold=50,
        stamp=Time(),
    )

    assert merged is not None
    assert -1 in merged.data
    assert 0 in merged.data
    assert 100 in merged.data


def test_merge_applies_nonzero_robot_alignment():
    grid = make_grid(width=1, height=1, data=[0])

    merged = merge_global_grids(
        {1: grid},
        alignment=(2.0, -1.0, math.pi / 2.0),
        common_frame_id="map",
        merged_padding_m=0.0,
        occupied_threshold=50,
        stamp=Time(),
    )

    assert merged is not None
    cells = known_cell_centers(merged, 50)
    assert cells[0][0] == pytest.approx(1.5)
    assert cells[0][1] == pytest.approx(-0.5)
    assert cells[0][2] == 0


def test_merge_occupied_wins_free_conflict():
    free_grid = make_grid(width=1, height=1, data=[0])
    occupied_grid = make_grid(width=1, height=1, data=[100])

    merged = merge_global_grids(
        {0: free_grid, 1: occupied_grid},
        alignment=(0.0, 0.0, 0.0),
        common_frame_id="map",
        merged_padding_m=0.0,
        occupied_threshold=50,
        stamp=Time(),
    )

    assert merged is not None
    assert list(merged.data) == [100]


def test_merge_skips_unaligned_nonzero_robot():
    grid = make_grid(width=1, height=1, data=[100])

    assert merge_global_grids(
        {1: grid},
        alignment=None,
        common_frame_id="map",
        merged_padding_m=0.0,
        occupied_threshold=50,
        stamp=Time(),
    ) is None
