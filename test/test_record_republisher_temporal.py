import math

import pytest
from nav_msgs.msg import OccupancyGrid

from co_3dto2d_mapping.record_republisher_temporal import (
    occupancy_stamp_key,
    quantized_grid_observations,
)


def make_grid(data, *, width=2, height=1, resolution=1.0):
    msg = OccupancyGrid()
    msg.header.stamp.sec = 12
    msg.header.stamp.nanosec = 34
    msg.info.width = width
    msg.info.height = height
    msg.info.resolution = resolution
    msg.info.origin.orientation.w = 1.0
    msg.data = data
    return msg


def test_stamp_key_deduplicates_republished_same_local_frame():
    msg = make_grid([0, 100])
    assert occupancy_stamp_key(msg) == (12, 34, 2, 1)
    assert occupancy_stamp_key(msg) == occupancy_stamp_key(msg)


def test_quantization_preserves_free_and_occupied_cells():
    msg = make_grid([0, 100])
    cols, rows, values = quantized_grid_observations(msg, 1.0, 50)

    assert cols.tolist() == [0, 1]
    assert rows.tolist() == [0, 0]
    assert values.tolist() == [0, 100]


def test_quantization_applies_robot_alignment_before_common_grid_index():
    msg = make_grid([100], width=1, height=1)
    cols, rows, values = quantized_grid_observations(
        msg,
        1.0,
        50,
        alignment=(2.0, -1.0, math.pi / 2.0),
    )

    assert cols.tolist() == [1]
    assert rows.tolist() == [-1]
    assert values.tolist() == [100]


def test_quantization_applies_rotated_grid_origin():
    msg = make_grid([0], width=1, height=1)
    msg.info.origin.position.x = 10.0
    msg.info.origin.position.y = 20.0
    msg.info.origin.orientation.z = math.sin(math.pi / 4.0)
    msg.info.origin.orientation.w = math.cos(math.pi / 4.0)

    cols, rows, values = quantized_grid_observations(msg, 0.5, 50)

    assert cols.tolist() == [19]
    assert rows.tolist() == [41]
    assert values.tolist() == [0]


def test_invalid_grid_returns_empty_observation():
    msg = make_grid([0], width=2, height=1)
    cols, rows, values = quantized_grid_observations(msg, 1.0, 50)

    assert cols.size == 0
    assert rows.size == 0
    assert values.size == 0
