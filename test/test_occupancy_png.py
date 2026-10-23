from __future__ import annotations

import math
import struct
import zlib
from pathlib import Path

from co_3dto2d_mapping.common.occupancy_png import (
    OccupancyGridGeometry,
    TrajectoryOverlay,
    occupancy_grid_png,
    occupancy_grid_png_with_trajectories,
    trajectory_pixels,
)


PACKAGE = Path(__file__).resolve().parents[1]


def _png_chunks(image: bytes) -> list[tuple[bytes, bytes]]:
    offset = 8
    chunks: list[tuple[bytes, bytes]] = []
    while offset < len(image):
        length = struct.unpack(">I", image[offset : offset + 4])[0]
        chunk_type = image[offset + 4 : offset + 8]
        data_start = offset + 8
        data_end = data_start + length
        chunks.append((chunk_type, image[data_start:data_end]))
        offset = data_end + 4
    return chunks


def test_occupancy_grid_png_uses_ros_map_orientation_and_standard_colors() -> None:
    # Given: a two-row ROS occupancy grid whose first row is the map bottom.
    occupancy = (0, 100, -1, 50)

    # When: it is encoded as a PNG image.
    image = occupancy_grid_png(width=2, height=2, occupancy=occupancy)

    # Then: the PNG top row is the map's upper row with free/occupied/unknown colors.
    chunks = _png_chunks(image)
    ihdr = next(data for kind, data in chunks if kind == b"IHDR")
    compressed = b"".join(data for kind, data in chunks if kind == b"IDAT")
    assert image.startswith(b"\x89PNG\r\n\x1a\n")
    assert struct.unpack(">II", ihdr[:8]) == (2, 2)
    assert zlib.decompress(compressed) == bytes((0, 205, 127, 0, 254, 0))


def test_trajectory_png_draws_a_colored_path_over_the_occupancy_grid() -> None:
    # Given: a free occupancy grid and a diagonal trajectory in PNG pixel space.
    overlay = TrajectoryOverlay(points=((0, 0), (2, 2)), color=(255, 0, 0))

    # When: the trajectory image is encoded.
    image = occupancy_grid_png_with_trajectories(
        width=3,
        height=3,
        occupancy=(0,) * 9,
        trajectories=(overlay,),
    )

    # Then: every path pixel is red while non-path cells retain the free-space color.
    chunks = _png_chunks(image)
    ihdr = next(data for kind, data in chunks if kind == b"IHDR")
    compressed = b"".join(data for kind, data in chunks if kind == b"IDAT")
    rows = zlib.decompress(compressed)
    assert struct.unpack(">IIBBBBB", ihdr) == (3, 3, 8, 2, 0, 0, 0)
    assert rows == bytes(
        (
            0,
            255,
            0,
            0,
            254,
            254,
            254,
            254,
            254,
            254,
            0,
            254,
            254,
            254,
            255,
            0,
            0,
            254,
            254,
            254,
            0,
            254,
            254,
            254,
            254,
            254,
            254,
            255,
            0,
            0,
        )
    )


def test_trajectory_pixels_respect_the_grid_origin_and_rotation() -> None:
    # Given: a 90-degree-rotated occupancy grid in a nonzero world origin.
    geometry = OccupancyGridGeometry(
        width=3,
        height=3,
        resolution=1.0,
        origin_x=10.0,
        origin_y=20.0,
        origin_yaw=math.pi / 2.0,
    )

    # When: world-space odometry positions are projected into the map image.
    pixels = trajectory_pixels(
        geometry=geometry,
        points=((9.5, 20.5), (7.5, 22.5), (10.5, 20.5)),
    )

    # Then: only in-bounds points remain, with ROS map rows flipped for PNG.
    assert pixels == ((0, 2), (2, 0))
