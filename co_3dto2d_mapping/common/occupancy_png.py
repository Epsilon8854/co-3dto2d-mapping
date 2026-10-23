"""Encode ROS occupancy-grid cells as a grayscale PNG image."""

from __future__ import annotations

import math
import struct
from typing import Final, NamedTuple, Sequence, Tuple, Union
import zlib


_PNG_SIGNATURE: Final = b"\x89PNG\r\n\x1a\n"
Pixel = Tuple[int, int]
RgbColor = Tuple[int, int, int]
WorldPoint = Tuple[float, float]


class OccupancyGridGeometry(NamedTuple):
    """World-space geometry needed to place points on an occupancy image."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float


class TrajectoryOverlay(NamedTuple):
    """One trajectory rendered with a distinct RGB color."""

    points: Tuple[Pixel, ...]
    color: RgbColor


class OccupancyGridEncodingError(Exception):
    """Raised when occupancy cells cannot form a valid PNG image."""

    def __init__(self, reason: str) -> None:
        self.reason = reason
        super().__init__(reason)

    def __str__(self) -> str:
        return self.reason


def _chunk(chunk_type: bytes, data: bytes) -> bytes:
    checksum = zlib.crc32(data, zlib.crc32(chunk_type))
    return struct.pack(">I", len(data)) + chunk_type + data + struct.pack(">I", checksum)


def _occupancy_gray(value: int) -> int:
    if value < 0:
        return 205
    clamped = min(100, max(0, value))
    return 254 - round(254 * clamped / 100)


def _validate_grid_dimensions(width: int, height: int, occupancy: Sequence[int]) -> None:
    if width <= 0 or height <= 0:
        raise OccupancyGridEncodingError(
            reason="occupancy image dimensions must be positive"
        )
    if len(occupancy) != width * height:
        raise OccupancyGridEncodingError(
            reason="occupancy image data does not match its dimensions"
        )


def _png(
    color_type: int, width: int, height: int, rows: Union[bytes, bytearray]
) -> bytes:
    header = struct.pack(">IIBBBBB", width, height, 8, color_type, 0, 0, 0)
    return _PNG_SIGNATURE + _chunk(b"IHDR", header) + _chunk(
        b"IDAT", zlib.compress(rows)
    ) + _chunk(b"IEND", b"")


def occupancy_grid_png(
    *, width: int, height: int, occupancy: Sequence[int]
) -> bytes:
    """Return a standards-compliant top-down PNG for a ROS occupancy grid."""
    _validate_grid_dimensions(width, height, occupancy)

    rows = bytearray()
    for row in range(height - 1, -1, -1):
        rows.append(0)
        offset = row * width
        rows.extend(_occupancy_gray(value) for value in occupancy[offset : offset + width])

    return _png(0, width, height, rows)


def trajectory_pixels(
    *, geometry: OccupancyGridGeometry, points: Sequence[WorldPoint]
) -> Tuple[Pixel, ...]:
    """Convert in-bounds world points to top-down PNG pixel coordinates."""
    if geometry.width <= 0 or geometry.height <= 0 or geometry.resolution <= 0.0:
        raise OccupancyGridEncodingError(
            reason="occupancy grid geometry must have positive dimensions and resolution"
        )

    cosine = math.cos(geometry.origin_yaw)
    sine = math.sin(geometry.origin_yaw)
    pixels = []
    for point_x, point_y in points:
        offset_x = point_x - geometry.origin_x
        offset_y = point_y - geometry.origin_y
        local_x = cosine * offset_x + sine * offset_y
        local_y = -sine * offset_x + cosine * offset_y
        column = math.floor(local_x / geometry.resolution)
        row = math.floor(local_y / geometry.resolution)
        if 0 <= column < geometry.width and 0 <= row < geometry.height:
            pixels.append((column, geometry.height - 1 - row))
    return tuple(pixels)


def _set_rgb_pixel(
    pixels: bytearray, width: int, height: int, point: Pixel, color: RgbColor
) -> None:
    column, row = point
    if not (0 <= column < width and 0 <= row < height):
        return
    offset = (row * width + column) * 3
    pixels[offset : offset + 3] = bytes(color)


def _draw_line(
    pixels: bytearray,
    width: int,
    height: int,
    start: Pixel,
    end: Pixel,
    color: RgbColor,
) -> None:
    column, row = start
    target_column, target_row = end
    column_step = 1 if column < target_column else -1
    row_step = 1 if row < target_row else -1
    column_delta = abs(target_column - column)
    row_delta = -abs(target_row - row)
    error = column_delta + row_delta

    while True:
        _set_rgb_pixel(pixels, width, height, (column, row), color)
        if column == target_column and row == target_row:
            return
        doubled_error = 2 * error
        if doubled_error >= row_delta:
            error += row_delta
            column += column_step
        if doubled_error <= column_delta:
            error += column_delta
            row += row_step


def occupancy_grid_png_with_trajectories(
    *,
    width: int,
    height: int,
    occupancy: Sequence[int],
    trajectories: Sequence[TrajectoryOverlay],
) -> bytes:
    """Return an RGB occupancy PNG with colored trajectory polylines."""
    _validate_grid_dimensions(width, height, occupancy)
    pixels = bytearray()
    for row in range(height - 1, -1, -1):
        offset = row * width
        for occupancy_value in occupancy[offset : offset + width]:
            grayscale = _occupancy_gray(occupancy_value)
            pixels.extend((grayscale, grayscale, grayscale))

    for trajectory in trajectories:
        if not trajectory.points:
            continue
        _set_rgb_pixel(pixels, width, height, trajectory.points[0], trajectory.color)
        for previous, current in zip(trajectory.points, trajectory.points[1:]):
            _draw_line(pixels, width, height, previous, current, trajectory.color)

    rows = bytearray()
    row_width = width * 3
    for row in range(height):
        rows.append(0)
        offset = row * row_width
        rows.extend(pixels[offset : offset + row_width])
    return _png(2, width, height, rows)
