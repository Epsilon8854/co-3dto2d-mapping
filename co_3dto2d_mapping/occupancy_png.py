"""Encode ROS occupancy-grid cells as a grayscale PNG image."""

from __future__ import annotations

import struct
from typing import Final, Sequence
import zlib


_PNG_SIGNATURE: Final = b"\x89PNG\r\n\x1a\n"


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


def occupancy_grid_png(
    *, width: int, height: int, occupancy: Sequence[int]
) -> bytes:
    """Return a standards-compliant top-down PNG for a ROS occupancy grid."""
    if width <= 0 or height <= 0:
        raise OccupancyGridEncodingError(
            reason="occupancy image dimensions must be positive"
        )
    if len(occupancy) != width * height:
        raise OccupancyGridEncodingError(
            reason="occupancy image data does not match its dimensions"
        )

    rows = bytearray()
    for row in range(height - 1, -1, -1):
        rows.append(0)
        offset = row * width
        rows.extend(_occupancy_gray(value) for value in occupancy[offset : offset + width])

    header = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
    return _PNG_SIGNATURE + _chunk(b"IHDR", header) + _chunk(b"IDAT", zlib.compress(rows)) + _chunk(b"IEND", b"")
