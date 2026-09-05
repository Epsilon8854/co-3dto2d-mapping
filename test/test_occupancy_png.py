from __future__ import annotations

import struct
import zlib
from pathlib import Path

from co_3dto2d_mapping.occupancy_png import occupancy_grid_png


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


def test_exporter_accepts_the_mapper_volatile_occupancy_topics() -> None:
    # Given: the mapper publishes OccupancyGrid with the default volatile QoS.
    exporter = (PACKAGE / "co_3dto2d_mapping" / "occupancy_png_exporter.py").read_text(
        encoding="utf-8"
    )

    # When: the exporter subscribes to the final global maps.

    # Then: it requests compatible volatile durability and can receive updates.
    assert "qos.durability = DurabilityPolicy.VOLATILE" in exporter
