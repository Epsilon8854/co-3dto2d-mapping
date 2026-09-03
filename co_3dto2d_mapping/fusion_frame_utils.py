"""Pure helpers for common-frame map fusion."""

from __future__ import annotations

from typing import Iterable, Tuple

import numpy as np


def seed_unknown_observations(
    fusion,
    cols: Iterable[int],
    rows: Iterable[int],
    values: Iterable[int],
) -> int:
    """Fill only currently unknown fusion cells from a persistent global map.

    This is safe to call for every growing global-map update: it adds newly
    explored space, but it cannot reinsert an old occupied cell after live local
    observations have already cleared that common-frame cell.
    """

    cols_array, rows_array, values_array = fusion._normalize_observations(
        cols, rows, values
    )
    if cols_array.size == 0:
        return 0
    fusion._ensure_bounds(cols_array, rows_array)
    local_cols = cols_array - fusion.origin_col
    local_rows = rows_array - fusion.origin_row
    unknown = fusion.data[local_rows, local_cols] < 0
    if not np.any(unknown):
        return 0
    fusion.data[local_rows[unknown], local_cols[unknown]] = values_array[unknown]
    return int(np.count_nonzero(unknown))


def reference_transform_for_robot(
    reference_robot: int | None,
    alignment_map1_to_map0: Tuple[float, float, float] | None,
    robot_id: int,
) -> Tuple[bool, Tuple[float, float, float] | None]:
    """Return whether a robot can be represented and its common transform.

    ``None`` transform means identity, while ``False`` means unavailable.
    """

    if reference_robot is None:
        return False, None
    if reference_robot == 0:
        if robot_id == 0:
            return True, None
        return (
            (True, alignment_map1_to_map0)
            if alignment_map1_to_map0 is not None
            else (False, None)
        )
    if reference_robot == 1:
        if robot_id == 1:
            return True, None
        # We intentionally do not expose r0 in an r1-anchored temporary frame.
        # Once alignment arrives the reference switches to canonical r0/map.
        return False, None
    raise ValueError("reference_robot must be 0, 1, or None")
