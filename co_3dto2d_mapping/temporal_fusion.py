"""Temporal occupancy fusion for current-frame local occupancy observations.

The per-robot global maps are useful as a bootstrap, but they do not carry
per-cell observation age.  Re-merging those maps with an occupied-wins policy
therefore makes an old occupied cell from either robot permanent.  This module
keeps temporal evidence in the common frame and updates it only from new local
observation frames.
"""

from dataclasses import dataclass
from typing import Iterable, Tuple

import numpy as np

UNKNOWN = -1
FREE = 0
OCCUPIED = 100


@dataclass(frozen=True)
class TemporalFusionConfig:
    free_clear_count: int = 4
    occupied_confirm_count: int = 3
    counter_decay: int = 1
    evidence_timeout_frames: int = 30

    def validated(self) -> "TemporalFusionConfig":
        return TemporalFusionConfig(
            free_clear_count=max(1, int(self.free_clear_count)),
            occupied_confirm_count=max(1, int(self.occupied_confirm_count)),
            counter_decay=max(0, int(self.counter_decay)),
            evidence_timeout_frames=max(0, int(self.evidence_timeout_frames)),
        )


@dataclass(frozen=True)
class FusionUpdateStats:
    observed_cells: int = 0
    free_observed_cells: int = 0
    occupied_observed_cells: int = 0
    cleared_cells: int = 0
    committed_occupied_cells: int = 0


def expand_free_observations(
    cols: Iterable[int],
    rows: Iterable[int],
    values: Iterable[int],
    radius_cells: int,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Give free rays a small Manhattan footprint for grid/scan jitter.

    Occupied samples are kept unchanged.  The caller still deduplicates the
    result with occupied priority, so a measured endpoint cannot be erased by
    the expanded free footprint in the same frame.
    """

    cols_array = np.asarray(cols, dtype=np.int64).reshape(-1)
    rows_array = np.asarray(rows, dtype=np.int64).reshape(-1)
    values_array = np.asarray(values, dtype=np.int16).reshape(-1)
    if not (cols_array.size == rows_array.size == values_array.size):
        raise ValueError("cols, rows, and values must have the same length")
    radius_cells = max(0, int(radius_cells))
    if radius_cells == 0 or cols_array.size == 0:
        return cols_array, rows_array, values_array.astype(np.int8)

    free_mask = values_array <= 50
    free_cols = cols_array[free_mask]
    free_rows = rows_array[free_mask]
    occupied_cols = cols_array[~free_mask]
    occupied_rows = rows_array[~free_mask]
    col_parts = [occupied_cols]
    row_parts = [occupied_rows]
    value_parts = [np.full(occupied_cols.size, OCCUPIED, dtype=np.int8)]
    for delta_row in range(-radius_cells, radius_cells + 1):
        remaining = radius_cells - abs(delta_row)
        for delta_col in range(-remaining, remaining + 1):
            col_parts.append(free_cols + delta_col)
            row_parts.append(free_rows + delta_row)
            value_parts.append(np.full(free_cols.size, FREE, dtype=np.int8))
    return (
        np.concatenate(col_parts),
        np.concatenate(row_parts),
        np.concatenate(value_parts),
    )


class TemporalFusionGrid:
    """Dynamically growing dense temporal occupancy grid.

    Coordinates passed to ``seed`` and ``observe`` are integer cells in a
    common, zero-origin lattice (``floor(world_coordinate / resolution)``).
    The backing arrays grow as needed and preserve all accumulated evidence.
    """

    def __init__(
        self,
        resolution: float,
        config: TemporalFusionConfig = TemporalFusionConfig(),
        padding_m: float = 1.0,
    ) -> None:
        resolution = float(resolution)
        if not np.isfinite(resolution) or resolution <= 0.0:
            raise ValueError("resolution must be a positive finite value")
        self.resolution = resolution
        self.config = config.validated()
        self.padding_cells = max(0, int(np.ceil(float(padding_m) / resolution)))
        self.origin_col = 0
        self.origin_row = 0
        self.data = np.empty((0, 0), dtype=np.int8)
        self.free_counts = np.empty((0, 0), dtype=np.uint16)
        self.occupied_counts = np.empty((0, 0), dtype=np.uint16)
        self.last_observed = np.empty((0, 0), dtype=np.uint32)
        self.frame_index = 0

    @property
    def ready(self) -> bool:
        return self.data.size > 0 and np.any(self.data >= 0)

    def reset(self) -> None:
        self.origin_col = 0
        self.origin_row = 0
        self.data = np.empty((0, 0), dtype=np.int8)
        self.free_counts = np.empty((0, 0), dtype=np.uint16)
        self.occupied_counts = np.empty((0, 0), dtype=np.uint16)
        self.last_observed = np.empty((0, 0), dtype=np.uint32)
        self.frame_index = 0

    @staticmethod
    def _normalize_observations(
        cols: Iterable[int], rows: Iterable[int], values: Iterable[int]
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        cols_array = np.asarray(cols, dtype=np.int64).reshape(-1)
        rows_array = np.asarray(rows, dtype=np.int64).reshape(-1)
        values_array = np.asarray(values, dtype=np.int16).reshape(-1)
        if not (
            cols_array.size == rows_array.size == values_array.size
        ):
            raise ValueError("cols, rows, and values must have the same length")
        if cols_array.size == 0:
            return cols_array, rows_array, np.empty(0, dtype=np.int8)

        normalized_values = np.where(values_array > 50, OCCUPIED, FREE).astype(
            np.int8
        )
        coordinates = np.column_stack((cols_array, rows_array))
        unique_coordinates, inverse = np.unique(
            coordinates, axis=0, return_inverse=True
        )
        # If transformed source cells collapse into one common-frame cell,
        # occupied wins only within that one observation frame.
        unique_values = np.zeros(unique_coordinates.shape[0], dtype=np.int8)
        np.maximum.at(unique_values, inverse, normalized_values)
        return (
            unique_coordinates[:, 0],
            unique_coordinates[:, 1],
            unique_values,
        )

    def _ensure_bounds(self, cols: np.ndarray, rows: np.ndarray) -> None:
        if cols.size == 0:
            return
        requested_min_col = int(np.min(cols)) - self.padding_cells
        requested_max_col = int(np.max(cols)) + self.padding_cells
        requested_min_row = int(np.min(rows)) - self.padding_cells
        requested_max_row = int(np.max(rows)) + self.padding_cells

        if self.data.size == 0:
            width = requested_max_col - requested_min_col + 1
            height = requested_max_row - requested_min_row + 1
            self.origin_col = requested_min_col
            self.origin_row = requested_min_row
            self.data = np.full((height, width), UNKNOWN, dtype=np.int8)
            self.free_counts = np.zeros((height, width), dtype=np.uint16)
            self.occupied_counts = np.zeros((height, width), dtype=np.uint16)
            self.last_observed = np.zeros((height, width), dtype=np.uint32)
            return

        current_height, current_width = self.data.shape
        current_max_col = self.origin_col + current_width - 1
        current_max_row = self.origin_row + current_height - 1
        new_min_col = min(self.origin_col, requested_min_col)
        new_max_col = max(current_max_col, requested_max_col)
        new_min_row = min(self.origin_row, requested_min_row)
        new_max_row = max(current_max_row, requested_max_row)
        if (
            new_min_col == self.origin_col
            and new_max_col == current_max_col
            and new_min_row == self.origin_row
            and new_max_row == current_max_row
        ):
            return

        new_width = new_max_col - new_min_col + 1
        new_height = new_max_row - new_min_row + 1
        row_offset = self.origin_row - new_min_row
        col_offset = self.origin_col - new_min_col
        target = np.s_[
            row_offset : row_offset + current_height,
            col_offset : col_offset + current_width,
        ]

        new_data = np.full((new_height, new_width), UNKNOWN, dtype=np.int8)
        new_free_counts = np.zeros((new_height, new_width), dtype=np.uint16)
        new_occupied_counts = np.zeros((new_height, new_width), dtype=np.uint16)
        new_last_observed = np.zeros((new_height, new_width), dtype=np.uint32)
        new_data[target] = self.data
        new_free_counts[target] = self.free_counts
        new_occupied_counts[target] = self.occupied_counts
        new_last_observed[target] = self.last_observed

        self.origin_col = new_min_col
        self.origin_row = new_min_row
        self.data = new_data
        self.free_counts = new_free_counts
        self.occupied_counts = new_occupied_counts
        self.last_observed = new_last_observed

    def seed(
        self, cols: Iterable[int], rows: Iterable[int], values: Iterable[int]
    ) -> None:
        """Bootstrap from persistent global maps without adding evidence.

        Occupied wins a bootstrap conflict.  Each robot is expected to be
        seeded at most once; subsequent changes must arrive through ``observe``.
        """

        cols_array, rows_array, values_array = self._normalize_observations(
            cols, rows, values
        )
        if cols_array.size == 0:
            return
        self._ensure_bounds(cols_array, rows_array)
        local_cols = cols_array - self.origin_col
        local_rows = rows_array - self.origin_row
        current = self.data[local_rows, local_cols]
        occupied = values_array == OCCUPIED
        free_into_unknown = (values_array == FREE) & (current == UNKNOWN)
        self.data[local_rows[occupied], local_cols[occupied]] = OCCUPIED
        self.data[
            local_rows[free_into_unknown], local_cols[free_into_unknown]
        ] = FREE

    def observe(
        self, cols: Iterable[int], rows: Iterable[int], values: Iterable[int]
    ) -> FusionUpdateStats:
        cols_array, rows_array, values_array = self._normalize_observations(
            cols, rows, values
        )
        if cols_array.size == 0:
            return FusionUpdateStats()
        self._ensure_bounds(cols_array, rows_array)

        self.frame_index += 1
        if self.frame_index >= np.iinfo(np.uint32).max:
            self.frame_index = 1
            self.last_observed.fill(0)
            self.free_counts.fill(0)
            self.occupied_counts.fill(0)

        local_cols = cols_array - self.origin_col
        local_rows = rows_array - self.origin_row
        previous_values = self.data[local_rows, local_cols].copy()
        previous_last = self.last_observed[local_rows, local_cols]
        timeout = self.config.evidence_timeout_frames
        if timeout > 0:
            stale = (previous_last != 0) & (
                self.frame_index - previous_last.astype(np.int64) > timeout
            )
            if np.any(stale):
                stale_rows = local_rows[stale]
                stale_cols = local_cols[stale]
                self.free_counts[stale_rows, stale_cols] = 0
                self.occupied_counts[stale_rows, stale_cols] = 0

        occupied_mask = values_array == OCCUPIED
        free_mask = ~occupied_mask
        decay = self.config.counter_decay

        if np.any(occupied_mask):
            target_rows = local_rows[occupied_mask]
            target_cols = local_cols[occupied_mask]
            counts = self.occupied_counts[target_rows, target_cols].astype(
                np.uint32
            )
            counts = np.minimum(
                counts + 1, self.config.occupied_confirm_count
            ).astype(np.uint16)
            self.occupied_counts[target_rows, target_cols] = counts
            if decay > 0:
                free_counts = self.free_counts[target_rows, target_cols].astype(
                    np.int32
                )
                self.free_counts[target_rows, target_cols] = np.maximum(
                    free_counts - decay, 0
                ).astype(np.uint16)
            current = self.data[target_rows, target_cols]
            commit = (current == OCCUPIED) | (
                counts >= self.config.occupied_confirm_count
            )
            if np.any(commit):
                self.data[target_rows[commit], target_cols[commit]] = OCCUPIED
                self.free_counts[target_rows[commit], target_cols[commit]] = 0

        if np.any(free_mask):
            target_rows = local_rows[free_mask]
            target_cols = local_cols[free_mask]
            counts = self.free_counts[target_rows, target_cols].astype(np.uint32)
            counts = np.minimum(
                counts + 1, self.config.free_clear_count
            ).astype(np.uint16)
            self.free_counts[target_rows, target_cols] = counts
            if decay > 0:
                occupied_counts = self.occupied_counts[
                    target_rows, target_cols
                ].astype(np.int32)
                self.occupied_counts[target_rows, target_cols] = np.maximum(
                    occupied_counts - decay, 0
                ).astype(np.uint16)
            current = self.data[target_rows, target_cols]
            clear = (current != OCCUPIED) | (
                counts >= self.config.free_clear_count
            )
            if np.any(clear):
                self.data[target_rows[clear], target_cols[clear]] = FREE
                self.occupied_counts[target_rows[clear], target_cols[clear]] = 0

        self.last_observed[local_rows, local_cols] = np.uint32(self.frame_index)
        updated_values = self.data[local_rows, local_cols]
        cleared = int(
            np.count_nonzero(
                (previous_values == OCCUPIED) & (updated_values == FREE)
            )
        )
        committed = int(
            np.count_nonzero(
                (previous_values != OCCUPIED) & (updated_values == OCCUPIED)
            )
        )
        return FusionUpdateStats(
            observed_cells=int(cols_array.size),
            free_observed_cells=int(np.count_nonzero(free_mask)),
            occupied_observed_cells=int(np.count_nonzero(occupied_mask)),
            cleared_cells=cleared,
            committed_occupied_cells=committed,
        )

    def dense_snapshot(self) -> Tuple[int, int, np.ndarray]:
        return self.origin_col, self.origin_row, self.data.copy()
