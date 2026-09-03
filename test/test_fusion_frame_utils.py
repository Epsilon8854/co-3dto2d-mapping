import numpy as np

from co_3dto2d_mapping.fusion_frame_utils import (
    reference_transform_for_robot,
    seed_unknown_observations,
)


class FakeFusion:
    def __init__(self):
        self.origin_col = 0
        self.origin_row = 0
        self.data = np.full((3, 3), -1, dtype=np.int8)

    @staticmethod
    def _normalize_observations(cols, rows, values):
        return (
            np.asarray(cols, dtype=np.int64),
            np.asarray(rows, dtype=np.int64),
            np.asarray(values, dtype=np.int8),
        )

    def _ensure_bounds(self, cols, rows):
        assert np.all((cols >= 0) & (cols < 3))
        assert np.all((rows >= 0) & (rows < 3))


def test_repeated_global_seed_only_fills_unknown_cells():
    fusion = FakeFusion()
    assert seed_unknown_observations(fusion, [1], [1], [100]) == 1
    # A live free observation cleared the cell.
    fusion.data[1, 1] = 0
    # A stale persistent global map must not reinsert occupied.
    assert seed_unknown_observations(fusion, [1], [1], [100]) == 0
    assert fusion.data[1, 1] == 0
    # Newly explored space is still added.
    assert seed_unknown_observations(fusion, [2], [1], [100]) == 1
    assert fusion.data[1, 2] == 100


def test_reference_transform_supports_r1_only_then_canonical_alignment():
    available, transform = reference_transform_for_robot(1, None, 1)
    assert available and transform is None
    assert reference_transform_for_robot(1, None, 0) == (False, None)
    alignment = (1.0, 2.0, 0.1)
    assert reference_transform_for_robot(0, alignment, 0) == (True, None)
    assert reference_transform_for_robot(0, alignment, 1) == (True, alignment)
