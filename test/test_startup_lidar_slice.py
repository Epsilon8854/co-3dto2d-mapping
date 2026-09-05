from pathlib import Path

import numpy as np

from co_3dto2d_mapping.startup_lidar_slice import LidarSliceConfig, filter_lidar_slice


def test_lidar_zero_slice_is_symmetric_and_uses_original_filters():
    points = np.asarray([
        [2.0, 0.0, -0.40], [2.0, 1.0, 0.40], [2.0, 2.0, 0.401],
        [0.2, 0.2, 0.0], [13.0, 0.0, 0.0], [-2.0, 0.0, 0.0],
    ])
    config = LidarSliceConfig(
        slice_half_height_m=0.40,
        center_box_half_extent_m=0.80,
        range_min_m=0.80,
        range_max_m=12.0,
        rear_filter_enabled=True,
        rear_filter_axis="-x",
        rear_filter_angle_deg=120.0,
    )
    kept, stats = filter_lidar_slice(points, config)
    flipped = points.copy()
    flipped[:, 2] *= -1.0
    inverted, _ = filter_lidar_slice(flipped, config)
    np.testing.assert_allclose(kept, inverted)
    np.testing.assert_allclose(kept, np.asarray([[2.0, 0.0], [2.0, 1.0]]))
    assert (stats.rejected_z, stats.rejected_range, stats.rejected_center, stats.rejected_rear) == (1, 2, 0, 1)


def test_public_startup_path_is_map_based_2d_icp():
    root = Path(__file__).resolve().parents[1]
    files = [
        root / "co_3dto2d_mapping" / "two_live_lidar_slice_launch.py",
        root / "co_3dto2d_mapping" / "startup_lidar_launch_map.py",
        root / "co_3dto2d_mapping" / "startup_lidar_launch_icp.py",
    ]
    wiring = "\n".join(path.read_text() for path in files)
    assert '"input_mode": "global_occupancy"' in wiring
    assert 'executable="startup_lidar_occupancy.py"' in wiring
    assert '"transform_cloud_to_local_frame": False' in wiring
    assert 'default_value="0.40"' in wiring
