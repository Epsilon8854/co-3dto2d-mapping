from pathlib import Path
import math
import struct

import numpy as np
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField

from co_3dto2d_mapping.rerun_mapping_node import (
    apply_planar_transform,
    apply_pose_transform,
    occupied_cell_centers,
    pointcloud_xyz,
    quaternion_rotation_matrix,
)


PACKAGE = Path(__file__).resolve().parents[1]


def test_occupied_cell_centers_excludes_unknown_and_free():
    msg = OccupancyGrid()
    msg.info.width = 3
    msg.info.height = 1
    msg.info.resolution = 1.0
    msg.info.origin.orientation.w = 1.0
    msg.data = [-1, 0, 100]

    points = occupied_cell_centers(msg)

    np.testing.assert_allclose(points, [[2.5, 0.5, 0.0]])


def test_pointcloud_xyz_keeps_only_finite_points():
    msg = PointCloud2()
    msg.height = 1
    msg.width = 2
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 12
    msg.row_step = 24
    msg.data = struct.pack("<ffffff", 1.0, 2.0, 3.0, math.nan, 5.0, 6.0)

    points = pointcloud_xyz(msg)

    np.testing.assert_allclose(points, [[1.0, 2.0, 3.0]])


def test_apply_planar_transform_rotates_then_translates():
    points = np.asarray([[1.0, 0.0, 0.0]], dtype=np.float32)

    transformed = apply_planar_transform(points, 1.0, 2.0, math.pi / 2.0)

    np.testing.assert_allclose(transformed, [[1.0, 3.0, 0.0]], atol=1e-6)


def test_apply_pose_transform_uses_z_roll_and_pitch_not_only_yaw():
    quaternion = Quaternion()
    quaternion.x = math.sin(math.pi / 4.0)
    quaternion.w = math.cos(math.pi / 4.0)
    points = np.asarray([[0.0, 1.0, 0.0]], dtype=np.float32)

    transformed = apply_pose_transform(points, [1.0, 2.0, 3.0], quaternion)

    np.testing.assert_allclose(transformed, [[1.0, 2.0, 4.0]], atol=1e-6)


def test_quaternion_rotation_matrix_normalizes_input():
    quaternion = Quaternion(x=0.0, y=0.0, z=2.0, w=2.0)
    rotation = quaternion_rotation_matrix(quaternion)

    np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
    assert math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-6)


def test_rviz_presets_show_plane_cloud_and_final_ground_fused_pose():
    single = (PACKAGE / "rviz" / "single_robot_mapping.rviz").read_text()
    two = (PACKAGE / "rviz" / "two_robot_mapping.rviz").read_text()

    assert "/r0/mapping/plane_height_filtered" in single
    assert "/r0/toy/corrected_odometry" in single
    assert "Plane-Relative Obstacle Cloud (0.05-1.00m)" in single
    assert "Reliability Policy: Best Effort" in single
    assert "Pitch: 1.05" in single

    for robot_id in (0, 1):
        assert f"/r{robot_id}/mapping/plane_height_filtered" in two
        assert f"/toy_record/r{robot_id}/odom" in two
        assert f"Reference Frame: r{robot_id}/base_link" in two
    assert two.count("Reliability Policy: Best Effort") >= 4
    assert "Fixed Frame: map" in two
    assert "Pitch: 1.08" in two


def test_rerun_defaults_use_plane_cloud_and_full_pose_axes():
    config = (PACKAGE / "config" / "rerun.yaml").read_text()
    node = (
        PACKAGE / "co_3dto2d_mapping" / "rerun_mapping_node.py"
    ).read_text()

    assert "visualize_plane_height_cloud: true" in config
    assert (
        'plane_height_cloud_topic_format: "/r{robot_id}/mapping/plane_height_filtered"'
        in config
    )
    assert "visualize_legacy_slice_points: false" in config
    assert "apply_pose_transform(local_points, position, orientation)" in node
    assert "quaternion_rotation_matrix" in node
    assert 'f"{root}/axes"' in node
    assert "qos_profile_sensor_data" in node
