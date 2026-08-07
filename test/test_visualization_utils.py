import math
import struct

import numpy as np
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField

from co_3dto2d_mapping.rerun_mapping_node import (
    apply_planar_transform,
    occupied_cell_centers,
    pointcloud_xyz,
)


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
