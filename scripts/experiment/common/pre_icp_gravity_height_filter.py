#!/usr/bin/env python3
"""Filter a Livox cloud by IMU-gravity height before RTAB-Map ICP.

This experiment-only node has no odometry input, so its output can safely be
used as ``icp_odometry`` input. It preserves every PointCloud2 field (including
Livox's per-point ``timestamp`` field) for the points it keeps.
"""

from __future__ import annotations

from pathlib import Path as _WorkspacePath
import sys as _workspace_sys
_workspace_root = next(p for p in _WorkspacePath(__file__).resolve().parents if (p/"package.xml").is_file() and (p/"CMakeLists.txt").is_file())
_workspace_sys.path.insert(0, str(_workspace_root))

from copy import deepcopy
import csv
import math
from pathlib import Path
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField

from scripts.experiment.support.legacy_floor.gravity_plane_utils import (
    imu_timestamp_is_usable,
    up_from_world_orientation,
)
from scripts.experiment.support.legacy_floor.plane_height_filter import filter_points_by_fixed_sensor_height


class PreIcpGravityHeightFilter(Node):
    def __init__(self) -> None:
        super().__init__("pre_icp_gravity_height_filter")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("input_topic", "/r0/mapping/lidar_rear_filtered"),
                ("imu_topic", "/r0/mapping/imu_filtered"),
                ("output_topic", "/r0/mapping/lidar_icp_height_filtered"),
                ("sensor_height_m", 0.30),
                ("min_height_m", 0.30),
                ("max_height_m", 1.20),
                ("imu_timeout_sec", 0.30),
                ("min_output_points", 20),
                ("stats_csv", ""),
            ],
        )
        value = lambda name: self.get_parameter(name).value
        self.sensor_height_m = float(value("sensor_height_m"))
        self.min_height_m = float(value("min_height_m"))
        self.max_height_m = float(value("max_height_m"))
        self.imu_timeout_sec = float(value("imu_timeout_sec"))
        self.min_output_points = int(value("min_output_points"))
        if self.sensor_height_m <= 0.0:
            raise ValueError("sensor_height_m must be positive")
        if self.min_height_m < 0.0 or self.max_height_m <= self.min_height_m:
            raise ValueError("height range is invalid")
        if self.min_output_points < 1:
            raise ValueError("min_output_points must be positive")

        self._latest_imu: Imu | None = None
        self._imu_lock = threading.Lock()
        self._warned_frame_pair: set[tuple[str, str]] = set()
        self._published_count = 0
        self._dropped_count = 0
        self._stats_file = None
        self._stats_writer = None
        stats_csv = str(value("stats_csv"))
        if stats_csv:
            path = Path(stats_csv)
            path.parent.mkdir(parents=True, exist_ok=True)
            self._stats_file = path.open("w", newline="", encoding="utf-8")
            self._stats_writer = csv.DictWriter(
                self._stats_file,
                fieldnames=(
                    "stamp_ns",
                    "input_points",
                    "output_points",
                    "retention_ratio",
                    "up_x",
                    "up_y",
                    "up_z",
                    "state",
                ),
            )
            self._stats_writer.writeheader()

        self._publisher = self.create_publisher(
            PointCloud2, str(value("output_topic")), qos_profile_sensor_data
        )
        self._imu_sub = self.create_subscription(
            Imu, str(value("imu_topic")), self._imu_callback, qos_profile_sensor_data
        )
        self._cloud_sub = self.create_subscription(
            PointCloud2,
            str(value("input_topic")),
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "Pre-ICP gravity-height filter %s -> %s, floor-relative height "
            "[%.3f, %.3f] m" % (
                value("input_topic"), value("output_topic"),
                self.min_height_m, self.max_height_m,
            )
        )

    @staticmethod
    def _stamp_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @staticmethod
    def _field_dtype(field: PointField, is_bigendian: bool) -> str:
        byte_order = ">" if is_bigendian else "<"
        if field.datatype == PointField.FLOAT32:
            return byte_order + "f4"
        if field.datatype == PointField.FLOAT64:
            return byte_order + "f8"
        raise ValueError("PointCloud2 xyz fields must be FLOAT32 or FLOAT64")

    @classmethod
    def _points_xyz(cls, cloud: PointCloud2) -> np.ndarray:
        fields = {field.name: field for field in cloud.fields}
        if not {"x", "y", "z"}.issubset(fields):
            raise ValueError("PointCloud2 must include x, y, z fields")
        if cloud.height != 1 or cloud.row_step != cloud.width * cloud.point_step:
            raise ValueError("only compact, unorganized PointCloud2 messages are supported")
        dtype = np.dtype(
            {
                "names": ["x", "y", "z"],
                "formats": [
                    cls._field_dtype(fields["x"], cloud.is_bigendian),
                    cls._field_dtype(fields["y"], cloud.is_bigendian),
                    cls._field_dtype(fields["z"], cloud.is_bigendian),
                ],
                "offsets": [fields["x"].offset, fields["y"].offset, fields["z"].offset],
                "itemsize": int(cloud.point_step),
            }
        )
        records = np.frombuffer(cloud.data, dtype=dtype, count=int(cloud.width))
        return np.column_stack((records["x"], records["y"], records["z"])).astype(
            np.float64, copy=False
        )

    @staticmethod
    def _select_records(cloud: PointCloud2, mask: np.ndarray) -> PointCloud2:
        records = np.frombuffer(cloud.data, dtype=np.uint8).reshape(
            int(cloud.width), int(cloud.point_step)
        )
        selected = np.ascontiguousarray(records[mask])
        output = deepcopy(cloud)
        output.height = 1
        output.width = int(len(selected))
        output.row_step = int(output.width * output.point_step)
        output.data = selected.tobytes()
        output.is_dense = True
        return output

    def _imu_callback(self, message: Imu) -> None:
        with self._imu_lock:
            self._latest_imu = message

    def _up_for_cloud(self, cloud: PointCloud2) -> np.ndarray | None:
        with self._imu_lock:
            imu = self._latest_imu
        if imu is None or imu.orientation_covariance[0] < 0.0:
            return None
        if not imu_timestamp_is_usable(
            self._stamp_ns(cloud.header.stamp),
            self._stamp_ns(imu.header.stamp),
            self.imu_timeout_sec,
        ):
            return None
        if imu.header.frame_id and cloud.header.frame_id and (
            imu.header.frame_id != cloud.header.frame_id
        ):
            pair = (imu.header.frame_id, cloud.header.frame_id)
            if pair not in self._warned_frame_pair:
                self._warned_frame_pair.add(pair)
                self.get_logger().error(
                    "IMU and cloud frame differ (%s != %s); refusing an unverified "
                    "gravity transform" % pair
                )
            return None
        try:
            return up_from_world_orientation(
                (
                    imu.orientation.x,
                    imu.orientation.y,
                    imu.orientation.z,
                    imu.orientation.w,
                )
            )
        except ValueError:
            return None

    def _write_stats(
        self, cloud: PointCloud2, input_points: int, output_points: int,
        up: np.ndarray | None, state: str,
    ) -> None:
        if self._stats_writer is None:
            return
        self._stats_writer.writerow(
            {
                "stamp_ns": self._stamp_ns(cloud.header.stamp),
                "input_points": input_points,
                "output_points": output_points,
                "retention_ratio": output_points / input_points if input_points else 0.0,
                "up_x": "" if up is None else float(up[0]),
                "up_y": "" if up is None else float(up[1]),
                "up_z": "" if up is None else float(up[2]),
                "state": state,
            }
        )
        self._stats_file.flush()

    def _cloud_callback(self, cloud: PointCloud2) -> None:
        input_points = int(cloud.width) * int(cloud.height)
        up = self._up_for_cloud(cloud)
        if up is None:
            self._dropped_count += 1
            self._write_stats(cloud, input_points, 0, None, "gravity_unavailable")
            return
        try:
            points = self._points_xyz(cloud)
            result = filter_points_by_fixed_sensor_height(
                points, up, self.sensor_height_m, self.min_height_m, self.max_height_m
            )
        except ValueError as error:
            self._dropped_count += 1
            self.get_logger().error(str(error))
            self._write_stats(cloud, input_points, 0, up, "invalid_cloud")
            return
        output_points = int(np.count_nonzero(result.mask))
        if output_points < self.min_output_points:
            self._dropped_count += 1
            self._write_stats(cloud, input_points, output_points, up, "too_few_points")
            return
        self._publisher.publish(self._select_records(cloud, result.mask))
        self._published_count += 1
        self._write_stats(cloud, input_points, output_points, up, "published")

    def destroy_node(self):
        self.get_logger().info(
            "Pre-ICP height filter complete: published=%d dropped=%d"
            % (self._published_count, self._dropped_count)
        )
        if self._stats_file is not None:
            self._stats_file.close()
            self._stats_file = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PreIcpGravityHeightFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
