"""Launch helpers for raw-LiDAR startup occupancy map builders."""

from __future__ import annotations

import math

from launch.substitutions import LaunchConfiguration


STARTUP_MAP_TOPICS = (
    "/r0/startup/lidar_slice_occupancy",
    "/r1/startup/lidar_slice_occupancy",
)


def float_argument(context, name: str) -> float:
    try:
        value = float(LaunchConfiguration(name).perform(context))
    except ValueError as exc:
        raise RuntimeError("%s must be numeric" % name) from exc
    if not math.isfinite(value):
        raise RuntimeError("%s must be finite" % name)
    return value


def positive_float_argument(context, name: str) -> float:
    value = float_argument(context, name)
    if value <= 0.0:
        raise RuntimeError("%s must be greater than zero" % name)
    return value


def positive_int_argument(context, name: str) -> int:
    try:
        value = int(LaunchConfiguration(name).perform(context))
    except ValueError as exc:
        raise RuntimeError("%s must be an integer" % name) from exc
    if value < 1:
        raise RuntimeError("%s must be at least one" % name)
    return value


def startup_map_node(layer, source_cloud_topic, context, robot_id: int, occupancy):
    """Create one odometry-free LiDAR-frame startup OccupancyGrid builder."""

    range_max = layer._occupancy_float(occupancy, "range_max_m", 12.0)
    requested_extent = float_argument(context, "startup_map_half_extent_m")
    map_half_extent = (
        requested_extent
        if requested_extent > 0.0
        else (range_max if range_max > 0.0 else 12.0)
    )
    return layer._ORIGINAL_NODE(
        package="co_3dto2d_mapping",
        executable="startup_lidar_occupancy.py",
        name="startup_lidar_occupancy_r%d" % robot_id,
        output="screen",
        parameters=[
            {
                "input_cloud_topic": source_cloud_topic(context, robot_id),
                "output_map_topic": STARTUP_MAP_TOPICS[robot_id],
                "output_frame_id": "r%d/livox_frame" % robot_id,
                "frame_count": int(
                    layer._ORIGINAL_VALUE(context, "alignment_frame_count")
                ),
                "startup_delay_sec": float(
                    layer._ORIGINAL_VALUE(context, "alignment_startup_delay_sec")
                ),
                "slice_center_z_m": float_argument(
                    context, "startup_map_slice_center_z_m"
                ),
                "slice_half_height_m": positive_float_argument(
                    context, "startup_map_slice_half_height_m"
                ),
                "center_box_half_extent_m": layer._occupancy_float(
                    occupancy, "center_box_filter_half_extent_m", 0.80
                ),
                "range_min_m": layer._occupancy_float(
                    occupancy, "range_min_m", 0.80
                ),
                "range_max_m": range_max,
                "rear_filter_enabled": layer._parse_bool(
                    layer._ORIGINAL_VALUE(context, "enable_rear_lidar_filter"),
                    "enable_rear_lidar_filter",
                ),
                "rear_filter_angle_deg": float(
                    layer._ORIGINAL_VALUE(context, "rear_filter_angle_deg")
                ),
                "rear_filter_axis": layer._ORIGINAL_VALUE(
                    context, "rear_filter_axis"
                ),
                "rear_filter_min_xy_range_m": float(
                    layer._ORIGINAL_VALUE(
                        context, "rear_filter_min_xy_range_m"
                    )
                ),
                "grid_resolution_m": layer._occupancy_float(
                    occupancy, "grid_resolution", 0.05
                ),
                "map_half_extent_m": map_half_extent,
                "occupied_threshold_points": int(
                    occupancy.get("occupied_threshold_points", 1)
                ),
                "min_occupied_cells": positive_int_argument(
                    context, "startup_map_min_occupied_cells"
                ),
                "max_points_per_frame": int(
                    layer._ORIGINAL_VALUE(context, "alignment_max_points")
                ),
                "lock_after_first_map": True,
                "publish_period_sec": positive_float_argument(
                    context, "startup_map_publish_period_sec"
                ),
                "use_sim_time": False,
            }
        ],
    )
