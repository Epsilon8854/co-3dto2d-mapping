"""Two-robot live mapping with a config-driven initial XYZ alignment.

The launch can run either robot pipeline independently and can also run the
cross-robot alignment/common-frame output on one host. Alignment algorithm
parameters come from ``occupancy_config_file``. Legacy ``alignment_*`` launch
arguments are optional compatibility overrides: their empty defaults do not
replace values from the YAML file.
"""

from __future__ import annotations

import os
from typing import Callable, Dict

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_TRUE_VALUES = {"1", "true", "yes", "on"}
_FALSE_VALUES = {"0", "false", "no", "off"}


def _value(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def _parse_bool(value: str, name: str) -> bool:
    normalized = value.strip().lower()
    if normalized in _TRUE_VALUES:
        return True
    if normalized in _FALSE_VALUES:
        return False
    raise RuntimeError("%s must be boolean, got %r" % (name, value))


def _bool_value(context, name: str) -> bool:
    return _parse_bool(_value(context, name), name)


def _require_absolute_topic(name: str, topic: str) -> None:
    if not topic.startswith("/"):
        raise RuntimeError(
            "%s must be an absolute ROS topic (start with '/'), got %r"
            % (name, topic)
        )


def _optional_override(
    context,
    output: Dict[str, object],
    launch_name: str,
    parameter_name: str,
    converter: Callable[[str], object],
) -> None:
    raw = _value(context, launch_name).strip()
    if raw:
        output[parameter_name] = converter(raw)


def _robot_actions(
    context,
    robot_id: int,
    live_launch_path: str,
    enable_rear_lidar_filter: bool,
):
    input_prefix = "robot%d" % robot_id
    robot_namespace = "/r%d" % robot_id
    lidar_input_topic = _value(context, input_prefix + "_lidar_topic")
    imu_input_topic = _value(context, input_prefix + "_imu_topic")

    sensor_parent_frame = "r%d/base_link" % robot_id
    sensor_child_frame = "r%d/livox_frame" % robot_id
    global_frame_id = "r%d/odom" % robot_id
    scan_cloud_topic = robot_namespace + "/mapping/lidar"
    lidar_relay_topic = (
        robot_namespace + "/mapping/lidar_unfiltered"
        if enable_rear_lidar_filter
        else scan_cloud_topic
    )
    imu_filtered_topic = robot_namespace + "/mapping/imu_filtered"
    imu_filter_raw_frame_topic = imu_filtered_topic + "_raw_frame"

    if lidar_input_topic in {lidar_relay_topic, scan_cloud_topic}:
        raise RuntimeError(
            "%s_lidar_topic=%r collides with an internal mapping topic and would "
            "create a republish loop" % (input_prefix, lidar_input_topic)
        )
    if imu_input_topic in {imu_filtered_topic, imu_filter_raw_frame_topic}:
        raise RuntimeError(
            "%s_imu_topic=%r collides with an internal mapping topic and would "
            "create an IMU filter loop" % (input_prefix, imu_input_topic)
        )

    lidar_frame_republisher = Node(
        package="co_3dto2d_mapping",
        executable="pointcloud_frame_republisher.py",
        name="pointcloud_frame_republisher_r%d" % robot_id,
        output="screen",
        parameters=[
            {
                "input_topic": lidar_input_topic,
                "output_topic": lidar_relay_topic,
                "output_frame_id": sensor_child_frame,
            }
        ],
    )

    live_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(live_launch_path),
        launch_arguments={
            "robot_id": str(robot_id),
            "use_sim_time": "false",
            "publish_tf_odom": "false",
            "publish_sensor_static_tf": _value(context, "publish_sensor_static_tf"),
            "sensor_parent_frame": sensor_parent_frame,
            "sensor_child_frame": sensor_child_frame,
            "local_frame_id": sensor_parent_frame,
            "global_frame_id": global_frame_id,
            "use_odom_header_frame": "false",
            "sensor_tf_x": _value(context, "sensor_tf_x_%d" % robot_id),
            "sensor_tf_y": _value(context, "sensor_tf_y_%d" % robot_id),
            "sensor_tf_z": _value(context, "sensor_tf_z_%d" % robot_id),
            "sensor_tf_yaw": _value(context, "sensor_tf_yaw_%d" % robot_id),
            "sensor_tf_pitch": _value(context, "sensor_tf_pitch_%d" % robot_id),
            "sensor_tf_roll": _value(context, "sensor_tf_roll_%d" % robot_id),
            "expected_update_rate": _value(context, "expected_update_rate"),
            "wait_imu_to_init": _value(context, "wait_imu_to_init"),
            "mapping_startup_delay_sec": _value(
                context, "mapping_startup_delay_sec"
            ),
            "enable_rear_lidar_filter": _value(
                context, "enable_rear_lidar_filter"
            ),
            "rear_filter_angle_deg": _value(context, "rear_filter_angle_deg"),
            "rear_filter_axis": _value(context, "rear_filter_axis"),
            "rear_filter_min_xy_range_m": _value(
                context, "rear_filter_min_xy_range_m"
            ),
            "rear_filter_log_period": _value(context, "rear_filter_log_period"),
            "bag_lidar_topic": lidar_relay_topic,
            "scan_cloud_topic": scan_cloud_topic,
            "imu_raw_topic": imu_input_topic,
            "imu_filtered_topic": imu_filtered_topic,
            "alignment_topic": _value(context, "alignment_topic"),
            "transform_cloud_to_local_frame": _value(
                context, "transform_cloud_to_local_frame"
            ),
            "center_box_filter_half_extent_m": _value(
                context, "center_box_filter_half_extent_m"
            ),
            "slice_z_in_cloud_frame": _value(context, "slice_z_in_cloud_frame"),
            "occupancy_config_file": _value(context, "occupancy_config_file"),
        }.items(),
    )
    return lidar_frame_republisher, live_pipeline, scan_cloud_topic


def _alignment_parameters(context, robot0_scan_topic: str, robot1_scan_topic: str):
    parameters: Dict[str, object] = {
        "robot0_cloud_topic": robot0_scan_topic,
        "robot1_cloud_topic": robot1_scan_topic,
        "robot0_map_topic": "/r0/toy/global_occupancy",
        "robot1_map_topic": "/r1/toy/global_occupancy",
        "input_mode": "cloud_initial",
        "alignment_topic": _value(context, "alignment_topic"),
        "target_frame_id": _value(context, "common_frame_id"),
        "source_frame_id": "r1/odom",
        "local_frame_id": "base_link",
        "robot0_local_frame_id": "r0/base_link",
        "robot1_local_frame_id": "r1/base_link",
        "transform_cloud_to_local_frame": _bool_value(
            context, "transform_cloud_to_local_frame"
        ),
        "startup_delay_sec": float(_value(context, "alignment_startup_delay_sec")),
        "retry_on_failure": True,
        "use_sim_time": False,
    }

    optional = (
        ("alignment_use_z_filter", "use_z_filter", lambda value: _parse_bool(value, "alignment_use_z_filter")),
        ("alignment_slice_z_in_cloud_frame", "slice_z_in_cloud_frame", lambda value: _parse_bool(value, "alignment_slice_z_in_cloud_frame")),
        ("alignment_z_min", "z_min", float),
        ("alignment_z_max", "z_max", float),
        ("alignment_invert_z_slice", "invert_z_slice", lambda value: _parse_bool(value, "alignment_invert_z_slice")),
        ("alignment_frame_count", "frame_count", int),
        ("alignment_invert_result", "invert_result", lambda value: _parse_bool(value, "alignment_invert_result")),
        ("alignment_center_box_half_extent_m", "center_box_half_extent_m", float),
        ("alignment_range_min_m", "range_min_m", float),
        ("alignment_range_max_m", "range_max_m", float),
        ("alignment_voxel_size", "voxel_size", float),
        ("alignment_max_points", "max_points", int),
        ("alignment_max_correspondence_distance", "max_correspondence_distance", float),
        ("alignment_min_correspondences", "min_correspondences", int),
        ("alignment_min_fitness", "min_fitness", float),
        ("alignment_max_rmse", "max_rmse", float),
        ("alignment_max_iterations", "max_iterations", int),
        ("alignment_recompute_period_sec", "recompute_period_sec", float),
        ("alignment_occupied_threshold", "occupied_threshold", int),
        ("alignment_lock_after_first", "lock_after_first_alignment", lambda value: _parse_bool(value, "alignment_lock_after_first")),
        ("alignment_required_consistent_results", "required_consistent_results", int),
        ("alignment_max_consistency_translation_m", "max_consistency_translation_m", float),
        ("alignment_max_consistency_rotation_rad", "max_consistency_rotation_rad", float),
        ("alignment_initialize_from_centroids", "initialize_from_centroids", lambda value: _parse_bool(value, "alignment_initialize_from_centroids")),
        ("alignment_enforce_tilt_prior", "enforce_tilt_prior", lambda value: _parse_bool(value, "alignment_enforce_tilt_prior")),
        ("alignment_max_tilt_deviation_rad", "max_tilt_deviation_rad", float),
    )
    for launch_name, parameter_name, converter in optional:
        _optional_override(
            context, parameters, launch_name, parameter_name, converter
        )
    return parameters


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    package_share = get_package_share_directory("co_3dto2d_mapping")
    live_launch_path = os.path.join(package_share, "launch", "live_mapping.launch.py")
    occupancy_config_file = _value(context, "occupancy_config_file")
    if not occupancy_config_file or not os.path.isfile(occupancy_config_file):
        raise RuntimeError(
            "occupancy_config_file must point to a readable YAML file: %r"
            % occupancy_config_file
        )

    source_topics = {
        "robot0_lidar_topic": _value(context, "robot0_lidar_topic"),
        "robot1_lidar_topic": _value(context, "robot1_lidar_topic"),
        "robot0_imu_topic": _value(context, "robot0_imu_topic"),
        "robot1_imu_topic": _value(context, "robot1_imu_topic"),
    }
    for name, topic in source_topics.items():
        _require_absolute_topic(name, topic)
    topic_owners = {}
    for name, topic in source_topics.items():
        topic_owners.setdefault(topic, []).append(name)
    duplicates = {
        topic: names for topic, names in topic_owners.items() if len(names) > 1
    }
    if duplicates:
        details = ", ".join(
            "%s used by %s" % (topic, "/".join(names))
            for topic, names in sorted(duplicates.items())
        )
        raise RuntimeError("all live sensor input topics must be unique: " + details)

    reserved_internal_topics = {
        "/r0/mapping/lidar",
        "/r0/mapping/lidar_unfiltered",
        "/r0/mapping/imu_filtered",
        "/r0/mapping/imu_filtered_raw_frame",
        "/r1/mapping/lidar",
        "/r1/mapping/lidar_unfiltered",
        "/r1/mapping/imu_filtered",
        "/r1/mapping/imu_filtered_raw_frame",
    }
    collisions = {
        name: topic
        for name, topic in source_topics.items()
        if topic in reserved_internal_topics
    }
    if collisions:
        raise RuntimeError(
            "live sensor inputs cannot use internal mapping topics: "
            + ", ".join("%s=%s" % item for item in sorted(collisions.items()))
        )

    enable_rear_lidar_filter = _bool_value(context, "enable_rear_lidar_filter")
    enable_record_republisher = _bool_value(context, "enable_record_republisher")
    enable_robot0_pipeline = _bool_value(context, "enable_robot0_pipeline")
    enable_robot1_pipeline = _bool_value(context, "enable_robot1_pipeline")
    enable_fusion = _bool_value(context, "enable_fusion")
    if not (enable_robot0_pipeline or enable_robot1_pipeline or enable_fusion):
        raise RuntimeError(
            "at least one robot pipeline or the fusion pipeline must be enabled"
        )

    alignment_startup_delay_sec = float(
        _value(context, "alignment_startup_delay_sec")
    )
    if alignment_startup_delay_sec < 0.0:
        raise RuntimeError("alignment_startup_delay_sec must be non-negative")

    robot0_scan_topic = "/r0/mapping/lidar"
    robot1_scan_topic = "/r1/mapping/lidar"
    actions = []
    if enable_robot0_pipeline:
        relay, pipeline, robot0_scan_topic = _robot_actions(
            context, 0, live_launch_path, enable_rear_lidar_filter
        )
        actions.extend([pipeline, relay])
    if enable_robot1_pipeline:
        relay, pipeline, robot1_scan_topic = _robot_actions(
            context, 1, live_launch_path, enable_rear_lidar_filter
        )
        actions.extend([pipeline, relay])

    if enable_fusion:
        actions.insert(
            0,
            Node(
                package="co_3dto2d_mapping",
                executable="initial_xy_icp_alignment.py",
                name="initial_xy_icp_alignment",
                output="screen",
                remappings=[("tf", "/tf"), ("tf_static", "/tf_static")],
                parameters=[
                    occupancy_config_file,
                    _alignment_parameters(
                        context, robot0_scan_topic, robot1_scan_topic
                    ),
                ],
            ),
        )

    if enable_fusion and enable_record_republisher:
        actions.insert(
            1,
            Node(
                package="co_3dto2d_mapping",
                executable="record_republisher.py",
                name="toy_record_republisher",
                output="screen",
                parameters=[
                    {
                        "target_frame_id": "odom",
                        "common_frame_id": _value(context, "common_frame_id"),
                        "alignment_topic": _value(context, "alignment_topic"),
                        "publish_period_ms": int(
                            _value(context, "record_publish_period_ms")
                        ),
                        "output_prefix": _value(context, "record_output_prefix"),
                        "robot_ids": [0, 1],
                        "publish_tf": True,
                        "publish_merged_global": _bool_value(
                            context, "record_publish_merged_global"
                        ),
                        "use_sim_time": False,
                    }
                ],
            ),
        )
    return actions


def _legacy_alignment_arguments():
    description = "Optional compatibility override; empty means use occupancy YAML."
    return [
        DeclareLaunchArgument("alignment_use_z_filter", default_value="", description=description),
        DeclareLaunchArgument("alignment_slice_z_in_cloud_frame", default_value="", description=description),
        DeclareLaunchArgument("alignment_z_min", default_value="", description=description),
        DeclareLaunchArgument("alignment_z_max", default_value="", description=description),
        DeclareLaunchArgument("alignment_invert_z_slice", default_value="", description=description),
        DeclareLaunchArgument("alignment_frame_count", default_value="", description=description),
        DeclareLaunchArgument("alignment_invert_result", default_value="", description=description),
        DeclareLaunchArgument("alignment_center_box_half_extent_m", default_value="", description=description),
        DeclareLaunchArgument("alignment_range_min_m", default_value="", description=description),
        DeclareLaunchArgument("alignment_range_max_m", default_value="", description=description),
        DeclareLaunchArgument("alignment_voxel_size", default_value="", description=description),
        DeclareLaunchArgument("alignment_max_points", default_value="", description=description),
        DeclareLaunchArgument("alignment_max_correspondence_distance", default_value="", description=description),
        DeclareLaunchArgument("alignment_min_correspondences", default_value="", description=description),
        DeclareLaunchArgument("alignment_min_fitness", default_value="", description=description),
        DeclareLaunchArgument("alignment_max_rmse", default_value="", description=description),
        DeclareLaunchArgument("alignment_max_iterations", default_value="", description=description),
        DeclareLaunchArgument("alignment_recompute_period_sec", default_value="", description=description),
        DeclareLaunchArgument("alignment_occupied_threshold", default_value="", description=description),
        DeclareLaunchArgument("alignment_lock_after_first", default_value="", description=description),
        DeclareLaunchArgument("alignment_required_consistent_results", default_value="", description=description),
        DeclareLaunchArgument("alignment_max_consistency_translation_m", default_value="", description=description),
        DeclareLaunchArgument("alignment_max_consistency_rotation_rad", default_value="", description=description),
        DeclareLaunchArgument("alignment_initialize_from_centroids", default_value="", description=description),
        DeclareLaunchArgument("alignment_enforce_tilt_prior", default_value="", description=description),
        DeclareLaunchArgument("alignment_max_tilt_deviation_rad", default_value="", description=description),
    ]


def generate_launch_description():
    package_share = get_package_share_directory("co_3dto2d_mapping")
    declarations = [
        DeclareLaunchArgument("enable_robot0_pipeline", default_value="true"),
        DeclareLaunchArgument("enable_robot1_pipeline", default_value="true"),
        DeclareLaunchArgument("enable_fusion", default_value="true"),
        DeclareLaunchArgument("robot0_lidar_topic", default_value="/r0/livox/lidar"),
        DeclareLaunchArgument("robot0_imu_topic", default_value="/r0/livox/imu"),
        DeclareLaunchArgument("robot1_lidar_topic", default_value="/r1/livox/lidar"),
        DeclareLaunchArgument("robot1_imu_topic", default_value="/r1/livox/imu"),
        DeclareLaunchArgument("expected_update_rate", default_value="10.0"),
        DeclareLaunchArgument("wait_imu_to_init", default_value="true"),
        DeclareLaunchArgument("mapping_startup_delay_sec", default_value="10.0"),
        DeclareLaunchArgument("publish_sensor_static_tf", default_value="true"),
        DeclareLaunchArgument("enable_rear_lidar_filter", default_value="false"),
        DeclareLaunchArgument("rear_filter_angle_deg", default_value="120.0"),
        DeclareLaunchArgument("rear_filter_axis", default_value="-x"),
        DeclareLaunchArgument("rear_filter_min_xy_range_m", default_value="0.0"),
        DeclareLaunchArgument("rear_filter_log_period", default_value="100"),
        DeclareLaunchArgument("transform_cloud_to_local_frame", default_value="true"),
        DeclareLaunchArgument("center_box_filter_half_extent_m", default_value="0.80"),
        DeclareLaunchArgument("slice_z_in_cloud_frame", default_value="true"),
        DeclareLaunchArgument("sensor_tf_x_0", default_value="0"),
        DeclareLaunchArgument("sensor_tf_y_0", default_value="0"),
        DeclareLaunchArgument("sensor_tf_z_0", default_value="0"),
        DeclareLaunchArgument("sensor_tf_yaw_0", default_value="0"),
        DeclareLaunchArgument("sensor_tf_pitch_0", default_value="0"),
        DeclareLaunchArgument("sensor_tf_roll_0", default_value="3.141592653589793"),
        DeclareLaunchArgument("sensor_tf_x_1", default_value="0"),
        DeclareLaunchArgument("sensor_tf_y_1", default_value="0"),
        DeclareLaunchArgument("sensor_tf_z_1", default_value="0"),
        DeclareLaunchArgument("sensor_tf_yaw_1", default_value="0"),
        DeclareLaunchArgument("sensor_tf_pitch_1", default_value="0"),
        DeclareLaunchArgument("sensor_tf_roll_1", default_value="3.141592653589793"),
        DeclareLaunchArgument("common_frame_id", default_value="map"),
        DeclareLaunchArgument("alignment_topic", default_value="/toy/initial_xy_alignment"),
        DeclareLaunchArgument("alignment_startup_delay_sec", default_value="3.0"),
        DeclareLaunchArgument("enable_record_republisher", default_value="true"),
        DeclareLaunchArgument("record_publish_period_ms", default_value="200"),
        DeclareLaunchArgument("record_output_prefix", default_value="/toy_record"),
        DeclareLaunchArgument("record_publish_merged_global", default_value="true"),
        DeclareLaunchArgument(
            "occupancy_config_file",
            default_value=os.path.join(package_share, "config", "occupancy.yaml"),
        ),
    ]
    return LaunchDescription(
        declarations + _legacy_alignment_arguments() + [OpaqueFunction(function=launch_setup)]
    )
