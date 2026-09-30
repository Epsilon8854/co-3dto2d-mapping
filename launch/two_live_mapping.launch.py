"""Two independent live mapping pipelines with occupancy-only inter-robot ICP.

LiDAR/IMU remain local inputs to each robot's odometry and mapper. The fusion
host subscribes to OccupancyGrid and corrected odometry for registration; no
cross-robot point cloud or accepted alignment is needed to start local mapping.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from co_3dto2d_mapping.dynamic_filter_config import merged_dynamic_parameters


_TRUE_VALUES = {"1", "true", "yes", "on"}
_FALSE_VALUES = {"0", "false", "no", "off"}


def _value(context, name):
    return LaunchConfiguration(name).perform(context)


def _bool_value(context, name):
    value = _value(context, name).strip().lower()
    if value in _TRUE_VALUES:
        return True
    if value in _FALSE_VALUES:
        return False
    raise RuntimeError("%s must be a boolean value, got %r" % (name, value))


def _require_absolute_topic(name, topic):
    if not topic.startswith("/"):
        raise RuntimeError(
            "%s must be an absolute ROS topic (start with '/'), got %r"
            % (name, topic)
        )


def _robot_actions(context, robot_id, live_launch_path, enable_rear_lidar_filter):
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
        if enable_rear_lidar_filter else scan_cloud_topic
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
        parameters=[{
            "input_topic": lidar_input_topic,
            "output_topic": lidar_relay_topic,
            "output_frame_id": sensor_child_frame,
        }],
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
            "mapping_startup_delay_sec": _value(context, "mapping_startup_delay_sec"),
            "enable_rear_lidar_filter": _value(context, "enable_rear_lidar_filter"),
            "rear_filter_angle_deg": _value(context, "rear_filter_angle_deg"),
            "rear_filter_axis": _value(context, "rear_filter_axis"),
            "rear_filter_min_xy_range_m": _value(context, "rear_filter_min_xy_range_m"),
            "rear_filter_log_period": _value(context, "rear_filter_log_period"),
            "bag_lidar_topic": lidar_relay_topic,
            "scan_cloud_topic": scan_cloud_topic,
            "imu_raw_topic": imu_input_topic,
            "imu_filtered_topic": imu_filtered_topic,
            "imu_input_is_filtered": _value(context, input_prefix + "_imu_input_is_filtered"),
            "alignment_topic": _value(context, "alignment_topic"),
            "transform_cloud_to_local_frame": _value(context, "transform_cloud_to_local_frame"),
            "center_box_filter_half_extent_m": _value(context, "center_box_filter_half_extent_m"),
            "slice_z_in_cloud_frame": _value(context, "slice_z_in_cloud_frame"),
            "occupancy_config_file": _value(context, "occupancy_config_file"),
        }.items(),
    )
    return lidar_frame_republisher, live_pipeline, scan_cloud_topic


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    package_share = get_package_share_directory("co_3dto2d_mapping")
    live_launch_path = os.path.join(package_share, "launch", "live_mapping.launch.py")
    source_topics = {
        name: _value(context, name)
        for name in (
            "robot0_lidar_topic", "robot1_lidar_topic",
            "robot0_imu_topic", "robot1_imu_topic",
        )
    }
    for name, topic in source_topics.items():
        _require_absolute_topic(name, topic)
    if source_topics["robot0_lidar_topic"] == source_topics["robot1_lidar_topic"]:
        raise RuntimeError(
            "robot0_lidar_topic and robot1_lidar_topic must be different; "
            "two live LiDAR streams cannot share one topic"
        )
    if source_topics["robot0_imu_topic"] == source_topics["robot1_imu_topic"]:
        raise RuntimeError(
            "robot0_imu_topic and robot1_imu_topic must be different; "
            "two live IMU streams cannot share one topic"
        )
    topic_owners = {}
    for name, topic in source_topics.items():
        topic_owners.setdefault(topic, []).append(name)
    cross_type_duplicates = {
        topic: names for topic, names in topic_owners.items() if len(names) > 1
    }
    if cross_type_duplicates:
        details = ", ".join(
            "%s used by %s" % (topic, "/".join(names))
            for topic, names in sorted(cross_type_duplicates.items())
        )
        raise RuntimeError("all live sensor input topics must be unique: " + details)
    reserved_internal_topics = {
        "/r%d/mapping/%s" % (rid, suffix)
        for rid in (0, 1)
        for suffix in ("lidar", "lidar_unfiltered", "imu_filtered", "imu_filtered_raw_frame")
    }
    internal_collisions = {
        name: topic for name, topic in source_topics.items()
        if topic in reserved_internal_topics
    }
    if internal_collisions:
        details = ", ".join("%s=%s" % item for item in sorted(internal_collisions.items()))
        raise RuntimeError("live sensor inputs cannot use internal mapping topics: " + details)

    enable_rear_lidar_filter = _bool_value(context, "enable_rear_lidar_filter")
    enable_record_republisher = _bool_value(context, "enable_record_republisher")
    enable_robot0_pipeline = _bool_value(context, "enable_robot0_pipeline")
    enable_robot1_pipeline = _bool_value(context, "enable_robot1_pipeline")
    enable_fusion = _bool_value(context, "enable_fusion")
    enable_place_recognition = _bool_value(context, "enable_place_recognition")
    if not (enable_robot0_pipeline or enable_robot1_pipeline or enable_fusion):
        raise RuntimeError("at least one robot pipeline or the fusion pipeline must be enabled")
    alignment_startup_delay_sec = float(_value(context, "alignment_startup_delay_sec"))
    if not math.isfinite(alignment_startup_delay_sec) or alignment_startup_delay_sec < 0.0:
        raise RuntimeError("alignment_startup_delay_sec must be finite and non-negative")

    actions = []
    for robot_id, enabled in ((0, enable_robot0_pipeline), (1, enable_robot1_pipeline)):
        if enabled:
            republisher, pipeline, _ = _robot_actions(
                context, robot_id, live_launch_path, enable_rear_lidar_filter
            )
            # No startup alignment gate: these actions depend on local inputs only.
            actions.extend([pipeline, republisher])

    alignment_topic = _value(context, "alignment_topic")
    common_frame_id = _value(context, "common_frame_id")
    if enable_fusion and enable_place_recognition:
        parameters = [os.path.join(package_share, "config", "place_recognition.yaml")]
        alignment_config_file = _value(context, "alignment_config_file").strip()
        if alignment_config_file:
            parameters.append(alignment_config_file)
        parameters.append({
            "robot0_map_topic": "/r0/toy/global_occupancy",
            "robot1_map_topic": "/r1/toy/global_occupancy",
            "robot0_odom_topic": "/r0/toy/corrected_odometry",
            "robot1_odom_topic": "/r1/toy/corrected_odometry",
            "alignment_topic": alignment_topic,
            "target_frame_id": common_frame_id,
            "source_frame_id": "r1/odom",
            "startup_delay_sec": alignment_startup_delay_sec,
            "occupied_threshold": int(_value(context, "alignment_occupied_threshold")),
            "lock_after_consensus": _bool_value(context, "alignment_lock_after_first"),
            "stop_processing_after_lock": _bool_value(context, "alignment_lock_after_first"),
            "use_sim_time": False,
        })
        actions.insert(0, Node(
            package="co_3dto2d_mapping",
            executable="inter_robot_place_alignment.py",
            name="inter_robot_place_alignment",
            output="screen",
            parameters=parameters,
        ))
    if enable_fusion and enable_record_republisher:
        actions.insert(1, Node(
            package="co_3dto2d_mapping",
            executable="record_republisher.py",
            name="toy_record_republisher",
            output="screen",
            parameters=[{
                **merged_dynamic_parameters(_value(context, "occupancy_config_file")),
                "target_frame_id": "odom",
                "common_frame_id": common_frame_id,
                "alignment_topic": alignment_topic,
                "publish_period_ms": int(_value(context, "record_publish_period_ms")),
                "output_prefix": _value(context, "record_output_prefix"),
                "robot_ids": [0, 1],
                "publish_tf": True,
                "publish_merged_global": _bool_value(context, "record_publish_merged_global"),
                "use_sim_time": False,
            }],
        ))
    return actions


def generate_launch_description():
    package_share = get_package_share_directory("co_3dto2d_mapping")
    return LaunchDescription([
        DeclareLaunchArgument("enable_robot0_pipeline", default_value="true"),
        DeclareLaunchArgument("enable_robot1_pipeline", default_value="true"),
        DeclareLaunchArgument("enable_fusion", default_value="true"),
        DeclareLaunchArgument(
            "enable_place_recognition",
            default_value="false",
            description="Run occupancy-map alignment on the fusion host.",
        ),
        DeclareLaunchArgument("robot0_lidar_topic", default_value="/r0/livox/lidar"),
        DeclareLaunchArgument("robot0_imu_topic", default_value="/r0/livox/imu"),
        DeclareLaunchArgument("robot1_lidar_topic", default_value="/r1/livox/lidar"),
        DeclareLaunchArgument("robot1_imu_topic", default_value="/r1/livox/imu"),
        DeclareLaunchArgument("robot0_imu_input_is_filtered", default_value="false"),
        DeclareLaunchArgument("robot1_imu_input_is_filtered", default_value="false"),
        DeclareLaunchArgument("expected_update_rate", default_value="10.0"),
        DeclareLaunchArgument("wait_imu_to_init", default_value="true"),
        DeclareLaunchArgument(
            "mapping_startup_delay_sec", default_value="10.0",
            description="Local sensor warm-up only; never waits for the other robot or alignment.",
        ),
        DeclareLaunchArgument("publish_sensor_static_tf", default_value="true"),
        DeclareLaunchArgument("enable_rear_lidar_filter", default_value="false"),
        DeclareLaunchArgument("rear_filter_angle_deg", default_value="120.0"),
        DeclareLaunchArgument("rear_filter_axis", default_value="-x"),
        DeclareLaunchArgument("rear_filter_min_xy_range_m", default_value="0.0"),
        DeclareLaunchArgument("rear_filter_log_period", default_value="100"),
        DeclareLaunchArgument("transform_cloud_to_local_frame", default_value="true"),
        DeclareLaunchArgument("center_box_filter_half_extent_m", default_value="0.80"),
        DeclareLaunchArgument("slice_z_in_cloud_frame", default_value="true"),
        *[
            DeclareLaunchArgument("sensor_tf_%s_%d" % (axis, rid), default_value=default)
            for rid in (0, 1)
            for axis, default in (
                ("x", "0"), ("y", "0"), ("z", "0"), ("yaw", "0"),
                ("pitch", "0"), ("roll", "3.141592653589793"),
            )
        ],
        DeclareLaunchArgument("common_frame_id", default_value="map"),
        DeclareLaunchArgument("alignment_topic", default_value="/toy/initial_xy_alignment"),
        DeclareLaunchArgument("alignment_config_file", default_value=""),
        DeclareLaunchArgument(
            "alignment_startup_delay_sec", default_value="3.0",
            description="Settle time inside the 2-D aligner after maps and odometry arrive; not a mapping gate.",
        ),
        DeclareLaunchArgument("alignment_occupied_threshold", default_value="50"),
        DeclareLaunchArgument("alignment_lock_after_first", default_value="true"),
        # Retain the old cloud-ICP CLI names so existing runners still launch.
        # They are deliberately NOT forwarded to the occupancy-only aligner.
        # Configure registration/consensus thresholds via alignment_config_file.
        *[
            DeclareLaunchArgument(name, default_value=default, description="Legacy cloud-ICP option; unused in live map-only registration.")
            for name, default in (
                ("alignment_use_z_filter", "true"),
                ("alignment_z_min", "0.4"), ("alignment_z_max", "0.8"),
                ("alignment_invert_z_slice", "true"),
                ("alignment_frame_count", "5"), ("alignment_invert_result", "false"),
                ("alignment_center_box_half_extent_m", "0.80"),
                ("alignment_range_min_m", "0.80"), ("alignment_range_max_m", "12.0"),
                ("alignment_voxel_size", "0.10"), ("alignment_max_points", "15000"),
                ("alignment_max_correspondence_distance", "0.75"),
                ("alignment_min_correspondences", "100"),
                ("alignment_min_fitness", "0.05"), ("alignment_max_rmse", "0.40"),
                ("alignment_max_iterations", "40"), ("alignment_recompute_period_sec", "5.0"),
                ("alignment_required_consistent_results", "2"),
                ("alignment_max_consistency_translation_m", "0.25"),
                ("alignment_max_consistency_rotation_rad", "0.08726646259971647"),
                ("alignment_initialize_from_centroids", "true"),
                ("alignment_enforce_tilt_prior", "true"),
                ("alignment_max_tilt_deviation_rad", "0.2617993877991494"),
            )
        ],
        DeclareLaunchArgument("enable_record_republisher", default_value="true"),
        DeclareLaunchArgument("record_publish_period_ms", default_value="200"),
        DeclareLaunchArgument("record_output_prefix", default_value="/toy_record"),
        DeclareLaunchArgument("record_publish_merged_global", default_value="true"),
        DeclareLaunchArgument("occupancy_config_file", default_value=os.path.join(package_share, "config", "occupancy.yaml")),
        OpaqueFunction(function=launch_setup),
    ])
