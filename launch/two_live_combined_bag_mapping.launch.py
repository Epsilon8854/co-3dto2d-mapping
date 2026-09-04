import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


TRUE_VALUES = {"1", "true", "yes", "on"}
FALSE_VALUES = {"0", "false", "no", "off"}


def value(context, name):
    return LaunchConfiguration(name).perform(context)


def boolean(context, name):
    result = value(context, name).strip().lower()
    if result in TRUE_VALUES:
        return True
    if result in FALSE_VALUES:
        return False
    raise RuntimeError(f"{name} must be boolean, got {result!r}")


def robot_actions(context, robot_id, single_launch):
    prefix = f"robot{robot_id}"
    namespace = f"/r{robot_id}"
    sensor_parent = f"r{robot_id}/base_link"
    sensor_child = f"r{robot_id}/livox_frame"
    lidar_input = value(context, f"{prefix}_lidar_topic")
    imu_input = value(context, f"{prefix}_imu_topic")
    scan_topic = f"{namespace}/mapping/lidar"
    lidar_relay = (
        f"{namespace}/mapping/lidar_unfiltered"
        if boolean(context, "enable_rear_lidar_filter")
        else scan_topic
    )
    filtered_imu = f"{namespace}/mapping/imu_filtered"

    relay = Node(
        package="co_3dto2d_mapping",
        executable="pointcloud_frame_republisher.py",
        name=f"pointcloud_frame_republisher_r{robot_id}",
        output="screen",
        parameters=[{
            "input_topic": lidar_input,
            "output_topic": lidar_relay,
            "output_frame_id": sensor_child,
        }],
    )
    pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(single_launch),
        launch_arguments={
            "robot_id": str(robot_id),
            "use_bag": "false",
            "use_sim_time": "false",
            "publish_tf_odom": "false",
            "play_tf_static": "false",
            "publish_sensor_static_tf": value(context, "publish_sensor_static_tf"),
            "sensor_parent_frame": sensor_parent,
            "sensor_child_frame": sensor_child,
            "local_frame_id": sensor_parent,
            "global_frame_id": f"r{robot_id}/odom",
            "use_odom_header_frame": "false",
            "sensor_tf_x": value(context, f"sensor_tf_x_{robot_id}"),
            "sensor_tf_y": value(context, f"sensor_tf_y_{robot_id}"),
            "sensor_tf_z": value(context, f"sensor_tf_z_{robot_id}"),
            "sensor_tf_yaw": value(context, f"sensor_tf_yaw_{robot_id}"),
            "sensor_tf_pitch": value(context, f"sensor_tf_pitch_{robot_id}"),
            "sensor_tf_roll": value(context, f"sensor_tf_roll_{robot_id}"),
            "expected_update_rate": value(context, "expected_update_rate"),
            "wait_imu_to_init": value(context, "wait_imu_to_init"),
            "mapping_startup_delay_sec": value(context, "mapping_startup_delay_sec"),
            "imu_input_is_filtered": value(
                context, f"{prefix}_imu_input_is_filtered"
            ),
            "enable_rear_lidar_filter": value(context, "enable_rear_lidar_filter"),
            "rear_filter_angle_deg": value(context, "rear_filter_angle_deg"),
            "rear_filter_axis": value(context, "rear_filter_axis"),
            "rear_filter_min_xy_range_m": value(context, "rear_filter_min_xy_range_m"),
            "rear_filter_log_period": value(context, "rear_filter_log_period"),
            "bag_lidar_topic": lidar_relay,
            "scan_cloud_topic": scan_topic,
            "imu_raw_topic": imu_input,
            "imu_filtered_topic": filtered_imu,
            "alignment_required": "false",
            "alignment_topic": value(context, "alignment_topic"),
            "transform_cloud_to_local_frame": value(
                context, "transform_cloud_to_local_frame"
            ),
            "center_box_filter_half_extent_m": value(
                context, "center_box_filter_half_extent_m"
            ),
            "slice_z_in_cloud_frame": value(context, "slice_z_in_cloud_frame"),
            "occupancy_config_file": value(context, "occupancy_config_file"),
        }.items(),
    )
    return [pipeline, relay]


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    package_share = get_package_share_directory("co_3dto2d_mapping")
    single_launch = os.path.join(
        package_share, "launch", "single_bag_mapping.launch.py"
    )
    actions = robot_actions(context, 0, single_launch)
    actions.extend(robot_actions(context, 1, single_launch))

    alignment_topic = value(context, "alignment_topic")
    common_frame = value(context, "common_frame_id")
    alignment_config_file = value(context, "alignment_config_file")
    alignment = Node(
        package="co_3dto2d_mapping",
        executable="initial_xy_icp_alignment.py",
        name="initial_xy_icp_alignment",
        output="screen",
        parameters=[{
            "robot0_cloud_topic": "/r0/mapping/lidar",
            "robot1_cloud_topic": "/r1/mapping/lidar",
            "robot0_map_topic": "/r0/toy/global_occupancy",
            "robot1_map_topic": "/r1/toy/global_occupancy",
            "input_mode": "global_occupancy",
            "alignment_topic": alignment_topic,
            "target_frame_id": common_frame,
            "source_frame_id": "r1/odom",
            "local_frame_id": "r0/base_link",
            "transform_cloud_to_local_frame": False,
            "z_min": float(value(context, "alignment_z_min")),
            "z_max": float(value(context, "alignment_z_max")),
            "invert_z_slice": boolean(context, "alignment_invert_z_slice"),
            "frame_count": int(value(context, "alignment_frame_count")),
            "invert_result": boolean(context, "alignment_invert_result"),
            "center_box_half_extent_m": float(
                value(context, "alignment_center_box_half_extent_m")
            ),
            "voxel_size": float(value(context, "alignment_voxel_size")),
            "max_points": int(value(context, "alignment_max_points")),
            "max_correspondence_distance": float(
                value(context, "alignment_max_correspondence_distance")
            ),
            "min_correspondences": int(
                value(context, "alignment_min_correspondences")
            ),
            "min_fitness": float(value(context, "alignment_min_fitness")),
            "max_rmse": float(value(context, "alignment_max_rmse")),
            "max_iterations": int(value(context, "alignment_max_iterations")),
            "recompute_period_sec": float(
                value(context, "alignment_recompute_period_sec")
            ),
            "occupied_threshold": int(
                value(context, "alignment_occupied_threshold")
            ),
            "startup_delay_sec": float(
                value(context, "alignment_startup_delay_sec")
            ),
            "retry_on_failure": True,
            "lock_after_first_alignment": boolean(
                context, "alignment_lock_after_first"
            ),
            "required_consistent_results": int(
                value(context, "alignment_required_consistent_results")
            ),
            "max_consistency_translation_m": float(
                value(context, "alignment_max_consistency_translation_m")
            ),
            "max_consistency_rotation_rad": float(
                value(context, "alignment_max_consistency_rotation_rad")
            ),
            "initialize_from_centroids": boolean(
                context, "alignment_initialize_from_centroids"
            ),
            "use_sim_time": False,
        }] + ([alignment_config_file] if alignment_config_file else []),
    )
    actions.insert(0, alignment)

    if boolean(context, "enable_record_republisher"):
        actions.insert(1, Node(
            package="co_3dto2d_mapping",
            executable="record_republisher.py",
            name="toy_record_republisher",
            output="screen",
            parameters=[{
                "target_frame_id": "odom",
                "common_frame_id": common_frame,
                "alignment_topic": alignment_topic,
                "publish_period_ms": int(value(context, "record_publish_period_ms")),
                "output_prefix": value(context, "record_output_prefix"),
                "robot_ids": [0, 1],
                "publish_tf": True,
                "publish_merged_global": boolean(
                    context, "record_publish_merged_global"
                ),
                "use_sim_time": False,
            }],
        ))
    return actions


DEFAULTS = {
    "robot0_lidar_topic": "/co_3dto2d_replay/r0/lidar",
    "robot0_imu_topic": "/co_3dto2d_replay/r0/imu",
    "robot1_lidar_topic": "/co_3dto2d_replay/r1/lidar",
    "robot1_imu_topic": "/co_3dto2d_replay/r1/imu",
    "robot0_imu_input_is_filtered": "false",
    "robot1_imu_input_is_filtered": "false",
    "expected_update_rate": "0.0",
    "wait_imu_to_init": "false",
    "mapping_startup_delay_sec": "10.0",
    "publish_sensor_static_tf": "true",
    "enable_rear_lidar_filter": "false",
    "rear_filter_angle_deg": "120.0",
    "rear_filter_axis": "-x",
    "rear_filter_min_xy_range_m": "0.0",
    "rear_filter_log_period": "100",
    "transform_cloud_to_local_frame": "true",
    "center_box_filter_half_extent_m": "0.80",
    "slice_z_in_cloud_frame": "true",
    "common_frame_id": "map",
    "alignment_topic": "/toy/initial_xy_alignment",
    "alignment_config_file": "",
    "alignment_startup_delay_sec": "3.0",
    "alignment_z_min": "0.4",
    "alignment_z_max": "0.8",
    "alignment_invert_z_slice": "true",
    "alignment_frame_count": "5",
    "alignment_invert_result": "false",
    "alignment_center_box_half_extent_m": "0.80",
    "alignment_voxel_size": "0.05",
    "alignment_max_points": "30000",
    "alignment_max_correspondence_distance": "0.75",
    "alignment_min_correspondences": "100",
    "alignment_min_fitness": "0.05",
    "alignment_max_rmse": "0.40",
    "alignment_max_iterations": "80",
    "alignment_recompute_period_sec": "2.0",
    "alignment_occupied_threshold": "50",
    "alignment_lock_after_first": "true",
    "alignment_required_consistent_results": "2",
    "alignment_max_consistency_translation_m": "0.25",
    "alignment_max_consistency_rotation_rad": "0.08726646259971647",
    "alignment_initialize_from_centroids": "true",
    "enable_record_republisher": "true",
    "record_publish_period_ms": "200",
    "record_output_prefix": "/toy_record",
    "record_publish_merged_global": "true",
}
for robot_id in (0, 1):
    for component in ("x", "y", "z", "yaw", "pitch"):
        DEFAULTS[f"sensor_tf_{component}_{robot_id}"] = "0"
    DEFAULTS[f"sensor_tf_roll_{robot_id}"] = "3.141592653589793"


def generate_launch_description():
    package_share = get_package_share_directory("co_3dto2d_mapping")
    defaults = dict(DEFAULTS)
    defaults["occupancy_config_file"] = os.path.join(package_share, "config", "occupancy.yaml")
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default) for name, default in defaults.items()]
        + [OpaqueFunction(function=launch_setup)]
    )
