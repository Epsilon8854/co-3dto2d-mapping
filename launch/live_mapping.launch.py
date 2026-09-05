import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory("co_3dto2d_mapping")
    single_launch = os.path.join(
        package_share,
        "launch",
        "single_bag_mapping.launch.py",
    )

    launch_arguments = {
        "robot_id": LaunchConfiguration("robot_id"),
        "use_bag": "false",
        "bag_path": "",
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "publish_tf_odom": LaunchConfiguration("publish_tf_odom"),
        "play_tf_static": "false",
        "publish_sensor_static_tf": LaunchConfiguration("publish_sensor_static_tf"),
        "sensor_parent_frame": LaunchConfiguration("sensor_parent_frame"),
        "sensor_child_frame": LaunchConfiguration("sensor_child_frame"),
        "local_frame_id": LaunchConfiguration("local_frame_id"),
        "global_frame_id": LaunchConfiguration("global_frame_id"),
        "use_odom_header_frame": LaunchConfiguration("use_odom_header_frame"),
        "sensor_tf_x": LaunchConfiguration("sensor_tf_x"),
        "sensor_tf_y": LaunchConfiguration("sensor_tf_y"),
        "sensor_tf_z": LaunchConfiguration("sensor_tf_z"),
        "sensor_tf_yaw": LaunchConfiguration("sensor_tf_yaw"),
        "sensor_tf_pitch": LaunchConfiguration("sensor_tf_pitch"),
        "sensor_tf_roll": LaunchConfiguration("sensor_tf_roll"),
        "expected_update_rate": LaunchConfiguration("expected_update_rate"),
        "wait_imu_to_init": LaunchConfiguration("wait_imu_to_init"),
        "mapping_startup_delay_sec": LaunchConfiguration(
            "mapping_startup_delay_sec"
        ),
        "rate": "1.0",
        "storage_id": "sqlite3",
        "enable_rear_lidar_filter": LaunchConfiguration("enable_rear_lidar_filter"),
        "rear_filter_angle_deg": LaunchConfiguration("rear_filter_angle_deg"),
        "rear_filter_axis": LaunchConfiguration("rear_filter_axis"),
        "rear_filter_min_xy_range_m": LaunchConfiguration("rear_filter_min_xy_range_m"),
        "rear_filter_log_period": LaunchConfiguration("rear_filter_log_period"),
        "bag_lidar_topic": LaunchConfiguration("bag_lidar_topic"),
        "scan_cloud_topic": LaunchConfiguration("scan_cloud_topic"),
        "imu_raw_topic": LaunchConfiguration("imu_raw_topic"),
        "imu_filtered_topic": LaunchConfiguration("imu_filtered_topic"),
        "imu_input_is_filtered": LaunchConfiguration("imu_input_is_filtered"),
        "alignment_required": "false",
        "alignment_topic": LaunchConfiguration("alignment_topic"),
        "transform_cloud_to_local_frame": LaunchConfiguration(
            "transform_cloud_to_local_frame"
        ),
        "center_box_filter_half_extent_m": LaunchConfiguration(
            "center_box_filter_half_extent_m"
        ),
        "slice_z_in_cloud_frame": LaunchConfiguration("slice_z_in_cloud_frame"),
        "occupancy_config_file": LaunchConfiguration("occupancy_config_file"),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_id", default_value="0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("publish_tf_odom", default_value="true"),
            DeclareLaunchArgument("publish_sensor_static_tf", default_value="true"),
            DeclareLaunchArgument("sensor_parent_frame", default_value="base_link"),
            DeclareLaunchArgument("sensor_child_frame", default_value="livox_frame"),
            DeclareLaunchArgument("local_frame_id", default_value="base_link"),
            DeclareLaunchArgument("global_frame_id", default_value="odom"),
            DeclareLaunchArgument("use_odom_header_frame", default_value="true"),
            DeclareLaunchArgument("sensor_tf_x", default_value="0"),
            DeclareLaunchArgument("sensor_tf_y", default_value="0"),
            DeclareLaunchArgument("sensor_tf_z", default_value="0"),
            DeclareLaunchArgument("sensor_tf_yaw", default_value="0"),
            DeclareLaunchArgument("sensor_tf_pitch", default_value="0"),
            DeclareLaunchArgument("sensor_tf_roll", default_value="3.141592653589793"),
            DeclareLaunchArgument("expected_update_rate", default_value="10.0"),
            DeclareLaunchArgument("wait_imu_to_init", default_value="true"),
            DeclareLaunchArgument("mapping_startup_delay_sec", default_value="0.0"),
            DeclareLaunchArgument("enable_rear_lidar_filter", default_value="false"),
            DeclareLaunchArgument("rear_filter_angle_deg", default_value="120.0"),
            DeclareLaunchArgument("rear_filter_axis", default_value="-x"),
            DeclareLaunchArgument("rear_filter_min_xy_range_m", default_value="0.0"),
            DeclareLaunchArgument("rear_filter_log_period", default_value="100"),
            DeclareLaunchArgument("bag_lidar_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("scan_cloud_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("imu_raw_topic", default_value="/livox/imu"),
            DeclareLaunchArgument(
                "imu_filtered_topic",
                default_value="/livox/imu_filtered",
            ),
            DeclareLaunchArgument("imu_input_is_filtered", default_value="false"),
            DeclareLaunchArgument(
                "alignment_topic",
                default_value="/toy/initial_xy_alignment",
            ),
            DeclareLaunchArgument("transform_cloud_to_local_frame", default_value="true"),
            DeclareLaunchArgument("center_box_filter_half_extent_m", default_value="0.80"),
            DeclareLaunchArgument("slice_z_in_cloud_frame", default_value="true"),
            DeclareLaunchArgument(
                "occupancy_config_file",
                default_value=os.path.join(package_share, "config", "occupancy.yaml"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(single_launch),
                launch_arguments=launch_arguments.items(),
            ),
        ]
    )
