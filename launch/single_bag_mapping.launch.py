import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    package_share = get_package_share_directory("co_3dto2d_mapping")
    robot_id = LaunchConfiguration("robot_id").perform(context)
    namespace = "/r" + robot_id

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_share,
                "launch",
                "mid360_mapping_pipeline.launch.py",
            )
        ),
        launch_arguments={
            "robot_id": robot_id,
            "use_bag": LaunchConfiguration("use_bag").perform(context),
            "use_sim_time": LaunchConfiguration("use_sim_time").perform(context),
            "publish_tf_odom": LaunchConfiguration("publish_tf_odom").perform(context),
            "play_tf_static": LaunchConfiguration("play_tf_static").perform(context),
            "publish_sensor_static_tf": LaunchConfiguration("publish_sensor_static_tf").perform(context),
            "sensor_parent_frame": LaunchConfiguration("sensor_parent_frame").perform(context),
            "sensor_child_frame": LaunchConfiguration("sensor_child_frame").perform(context),
            "sensor_tf_x": LaunchConfiguration("sensor_tf_x").perform(context),
            "sensor_tf_y": LaunchConfiguration("sensor_tf_y").perform(context),
            "sensor_tf_z": LaunchConfiguration("sensor_tf_z").perform(context),
            "sensor_tf_yaw": LaunchConfiguration("sensor_tf_yaw").perform(context),
            "sensor_tf_pitch": LaunchConfiguration("sensor_tf_pitch").perform(context),
            "sensor_tf_roll": LaunchConfiguration("sensor_tf_roll").perform(context),
            "expected_update_rate": LaunchConfiguration("expected_update_rate").perform(context),
            "bag_path": LaunchConfiguration("bag_path").perform(context),
            "rate": LaunchConfiguration("rate").perform(context),
            "storage_id": LaunchConfiguration("storage_id").perform(context),
            "enable_rear_lidar_filter": LaunchConfiguration("enable_rear_lidar_filter").perform(context),
            "rear_filter_angle_deg": LaunchConfiguration("rear_filter_angle_deg").perform(context),
            "rear_filter_axis": LaunchConfiguration("rear_filter_axis").perform(context),
            "rear_filter_min_xy_range_m": LaunchConfiguration("rear_filter_min_xy_range_m").perform(context),
            "rear_filter_log_period": LaunchConfiguration("rear_filter_log_period").perform(context),
            "bag_lidar_topic": LaunchConfiguration("bag_lidar_topic").perform(context),
            "scan_cloud_topic": LaunchConfiguration("scan_cloud_topic").perform(context),
            "imu_raw_topic": LaunchConfiguration("imu_raw_topic").perform(context),
            "imu_filtered_topic": LaunchConfiguration("imu_filtered_topic").perform(context),
        }.items(),
    )

    use_sim_time = (
        LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    )

    mapper_node = Node(
        package="co_3dto2d_mapping",
        executable="occupancy_mapper",
        name="occupancy_mapper",
        namespace=namespace,
        output="screen",
        parameters=[
            LaunchConfiguration("occupancy_config_file").perform(context),
            {
                "scan_cloud_topic": LaunchConfiguration("scan_cloud_topic").perform(context),
                "odom_topic": "odom",
                "local_frame_id": LaunchConfiguration("local_frame_id").perform(context),
                "global_frame_id": LaunchConfiguration("global_frame_id").perform(context),
                "use_odom_header_frame": (
                    LaunchConfiguration("use_odom_header_frame").perform(context).lower()
                    == "true"
                ),
                "alignment_required": (
                    LaunchConfiguration("alignment_required").perform(context).lower()
                    == "true"
                ),
                "alignment_topic": LaunchConfiguration("alignment_topic").perform(context),
                "transform_cloud_to_local_frame": (
                    LaunchConfiguration("transform_cloud_to_local_frame")
                    .perform(context)
                    .lower()
                    == "true"
                ),
                "center_box_filter_half_extent_m": float(
                    LaunchConfiguration("center_box_filter_half_extent_m").perform(context)
                ),
                "slice_z_in_cloud_frame": (
                    LaunchConfiguration("slice_z_in_cloud_frame")
                    .perform(context)
                    .lower()
                    == "true"
                ),
                "use_sim_time": use_sim_time,
            },
        ],
    )

    return [base_launch, mapper_node]


def generate_launch_description():
    package_share = get_package_share_directory("co_3dto2d_mapping")
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_id", default_value="0"),
            DeclareLaunchArgument("use_bag", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("publish_tf_odom", default_value="true"),
            DeclareLaunchArgument("play_tf_static", default_value="true"),
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
            DeclareLaunchArgument(
                "bag_path",
                default_value="",
            ),
            DeclareLaunchArgument("rate", default_value="1.0"),
            DeclareLaunchArgument("storage_id", default_value="sqlite3"),
            DeclareLaunchArgument("enable_rear_lidar_filter", default_value="true"),
            DeclareLaunchArgument("rear_filter_angle_deg", default_value="120.0"),
            DeclareLaunchArgument("rear_filter_axis", default_value="-x"),
            DeclareLaunchArgument("rear_filter_min_xy_range_m", default_value="0.0"),
            DeclareLaunchArgument("rear_filter_log_period", default_value="100"),
            DeclareLaunchArgument("bag_lidar_topic", default_value="/livox/lidar_raw"),
            DeclareLaunchArgument("scan_cloud_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("imu_raw_topic", default_value="/livox/imu"),
            DeclareLaunchArgument(
                "imu_filtered_topic", default_value="/livox/imu_filtered"
            ),
            DeclareLaunchArgument("alignment_required", default_value="false"),
            DeclareLaunchArgument("alignment_topic", default_value="/toy/initial_xy_alignment"),
            DeclareLaunchArgument("transform_cloud_to_local_frame", default_value="true"),
            DeclareLaunchArgument("center_box_filter_half_extent_m", default_value="0.80"),
            DeclareLaunchArgument("slice_z_in_cloud_frame", default_value="true"),
            DeclareLaunchArgument(
                "occupancy_config_file",
                default_value=os.path.join(package_share, "config", "occupancy.yaml"),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
