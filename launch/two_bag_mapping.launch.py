import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    package_share = get_package_share_directory("co_3dto2d_mapping")
    launch_path = os.path.join(
        package_share,
        "launch",
        "single_bag_mapping.launch.py",
    )

    robot_delay_s = float(LaunchConfiguration("robot_delay_s").perform(context))
    rate = LaunchConfiguration("rate").perform(context)
    storage_id = LaunchConfiguration("storage_id").perform(context)
    occupancy_config_file = LaunchConfiguration("occupancy_config_file").perform(context)
    publish_sensor_static_tf = LaunchConfiguration("publish_sensor_static_tf").perform(context)
    enable_rear_lidar_filter = LaunchConfiguration("enable_rear_lidar_filter").perform(context)
    rear_filter_angle_deg = LaunchConfiguration("rear_filter_angle_deg").perform(context)
    rear_filter_axis = LaunchConfiguration("rear_filter_axis").perform(context)
    rear_filter_min_xy_range_m = LaunchConfiguration("rear_filter_min_xy_range_m").perform(context)
    rear_filter_log_period = LaunchConfiguration("rear_filter_log_period").perform(context)
    transform_cloud_to_local_frame = LaunchConfiguration(
        "transform_cloud_to_local_frame"
    ).perform(context)
    center_box_filter_half_extent_m = LaunchConfiguration(
        "center_box_filter_half_extent_m"
    ).perform(context)
    slice_z_in_cloud_frame = LaunchConfiguration("slice_z_in_cloud_frame").perform(context)
    alignment_topic = LaunchConfiguration("alignment_topic").perform(context)
    alignment_startup_delay_s = float(
        LaunchConfiguration("alignment_startup_delay_s").perform(context)
    )
    enable_record_republisher = (
        LaunchConfiguration("enable_record_republisher").perform(context).lower()
        == "true"
    )
    record_publish_merged_global = (
        LaunchConfiguration("record_publish_merged_global").perform(context).lower()
        == "true"
    )

    robot0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments={
            "robot_id": "0",
            "occupancy_config_file": occupancy_config_file,
            "bag_path": LaunchConfiguration("bag_path_0").perform(context),
            "rate": rate,
            "storage_id": storage_id,
            "use_sim_time": "false",
            "publish_tf_odom": "false",
            "play_tf_static": "true",
            "publish_sensor_static_tf": publish_sensor_static_tf,
            "enable_rear_lidar_filter": enable_rear_lidar_filter,
            "rear_filter_angle_deg": rear_filter_angle_deg,
            "rear_filter_axis": rear_filter_axis,
            "rear_filter_min_xy_range_m": rear_filter_min_xy_range_m,
            "rear_filter_log_period": rear_filter_log_period,
            "sensor_parent_frame": "r0/base_link",
            "sensor_child_frame": "r0/livox_frame",
            "local_frame_id": "r0/base_link",
            "global_frame_id": "r0/odom",
            "use_odom_header_frame": "false",
            "sensor_tf_x": LaunchConfiguration("sensor_tf_x").perform(context),
            "sensor_tf_y": LaunchConfiguration("sensor_tf_y").perform(context),
            "sensor_tf_z": LaunchConfiguration("sensor_tf_z").perform(context),
            "sensor_tf_yaw": LaunchConfiguration("sensor_tf_yaw").perform(context),
            "sensor_tf_pitch": LaunchConfiguration("sensor_tf_pitch").perform(context),
            "sensor_tf_roll": LaunchConfiguration("sensor_tf_roll").perform(context),
            "expected_update_rate": "0.0",
            "bag_lidar_topic": "/r0/livox/lidar_raw",
            "scan_cloud_topic": "/r0/livox/lidar",
            "imu_raw_topic": "/r0/livox/imu",
            "imu_filtered_topic": "/r0/livox/imu_filtered",
            "alignment_required": "false",
            "alignment_topic": alignment_topic,
            "transform_cloud_to_local_frame": transform_cloud_to_local_frame,
            "center_box_filter_half_extent_m": center_box_filter_half_extent_m,
            "slice_z_in_cloud_frame": slice_z_in_cloud_frame,
        }.items(),
    )

    robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments={
            "robot_id": "1",
            "occupancy_config_file": occupancy_config_file,
            "bag_path": LaunchConfiguration("bag_path_1").perform(context),
            "rate": rate,
            "storage_id": storage_id,
            "use_sim_time": "false",
            "publish_tf_odom": "false",
            "play_tf_static": "false",
            "publish_sensor_static_tf": publish_sensor_static_tf,
            "enable_rear_lidar_filter": enable_rear_lidar_filter,
            "rear_filter_angle_deg": rear_filter_angle_deg,
            "rear_filter_axis": rear_filter_axis,
            "rear_filter_min_xy_range_m": rear_filter_min_xy_range_m,
            "rear_filter_log_period": rear_filter_log_period,
            "sensor_parent_frame": "r1/base_link",
            "sensor_child_frame": "r1/livox_frame",
            "local_frame_id": "r1/base_link",
            "global_frame_id": "r1/odom",
            "use_odom_header_frame": "false",
            "sensor_tf_x": LaunchConfiguration("sensor_tf_x").perform(context),
            "sensor_tf_y": LaunchConfiguration("sensor_tf_y").perform(context),
            "sensor_tf_z": LaunchConfiguration("sensor_tf_z").perform(context),
            "sensor_tf_yaw": LaunchConfiguration("sensor_tf_yaw").perform(context),
            "sensor_tf_pitch": LaunchConfiguration("sensor_tf_pitch").perform(context),
            "sensor_tf_roll": LaunchConfiguration("sensor_tf_roll").perform(context),
            "expected_update_rate": "0.0",
            "bag_lidar_topic": "/r1/livox/lidar_raw",
            "scan_cloud_topic": "/r1/livox/lidar",
            "imu_raw_topic": "/r1/livox/imu",
            "imu_filtered_topic": "/r1/livox/imu_filtered",
            "alignment_required": "false",
            "alignment_topic": alignment_topic,
            "transform_cloud_to_local_frame": transform_cloud_to_local_frame,
            "center_box_filter_half_extent_m": center_box_filter_half_extent_m,
            "slice_z_in_cloud_frame": slice_z_in_cloud_frame,
        }.items(),
    )

    alignment_node = Node(
        package="co_3dto2d_mapping",
        executable="initial_xy_icp_alignment.py",
        name="initial_xy_icp_alignment",
        output="screen",
        parameters=[
            {
                "robot0_cloud_topic": "/r0/livox/lidar",
                "robot1_cloud_topic": "/r1/livox/lidar",
                "robot0_map_topic": "/r0/toy/global_occupancy",
                "robot1_map_topic": "/r1/toy/global_occupancy",
                "input_mode": "global_occupancy",
                "alignment_topic": alignment_topic,
                "target_frame_id": "map",
                "source_frame_id": "r1/odom",
                "local_frame_id": LaunchConfiguration("sensor_parent_frame").perform(context),
                "transform_cloud_to_local_frame": (
                    LaunchConfiguration("transform_cloud_to_local_frame")
                    .perform(context)
                    .lower()
                    == "true"
                ),
                "z_min": float(LaunchConfiguration("alignment_z_min").perform(context)),
                "z_max": float(LaunchConfiguration("alignment_z_max").perform(context)),
                "invert_z_slice": (
                    LaunchConfiguration("alignment_invert_z_slice")
                    .perform(context)
                    .lower()
                    == "true"
                ),
                "frame_count": int(
                    LaunchConfiguration("alignment_frame_count").perform(context)
                ),
                "invert_result": (
                    LaunchConfiguration("alignment_invert_result")
                    .perform(context)
                    .lower()
                    == "true"
                ),
                "center_box_half_extent_m": float(
                    LaunchConfiguration("alignment_center_box_half_extent_m").perform(context)
                ),
                "voxel_size": float(
                    LaunchConfiguration("alignment_voxel_size").perform(context)
                ),
                "max_points": int(
                    LaunchConfiguration("alignment_max_points").perform(context)
                ),
                "max_correspondence_distance": float(
                    LaunchConfiguration("alignment_max_correspondence_distance").perform(context)
                ),
                "min_correspondences": int(
                    LaunchConfiguration("alignment_min_correspondences").perform(context)
                ),
                "min_fitness": float(
                    LaunchConfiguration("alignment_min_fitness").perform(context)
                ),
                "max_rmse": float(LaunchConfiguration("alignment_max_rmse").perform(context)),
                "max_iterations": int(
                    LaunchConfiguration("alignment_max_iterations").perform(context)
                ),
                "recompute_period_sec": float(
                    LaunchConfiguration("alignment_recompute_period_sec").perform(context)
                ),
                "occupied_threshold": int(
                    LaunchConfiguration("alignment_occupied_threshold").perform(context)
                ),
            }
        ],
    )

    record_republisher = Node(
        package="co_3dto2d_mapping",
        executable="record_republisher.py",
        name="toy_record_republisher",
        output="screen",
        parameters=[
            {
                "target_frame_id": "odom",
                "common_frame_id": "map",
                "alignment_topic": alignment_topic,
                "publish_period_ms": int(
                    LaunchConfiguration("record_publish_period_ms").perform(context)
                ),
                "output_prefix": LaunchConfiguration("record_output_prefix").perform(context),
                "robot_ids": [0, 1],
                "publish_tf": True,
                "publish_merged_global": record_publish_merged_global,
            }
        ],
    )

    actions = [
        alignment_node,
        TimerAction(period=alignment_startup_delay_s, actions=[robot0]),
        TimerAction(period=alignment_startup_delay_s + robot_delay_s, actions=[robot1]),
    ]
    if enable_record_republisher:
        actions.append(record_republisher)
    return actions


def generate_launch_description():
    package_share = get_package_share_directory("co_3dto2d_mapping")
    return LaunchDescription(
        [
            DeclareLaunchArgument("rate", default_value="1.0"),
            DeclareLaunchArgument("storage_id", default_value="sqlite3"),
            DeclareLaunchArgument("robot_delay_s", default_value="20.0"),
            DeclareLaunchArgument("enable_record_republisher", default_value="true"),
            DeclareLaunchArgument("record_publish_period_ms", default_value="200"),
            DeclareLaunchArgument("record_output_prefix", default_value="/toy_record"),
            DeclareLaunchArgument("record_publish_merged_global", default_value="true"),
            DeclareLaunchArgument("publish_sensor_static_tf", default_value="true"),
            DeclareLaunchArgument("enable_rear_lidar_filter", default_value="true"),
            DeclareLaunchArgument("rear_filter_angle_deg", default_value="120.0"),
            DeclareLaunchArgument("rear_filter_axis", default_value="-x"),
            DeclareLaunchArgument("rear_filter_min_xy_range_m", default_value="0.0"),
            DeclareLaunchArgument("rear_filter_log_period", default_value="100"),
            DeclareLaunchArgument("transform_cloud_to_local_frame", default_value="true"),
            DeclareLaunchArgument("center_box_filter_half_extent_m", default_value="0.80"),
            DeclareLaunchArgument("slice_z_in_cloud_frame", default_value="true"),
            DeclareLaunchArgument("alignment_topic", default_value="/toy/initial_xy_alignment"),
            DeclareLaunchArgument("alignment_startup_delay_s", default_value="1.0"),
            DeclareLaunchArgument("alignment_z_min", default_value="0.4"),
            DeclareLaunchArgument("alignment_z_max", default_value="1.2"),
            DeclareLaunchArgument("alignment_invert_z_slice", default_value="true"),
            DeclareLaunchArgument("alignment_frame_count", default_value="5"),
            DeclareLaunchArgument("alignment_invert_result", default_value="false"),
            DeclareLaunchArgument("alignment_center_box_half_extent_m", default_value="0.80"),
            DeclareLaunchArgument("alignment_voxel_size", default_value="0.05"),
            DeclareLaunchArgument("alignment_max_points", default_value="30000"),
            DeclareLaunchArgument(
                "alignment_max_correspondence_distance", default_value="0.75"
            ),
            DeclareLaunchArgument("alignment_min_correspondences", default_value="100"),
            DeclareLaunchArgument("alignment_min_fitness", default_value="0.05"),
            DeclareLaunchArgument("alignment_max_rmse", default_value="0.40"),
            DeclareLaunchArgument("alignment_max_iterations", default_value="80"),
            DeclareLaunchArgument("alignment_recompute_period_sec", default_value="5.0"),
            DeclareLaunchArgument("alignment_occupied_threshold", default_value="50"),
            DeclareLaunchArgument("sensor_parent_frame", default_value="base_link"),
            DeclareLaunchArgument("sensor_child_frame", default_value="livox_frame"),
            DeclareLaunchArgument("sensor_tf_x", default_value="0"),
            DeclareLaunchArgument("sensor_tf_y", default_value="0"),
            DeclareLaunchArgument("sensor_tf_z", default_value="0"),
            DeclareLaunchArgument("sensor_tf_yaw", default_value="0"),
            DeclareLaunchArgument("sensor_tf_pitch", default_value="0"),
            DeclareLaunchArgument("sensor_tf_roll", default_value="3.141592653589793"),
            DeclareLaunchArgument(
                "occupancy_config_file",
                default_value=os.path.join(package_share, "config", "occupancy.yaml"),
            ),
            DeclareLaunchArgument(
                "bag_path_0",
                default_value="",
            ),
            DeclareLaunchArgument(
                "bag_path_1",
                default_value="",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
