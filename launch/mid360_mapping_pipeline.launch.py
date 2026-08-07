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
    publish_sensor_static_tf = (
        LaunchConfiguration("publish_sensor_static_tf").perform(context).lower() == "true"
    )
    enable_rear_lidar_filter = (
        LaunchConfiguration("enable_rear_lidar_filter").perform(context).lower() == "true"
    )
    scan_cloud_topic = LaunchConfiguration("scan_cloud_topic").perform(context)
    imu_filtered_topic = LaunchConfiguration("imu_filtered_topic").perform(context)
    imu_filter_output_topic = imu_filtered_topic + "_raw_frame"
    bag_lidar_topic = (
        LaunchConfiguration("bag_lidar_topic").perform(context)
        if enable_rear_lidar_filter
        else scan_cloud_topic
    )

    bag_proc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_share,
                "launch",
                "bag_mid360.launch.py",
            )
        ),
        launch_arguments={
            "bag_path": LaunchConfiguration("bag_path").perform(context),
            "rate": LaunchConfiguration("rate").perform(context),
            "storage_id": LaunchConfiguration("storage_id").perform(context),
            "lidar_topic": bag_lidar_topic,
            "imu_topic": LaunchConfiguration("imu_raw_topic").perform(context),
            "play_tf_static": LaunchConfiguration("play_tf_static").perform(context),
        }.items(),
    )

    use_sim_time = (
        LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    )

    imu_filter_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "use_mag": False,
                "publish_tf": False,
                "reverse_tf": False,
                "world_frame": "enu",
                "remove_gravity_vector": False,
                "use_sim_time": use_sim_time,
            }
        ],
        remappings=[
            ("imu/data_raw", LaunchConfiguration("imu_raw_topic").perform(context)),
            ("imu/data", imu_filter_output_topic),
        ],
    )

    imu_frame_republisher_node = Node(
        package="co_3dto2d_mapping",
        executable="imu_frame_republisher.py",
        name="imu_frame_republisher",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "input_topic": imu_filter_output_topic,
                "output_topic": imu_filtered_topic,
                "output_frame_id": LaunchConfiguration("sensor_child_frame").perform(context),
                "use_sim_time": use_sim_time,
            }
        ],
    )

    rear_lidar_filter_node = Node(
        package="co_3dto2d_mapping",
        executable="pointcloud_rear_sector_filter.py",
        name="pointcloud_rear_sector_filter",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "input_topic": bag_lidar_topic,
                "output_topic": scan_cloud_topic,
                "enabled": True,
                "rear_filter_angle_deg": float(
                    LaunchConfiguration("rear_filter_angle_deg").perform(context)
                ),
                "rear_axis": LaunchConfiguration("rear_filter_axis").perform(context),
                "min_xy_range_m": float(
                    LaunchConfiguration("rear_filter_min_xy_range_m").perform(context)
                ),
                "log_period": int(
                    LaunchConfiguration("rear_filter_log_period").perform(context)
                ),
                "output_frame_id": LaunchConfiguration("sensor_child_frame").perform(context),
                "use_sim_time": use_sim_time,
            }
        ],
    )

    odom_proc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_share,
                "launch",
                "rtabmap_mid360_odometry.launch.py",
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "frame_id": LaunchConfiguration("sensor_parent_frame").perform(context),
            "odom_topic": "odom",
            "scan_cloud_topic": scan_cloud_topic,
            "imu_topic": LaunchConfiguration("imu_filtered_topic").perform(context),
            "wait_imu_to_init": "true",
            "expected_update_rate": LaunchConfiguration("expected_update_rate").perform(context),
            "publish_tf": LaunchConfiguration("publish_tf_odom").perform(context),
            "use_sim_time": LaunchConfiguration("use_sim_time").perform(context),
        }.items(),
    )

    actions = [bag_proc, imu_filter_node, imu_frame_republisher_node, odom_proc]
    if enable_rear_lidar_filter:
        actions.insert(1, rear_lidar_filter_node)
    if publish_sensor_static_tf:
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="mid360_sensor_static_tf_r%s" % robot_id,
                output="screen",
                arguments=[
                    LaunchConfiguration("sensor_tf_x").perform(context),
                    LaunchConfiguration("sensor_tf_y").perform(context),
                    LaunchConfiguration("sensor_tf_z").perform(context),
                    LaunchConfiguration("sensor_tf_yaw").perform(context),
                    LaunchConfiguration("sensor_tf_pitch").perform(context),
                    LaunchConfiguration("sensor_tf_roll").perform(context),
                    LaunchConfiguration("sensor_parent_frame").perform(context),
                    LaunchConfiguration("sensor_child_frame").perform(context),
                ],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_id", default_value="0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("publish_tf_odom", default_value="true"),
            DeclareLaunchArgument("play_tf_static", default_value="true"),
            DeclareLaunchArgument("publish_sensor_static_tf", default_value="true"),
            DeclareLaunchArgument("sensor_parent_frame", default_value="base_link"),
            DeclareLaunchArgument("sensor_child_frame", default_value="livox_frame"),
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
            OpaqueFunction(function=launch_setup),
        ]
    )
