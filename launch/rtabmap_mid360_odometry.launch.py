import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    namespace = LaunchConfiguration("namespace").perform(context)
    use_sim_time = (
        LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    )
    startup_delay_sec = float(
        LaunchConfiguration("startup_delay_sec").perform(context)
    )
    if startup_delay_sec < 0.0:
        raise RuntimeError("startup_delay_sec must be non-negative")

    odometry_config_file = LaunchConfiguration("odometry_config_file").perform(
        context
    )
    if not os.path.isfile(odometry_config_file):
        raise RuntimeError(
            "odometry_config_file does not exist: %s" % odometry_config_file
        )

    runtime_parameters = {
        "frame_id": LaunchConfiguration("frame_id").perform(context),
        "odom_frame_id": LaunchConfiguration("odom_topic").perform(context),
        "publish_tf": LaunchConfiguration("publish_tf").perform(context).lower() == "true",
        "use_sim_time": use_sim_time,
        "expected_update_rate": float(
            LaunchConfiguration("expected_update_rate").perform(context)
        ),
        "wait_imu_to_init": LaunchConfiguration("wait_imu_to_init").perform(
            context
        ).lower() == "true",
    }

    odometry_node = Node(
        package="rtabmap_odom",
        executable="icp_odometry",
        output="screen",
        name="mid360_icp_odometry",
        namespace=namespace,
        # Later dictionaries override YAML values. Only per-run frame, timing,
        # and topic-adjacent settings belong in runtime_parameters.
        parameters=[odometry_config_file, runtime_parameters],
        remappings=[
            ("scan_cloud", LaunchConfiguration("scan_cloud_topic").perform(context)),
            ("imu", LaunchConfiguration("imu_topic").perform(context)),
            ("odom", LaunchConfiguration("odom_topic").perform(context)),
        ],
    )
    if startup_delay_sec > 0.0:
        return [TimerAction(period=startup_delay_sec, actions=[odometry_node])]
    return [odometry_node]


def generate_launch_description():
    package_share = get_package_share_directory("co_3dto2d_mapping")
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="/r0"),
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            DeclareLaunchArgument("odom_topic", default_value="odom"),
            DeclareLaunchArgument("scan_cloud_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("imu_topic", default_value="/livox/imu"),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            DeclareLaunchArgument("wait_imu_to_init", default_value="true"),
            DeclareLaunchArgument("expected_update_rate", default_value="10.0"),
            DeclareLaunchArgument("startup_delay_sec", default_value="0.0"),
            DeclareLaunchArgument(
                "odometry_config_file",
                default_value=os.path.join(
                    package_share, "config", "lidar_odometry.yaml"
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=launch_setup),
        ]
    )
