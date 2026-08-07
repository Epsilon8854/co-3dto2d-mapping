import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    bag_path = LaunchConfiguration("bag_path").perform(context)
    if not bag_path:
        raise RuntimeError("bag_path is required")
    metadata_path = os.path.join(bag_path, "metadata.yaml")
    if not os.path.isfile(metadata_path):
        raise RuntimeError(f"bag metadata not found: {bag_path}")

    play_tf_static = (
        LaunchConfiguration("play_tf_static").perform(context).lower() == "true"
    )

    cmd = [
        "ros2",
        "bag",
        "play",
        "-s",
        LaunchConfiguration("storage_id").perform(context),
        bag_path,
        "-r",
        LaunchConfiguration("rate").perform(context),
        "--topics",
        "/livox/lidar",
        "/livox/imu",
    ]

    if play_tf_static:
        cmd.append("/tf_static")

    cmd.extend(
        [
            "--remap",
            "/livox/lidar:=" + LaunchConfiguration("lidar_topic").perform(context),
            "/livox/imu:=" + LaunchConfiguration("imu_topic").perform(context),
        ]
    )

    return [
        ExecuteProcess(
            cmd=cmd,
            name="mid360_bag_play",
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_path",
                default_value="",
                description="Path to the MID-360 rosbag2 directory.",
            ),
            DeclareLaunchArgument("rate", default_value="1.0"),
            DeclareLaunchArgument("storage_id", default_value="sqlite3"),
            DeclareLaunchArgument("lidar_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("imu_topic", default_value="/livox/imu"),
            DeclareLaunchArgument("play_tf_static", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
