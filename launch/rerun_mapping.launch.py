import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    spawn_viewer = (
        LaunchConfiguration("spawn_viewer").perform(context).lower() == "true"
    )
    return [
        Node(
            package="co_3dto2d_mapping",
            executable="rerun_mapping_node.py",
            name="co_3dto2d_rerun",
            output="screen",
            parameters=[
                LaunchConfiguration("config_file").perform(context),
                {
                    "spawn_viewer": spawn_viewer,
                    "rerun_port": int(
                        LaunchConfiguration("rerun_port").perform(context)
                    ),
                },
            ],
        )
    ]


def generate_launch_description():
    package_share = get_package_share_directory("co_3dto2d_mapping")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=os.path.join(package_share, "config", "rerun.yaml"),
            ),
            DeclareLaunchArgument("spawn_viewer", default_value="true"),
            DeclareLaunchArgument("rerun_port", default_value="9876"),
            OpaqueFunction(function=launch_setup),
        ]
    )
