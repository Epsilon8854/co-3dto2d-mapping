
from pathlib import Path as _WorkspacePath
import sys as _workspace_sys
_workspace_root = next(p for p in _WorkspacePath(__file__).resolve().parents if (p/"package.xml").is_file() and (p/"CMakeLists.txt").is_file())
_workspace_sys.path.insert(0, str(_workspace_root))
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    package_share = str(_workspace_root)
    robot_id = LaunchConfiguration("robot_id").perform(context)
    namespace = "/r" + robot_id
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    occupancy_config_file = LaunchConfiguration("occupancy_config_file").perform(context)
    if not os.path.isfile(occupancy_config_file):
        raise RuntimeError("occupancy_config_file does not exist: %s" % occupancy_config_file)
    return [
        Node(
            package="co_3dto2d_mapping",
            executable=str(_workspace_root/"outputs/experiment_support/legacy_mapper/occupancy_mapper"),
            name="occupancy_mapper",
            namespace=namespace,
            output="screen",
            parameters=[
                occupancy_config_file,
                {
                    "scan_cloud_topic": LaunchConfiguration("scan_cloud_topic").perform(context),
                    "odom_topic": LaunchConfiguration("odom_topic").perform(context),
                    "local_frame_id": LaunchConfiguration("local_frame_id").perform(context),
                    "global_frame_id": LaunchConfiguration("global_frame_id").perform(context),
                    "transform_cloud_to_local_frame": True,
                    "slice_in_global_frame": False,
                    "slice_z_in_cloud_frame": True,
                    "invert_z_slice": False,
                    "z_min": -1000.0,
                    "z_max": 1000.0,
                    "log_z_slice_stats": False,
                    "publish_corrected_odometry": True,
                    "corrected_odometry_topic": LaunchConfiguration("corrected_odometry_topic").perform(context),
                    "local_pht_submap_diagnostics_csv_path": LaunchConfiguration("local_pht_submap_diagnostics_csv_path").perform(context),
                    "local_pht_submap_visualization_dir": LaunchConfiguration("local_pht_submap_visualization_dir").perform(context),
                    "local_pht_submap_pose_graph_mode": LaunchConfiguration("local_pht_submap_pose_graph_mode").perform(context),
                    "local_pht_submap_history_keyframes": int(LaunchConfiguration("local_pht_submap_history_keyframes").perform(context)),
                    "use_sim_time": use_sim_time,
                },
            ],
        )
    ]


def generate_launch_description():
    package_share = str(_workspace_root)
    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("scan_cloud_topic", default_value="/r0/mapping/plane_height_filtered"),
        DeclareLaunchArgument("odom_topic", default_value="/r0/mapping/floor_based_odometry"),
        DeclareLaunchArgument("local_frame_id", default_value="r0/base_link"),
        DeclareLaunchArgument("global_frame_id", default_value="odom"),
        DeclareLaunchArgument("corrected_odometry_topic", default_value="/r0/mapping/occupancy_mapper_odometry"),
        DeclareLaunchArgument("local_pht_submap_diagnostics_csv_path", default_value=""),
        DeclareLaunchArgument("local_pht_submap_visualization_dir", default_value=""),
        DeclareLaunchArgument("local_pht_submap_pose_graph_mode", default_value="visualization_only"),
        DeclareLaunchArgument("local_pht_submap_history_keyframes", default_value="10"),
        DeclareLaunchArgument("occupancy_config_file", default_value=os.path.join(package_share, "config/experiment/common", "occupancy_legacy.yaml")),
        OpaqueFunction(function=launch_setup),
    ])
