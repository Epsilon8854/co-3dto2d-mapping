
from pathlib import Path as _WorkspacePath
import sys as _workspace_sys
_workspace_root = next(p for p in _WorkspacePath(__file__).resolve().parents if (p/"package.xml").is_file() and (p/"CMakeLists.txt").is_file())
_workspace_sys.path.insert(0, str(_workspace_root))
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    package_share = str(_workspace_root)
    robot_id = LaunchConfiguration("robot_id").perform(context)
    namespace = "/r" + robot_id
    mapping_startup_delay_sec = float(
        LaunchConfiguration("mapping_startup_delay_sec").perform(context)
    )
    if mapping_startup_delay_sec < 0.0:
        raise RuntimeError("mapping_startup_delay_sec must be non-negative")

    raw_scan_cloud_topic = LaunchConfiguration("scan_cloud_topic").perform(context)
    icp_scan_cloud_topic = LaunchConfiguration("icp_scan_cloud_topic").perform(
        context
    )
    plane_filtered_cloud_topic = LaunchConfiguration(
        "plane_filtered_cloud_topic"
    ).perform(context)
    imu_filtered_topic = LaunchConfiguration("imu_filtered_topic").perform(context)
    local_frame_id = LaunchConfiguration("local_frame_id").perform(context)
    occupancy_config_file = LaunchConfiguration("occupancy_config_file").perform(
        context
    )
    mapping_odometry_topic = LaunchConfiguration(
        "mapping_odometry_topic"
    ).perform(context)
    corrected_odometry_topic = LaunchConfiguration(
        "corrected_odometry_topic"
    ).perform(context)
    if raw_scan_cloud_topic == plane_filtered_cloud_topic:
        raise RuntimeError(
            "scan_cloud_topic and plane_filtered_cloud_topic must differ to avoid "
            "a point-cloud feedback loop"
        )
    if mapping_odometry_topic in {"odom", corrected_odometry_topic}:
        raise RuntimeError(
            "mapping_odometry_topic must differ from raw odom and the mapper's "
            "corrected_odometry_topic"
        )

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_share,
                "scripts/experiment/support/launch",
                "mid360_mapping_pipeline.launch.py",
            )
        ),
        launch_arguments={
            "robot_id": robot_id,
            "use_bag": LaunchConfiguration("use_bag").perform(context),
            "use_sim_time": LaunchConfiguration("use_sim_time").perform(context),
            "publish_tf_odom": LaunchConfiguration("publish_tf_odom").perform(
                context
            ),
            "play_tf_static": LaunchConfiguration("play_tf_static").perform(context),
            "publish_sensor_static_tf": LaunchConfiguration(
                "publish_sensor_static_tf"
            ).perform(context),
            "sensor_parent_frame": LaunchConfiguration("sensor_parent_frame").perform(
                context
            ),
            "sensor_child_frame": LaunchConfiguration("sensor_child_frame").perform(
                context
            ),
            "sensor_tf_x": LaunchConfiguration("sensor_tf_x").perform(context),
            "sensor_tf_y": LaunchConfiguration("sensor_tf_y").perform(context),
            "sensor_tf_z": LaunchConfiguration("sensor_tf_z").perform(context),
            "sensor_tf_yaw": LaunchConfiguration("sensor_tf_yaw").perform(context),
            "sensor_tf_pitch": LaunchConfiguration("sensor_tf_pitch").perform(
                context
            ),
            "sensor_tf_roll": LaunchConfiguration("sensor_tf_roll").perform(context),
            "expected_update_rate": LaunchConfiguration(
                "expected_update_rate"
            ).perform(context),
            "icp_topic_queue_size": LaunchConfiguration(
                "icp_topic_queue_size"
            ).perform(context),
            "odometry_config_file": LaunchConfiguration(
                "odometry_config_file"
            ).perform(context),
            "wait_imu_to_init": LaunchConfiguration("wait_imu_to_init").perform(
                context
            ),
            "mapping_startup_delay_sec": str(mapping_startup_delay_sec),
            "imu_input_is_filtered": LaunchConfiguration(
                "imu_input_is_filtered"
            ).perform(context),
            "bag_path": LaunchConfiguration("bag_path").perform(context),
            "rate": LaunchConfiguration("rate").perform(context),
            "storage_id": LaunchConfiguration("storage_id").perform(context),
            "enable_rear_lidar_filter": LaunchConfiguration(
                "enable_rear_lidar_filter"
            ).perform(context),
            "rear_filter_angle_deg": LaunchConfiguration(
                "rear_filter_angle_deg"
            ).perform(context),
            "rear_filter_axis": LaunchConfiguration("rear_filter_axis").perform(
                context
            ),
            "rear_filter_min_xy_range_m": LaunchConfiguration(
                "rear_filter_min_xy_range_m"
            ).perform(context),
            "rear_filter_log_period": LaunchConfiguration(
                "rear_filter_log_period"
            ).perform(context),
            "bag_lidar_topic": LaunchConfiguration("bag_lidar_topic").perform(
                context
            ),
            "scan_cloud_topic": raw_scan_cloud_topic,
            "icp_scan_cloud_topic": icp_scan_cloud_topic,
            "imu_raw_topic": LaunchConfiguration("imu_raw_topic").perform(context),
            "imu_filtered_topic": imu_filtered_topic,
        }.items(),
    )

    use_sim_time = (
        LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    )

    # The shared floor node consumes raw RTAB-Map odometry and the uncut cloud.
    # It publishes a plane-filtered cloud and a time-matched mapping odometry.
    # ground_plane_pose_enabled controls whether z/roll/pitch are replaced; when
    # disabled, raw odometry is republished on the same topic for a clean ablation.
    ground_plane_pose_node = Node(
        package="co_3dto2d_mapping",
        executable=str(_workspace_root/"scripts/experiment/support/legacy_floor/gravity_plane_height_cloud.py"),
        name="gravity_plane_pose_fusion",
        namespace=namespace,
        output="screen",
        remappings=[("tf", "/tf"), ("tf_static", "/tf_static")],
        parameters=[
            occupancy_config_file,
            {
                "ground_plane_pointcloud_topic": raw_scan_cloud_topic,
                "ground_plane_filtered_cloud_topic": plane_filtered_cloud_topic,
                "ground_plane_imu_topic": imu_filtered_topic,
                "ground_plane_planar_odometry_topic": "odom",
                "ground_plane_output_odometry_topic": mapping_odometry_topic,
                "ground_plane_local_frame_id": local_frame_id,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    # The mapper now receives the exact odometry state produced from the same
    # plane observation as the filtered cloud. Local-window ICP only refines
    # x/y/yaw and publishes the final mapping pose directly.
    mapper_node = Node(
        package="co_3dto2d_mapping",
        executable=str(_workspace_root/"outputs/experiment_support/legacy_mapper/occupancy_mapper"),
        name="occupancy_mapper",
        namespace=namespace,
        output="screen",
        parameters=[
            occupancy_config_file,
            {
                "scan_cloud_topic": plane_filtered_cloud_topic,
                "odom_topic": mapping_odometry_topic,
                "local_frame_id": local_frame_id,
                "global_frame_id": LaunchConfiguration("global_frame_id").perform(
                    context
                ),
                "use_odom_header_frame": (
                    LaunchConfiguration("use_odom_header_frame")
                    .perform(context)
                    .lower()
                    == "true"
                ),
                "alignment_required": (
                    LaunchConfiguration("alignment_required")
                    .perform(context)
                    .lower()
                    == "true"
                ),
                "alignment_topic": LaunchConfiguration("alignment_topic").perform(
                    context
                ),
                "transform_cloud_to_local_frame": True,
                "center_box_filter_half_extent_m": float(
                    LaunchConfiguration("center_box_filter_half_extent_m").perform(
                        context
                    )
                ),
                "slice_in_global_frame": False,
                "slice_z_in_cloud_frame": True,
                "invert_z_slice": False,
                "z_min": -1000.0,
                "z_max": 1000.0,
                "log_z_slice_stats": False,
                "publish_corrected_odometry": True,
                "corrected_odometry_topic": corrected_odometry_topic,
                "local_submap_diagnostics_csv_path": LaunchConfiguration(
                    "local_submap_diagnostics_csv_path"
                ).perform(context),
                "local_pose_graph_diagnostics_csv_path": LaunchConfiguration(
                    "local_pose_graph_diagnostics_csv_path"
                ).perform(context),
                "local_pose_graph_visualization_dir": LaunchConfiguration(
                    "local_pose_graph_visualization_dir"
                ).perform(context),
                "local_line_pose_graph_diagnostics_csv_path": LaunchConfiguration(
                    "local_line_pose_graph_diagnostics_csv_path"
                ).perform(context),
                "local_line_pose_graph_visualization_dir": LaunchConfiguration(
                    "local_line_pose_graph_visualization_dir"
                ).perform(context),
                "local_pht_submap_diagnostics_csv_path": LaunchConfiguration(
                    "local_pht_submap_diagnostics_csv_path"
                ).perform(context),
                "local_pht_submap_visualization_dir": LaunchConfiguration(
                    "local_pht_submap_visualization_dir"
                ).perform(context),
                "local_pht_submap_pose_graph_mode": LaunchConfiguration(
                    "local_pht_submap_pose_graph_mode"
                ).perform(context),
                "use_sim_time": use_sim_time,
            },
        ],
    )

    mapping_actions = [ground_plane_pose_node, mapper_node]
    if mapping_startup_delay_sec > 0.0:
        return [
            base_launch,
            TimerAction(
                period=mapping_startup_delay_sec,
                actions=mapping_actions,
            ),
        ]
    return [base_launch, *mapping_actions]


def generate_launch_description():
    package_share = str(_workspace_root)
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
            DeclareLaunchArgument("icp_topic_queue_size", default_value="1"),
            DeclareLaunchArgument(
                "odometry_config_file",
                default_value=os.path.join(
                    package_share, "config", "lidar_odometry.yaml"
                ),
            ),
            DeclareLaunchArgument("wait_imu_to_init", default_value="true"),
            DeclareLaunchArgument("mapping_startup_delay_sec", default_value="0.0"),
            DeclareLaunchArgument("imu_input_is_filtered", default_value="false"),
            DeclareLaunchArgument("bag_path", default_value=""),
            DeclareLaunchArgument("rate", default_value="1.0"),
            DeclareLaunchArgument("storage_id", default_value="sqlite3"),
            DeclareLaunchArgument("enable_rear_lidar_filter", default_value="true"),
            DeclareLaunchArgument("rear_filter_angle_deg", default_value="120.0"),
            DeclareLaunchArgument("rear_filter_axis", default_value="-x"),
            DeclareLaunchArgument("rear_filter_min_xy_range_m", default_value="0.0"),
            DeclareLaunchArgument("rear_filter_log_period", default_value="100"),
            DeclareLaunchArgument("bag_lidar_topic", default_value="/livox/lidar_raw"),
            DeclareLaunchArgument("scan_cloud_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("icp_scan_cloud_topic", default_value=""),
            DeclareLaunchArgument("imu_raw_topic", default_value="/livox/imu"),
            DeclareLaunchArgument(
                "imu_filtered_topic", default_value="/livox/imu_filtered"
            ),
            DeclareLaunchArgument("alignment_required", default_value="false"),
            DeclareLaunchArgument(
                "alignment_topic", default_value="/mapping/initial_xy_alignment"
            ),
            DeclareLaunchArgument(
                "transform_cloud_to_local_frame", default_value="true"
            ),
            DeclareLaunchArgument(
                "center_box_filter_half_extent_m", default_value="0.80"
            ),
            # Kept only for CLI compatibility. The active mapper receives an
            # already plane-filtered cloud and no longer uses this fixed-Z mode.
            DeclareLaunchArgument("slice_z_in_cloud_frame", default_value="true"),
            DeclareLaunchArgument(
                "plane_filtered_cloud_topic",
                default_value="mapping/plane_height_filtered",
            ),
            DeclareLaunchArgument(
                "mapping_odometry_topic",
                default_value="mapping/floor_based_odometry",
            ),
            # Deprecated compatibility argument. The mapper now publishes the
            # final pose directly on corrected_odometry_topic.
            DeclareLaunchArgument(
                "planar_odometry_topic", default_value="odom"
            ),
            DeclareLaunchArgument(
                "corrected_odometry_topic", default_value="mapping/occupancy_mapper_odometry"
            ),
            DeclareLaunchArgument("local_submap_diagnostics_csv_path", default_value=""),
            DeclareLaunchArgument("local_pose_graph_diagnostics_csv_path", default_value=""),
            DeclareLaunchArgument("local_pose_graph_visualization_dir", default_value=""),
            DeclareLaunchArgument("local_line_pose_graph_diagnostics_csv_path", default_value=""),
            DeclareLaunchArgument("local_line_pose_graph_visualization_dir", default_value=""),
            DeclareLaunchArgument("local_pht_submap_diagnostics_csv_path", default_value=""),
            DeclareLaunchArgument("local_pht_submap_visualization_dir", default_value=""),
            DeclareLaunchArgument(
                "local_pht_submap_pose_graph_mode", default_value="visualization_only"
            ),
            DeclareLaunchArgument(
                "occupancy_config_file",
                default_value=os.path.join(package_share, "config/experiment/common", "occupancy_legacy.yaml"),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
