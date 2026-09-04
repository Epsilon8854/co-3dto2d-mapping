"""Alignment-only startup stage for the two-MID360 live workflow.

This launch intentionally starts no odometry or occupancy mapper.  It consumes
both raw, namespaced LiDAR streams, publishes the two sensor static transforms
when requested, and runs the cropped-XYZ initial alignment with the same YAML
file used by the occupancy mapper.  The runner keeps this launch alive after a
valid transform is published so late transient-local subscribers receive it.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_TRUE_VALUES = {"1", "true", "yes", "on"}
_FALSE_VALUES = {"0", "false", "no", "off"}


def _value(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def _boolean(context, name: str) -> bool:
    value = _value(context, name).strip().lower()
    if value in _TRUE_VALUES:
        return True
    if value in _FALSE_VALUES:
        return False
    raise RuntimeError("%s must be boolean, got %r" % (name, value))


def _static_transform_node(context, robot_id: int) -> Node:
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="initial_alignment_sensor_static_tf_r%d" % robot_id,
        output="screen",
        arguments=[
            _value(context, "sensor_tf_x_%d" % robot_id),
            _value(context, "sensor_tf_y_%d" % robot_id),
            _value(context, "sensor_tf_z_%d" % robot_id),
            _value(context, "sensor_tf_yaw_%d" % robot_id),
            _value(context, "sensor_tf_pitch_%d" % robot_id),
            _value(context, "sensor_tf_roll_%d" % robot_id),
            "r%d/base_link" % robot_id,
            "r%d/livox_frame" % robot_id,
        ],
    )


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    config_file = _value(context, "mapping_config_file")
    if not config_file or not os.path.isfile(config_file):
        raise RuntimeError(
            "mapping_config_file must point to a readable occupancy YAML: %r"
            % config_file
        )

    startup_delay_sec = float(_value(context, "startup_delay_sec"))
    if startup_delay_sec < 0.0:
        raise RuntimeError("startup_delay_sec must be non-negative")

    actions = [
        LogInfo(
            msg=(
                "[STAGE 1/2] Initial cropped-XYZ alignment is the only active "
                "estimator. RTAB-Map odometry and occupancy mapping remain "
                "stopped until %s is published. Alignment parameters are read "
                "from %s."
                % (_value(context, "alignment_topic"), config_file)
            )
        )
    ]
    if _boolean(context, "publish_sensor_static_tf"):
        actions.extend(
            [_static_transform_node(context, 0), _static_transform_node(context, 1)]
        )

    actions.append(
        Node(
            package="co_3dto2d_mapping",
            executable="initial_xy_icp_alignment.py",
            name="initial_xy_icp_alignment",
            output="screen",
            remappings=[("tf", "/tf"), ("tf_static", "/tf_static")],
            parameters=[
                config_file,
                {
                    "robot0_cloud_topic": _value(context, "robot0_cloud_topic"),
                    "robot1_cloud_topic": _value(context, "robot1_cloud_topic"),
                    "input_mode": "cloud_initial",
                    "alignment_topic": _value(context, "alignment_topic"),
                    "target_frame_id": _value(context, "common_frame_id"),
                    "source_frame_id": "r1/odom",
                    "local_frame_id": "base_link",
                    "robot0_local_frame_id": "r0/base_link",
                    "robot1_local_frame_id": "r1/base_link",
                    "transform_cloud_to_local_frame": True,
                    # These two safety overrides prevent the legacy inverse-Z
                    # path even when an older custom YAML omits the dedicated
                    # /initial_xy_icp_alignment section.  All crop/range/ICP
                    # thresholds still come from mapping_config_file.
                    "use_z_filter": False,
                    "invert_z_slice": False,
                    "startup_delay_sec": startup_delay_sec,
                    "retry_on_failure": True,
                    "lock_after_first_alignment": True,
                    "use_sim_time": False,
                },
            ],
        )
    )
    return actions


def generate_launch_description():
    package_share = get_package_share_directory("co_3dto2d_mapping")
    declarations = [
        DeclareLaunchArgument(
            "mapping_config_file",
            default_value=os.path.join(package_share, "config", "occupancy.yaml"),
        ),
        DeclareLaunchArgument("robot0_cloud_topic", default_value="/r0/livox/lidar"),
        DeclareLaunchArgument("robot1_cloud_topic", default_value="/r1/livox/lidar"),
        DeclareLaunchArgument(
            "alignment_topic", default_value="/toy/initial_xy_alignment"
        ),
        DeclareLaunchArgument("common_frame_id", default_value="map"),
        DeclareLaunchArgument("startup_delay_sec", default_value="10.0"),
        DeclareLaunchArgument("publish_sensor_static_tf", default_value="true"),
    ]
    for robot_id in (0, 1):
        declarations.extend(
            [
                DeclareLaunchArgument(
                    "sensor_tf_x_%d" % robot_id, default_value="0"
                ),
                DeclareLaunchArgument(
                    "sensor_tf_y_%d" % robot_id, default_value="0"
                ),
                DeclareLaunchArgument(
                    "sensor_tf_z_%d" % robot_id, default_value="0"
                ),
                DeclareLaunchArgument(
                    "sensor_tf_yaw_%d" % robot_id, default_value="0"
                ),
                DeclareLaunchArgument(
                    "sensor_tf_pitch_%d" % robot_id, default_value="0"
                ),
                DeclareLaunchArgument(
                    "sensor_tf_roll_%d" % robot_id,
                    default_value="3.141592653589793",
                ),
            ]
        )
    return LaunchDescription(declarations + [OpaqueFunction(function=launch_setup)])
