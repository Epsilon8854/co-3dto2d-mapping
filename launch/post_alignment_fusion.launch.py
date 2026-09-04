"""Start common-frame republishing and merged-map fusion after alignment.

The two-live shell runner invokes this launch only after the transient-local
initial alignment has been observed.  Keeping it separate prevents pre-alignment
odom/map warnings from being interleaved with the initial ICP logs.
"""

from __future__ import annotations

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


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    return [
        LogInfo(
            msg=(
                "[STAGE 2/2] Initial alignment is ready. Starting common-frame "
                "odometry republishing and merged occupancy fusion."
            )
        ),
        Node(
            package="co_3dto2d_mapping",
            executable="record_republisher.py",
            name="toy_record_republisher",
            output="screen",
            parameters=[
                {
                    "target_frame_id": "odom",
                    "common_frame_id": _value(context, "common_frame_id"),
                    "alignment_topic": _value(context, "alignment_topic"),
                    "publish_period_ms": int(
                        _value(context, "record_publish_period_ms")
                    ),
                    "output_prefix": _value(context, "record_output_prefix"),
                    "robot_ids": [0, 1],
                    "publish_tf": True,
                    "publish_merged_global": _boolean(
                        context, "record_publish_merged_global"
                    ),
                    "publish_world_odometry": True,
                    "suppress_unaligned_world_odometry": True,
                    "publish_world_maps": True,
                    "suppress_unaligned_world_maps": True,
                    "lock_world_alignment": True,
                    "use_sim_time": False,
                }
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "alignment_topic", default_value="/toy/initial_xy_alignment"
            ),
            DeclareLaunchArgument("common_frame_id", default_value="map"),
            DeclareLaunchArgument("record_publish_period_ms", default_value="200"),
            DeclareLaunchArgument(
                "record_output_prefix", default_value="/toy_record"
            ),
            DeclareLaunchArgument(
                "record_publish_merged_global", default_value="true"
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
