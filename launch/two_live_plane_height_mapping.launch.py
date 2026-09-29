"""Compatibility entry point for occupancy-only two-robot live mapping.

CMake installs this file as two_live_mapping.launch.py and the implementation
as two_live_mapping_base.launch.py. There is no cloud startup aligner, process
exit barrier, or dependency on remote sensor TF here. Floor filtering and
LiDAR odometry remain inside each independent local mapping pipeline.
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _load_base_module():
    package_share = get_package_share_directory("co_3dto2d_mapping")
    path = os.path.join(package_share, "launch", "two_live_mapping_base.launch.py")
    spec = importlib.util.spec_from_file_location("co3dto2d_two_live_base", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("could not load two-live base launch: %s" % path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_BASE = _load_base_module()


def _legacy_startup_notice(context):
    value = LaunchConfiguration("wait_for_initial_alignment").perform(context).strip().lower()
    if value not in {"0", "1", "false", "true", "no", "yes", "off", "on"}:
        raise RuntimeError("wait_for_initial_alignment must be a boolean value")
    if value in {"1", "true", "yes", "on"}:
        return [LogInfo(msg=(
            "wait_for_initial_alignment is deprecated and ignored. Local mapping "
            "starts independently; inter-robot alignment uses 2-D occupancy maps only."
        ))]
    return []


def generate_launch_description():
    # Keep old launch arguments accepted, but never re-enable a 3-D startup
    # dependency. The physical runner may still supply the old timeout and
    # CO3DTO2D_STARTUP_DIRECT_LIDAR; neither selects an input in this path.
    legacy = (
        ("wait_for_initial_alignment", "false"),
        ("startup_alignment_topic", "/toy/startup_xy_alignment"),
        ("startup_alignment_timeout_sec", "0.0"),
        ("startup_alignment_status_period_sec", "2.0"),
        ("startup_alignment_required_consistent_results", "1"),
    )
    return LaunchDescription([
        *[
            DeclareLaunchArgument(name, default_value=default,
                                  description="Deprecated startup-gate option; no effect on map-only registration.")
            for name, default in legacy
        ],
        OpaqueFunction(function=_legacy_startup_notice),
        *list(_BASE.generate_launch_description().entities),
    ])
