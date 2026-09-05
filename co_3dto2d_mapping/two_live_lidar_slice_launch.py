"""Compose current two-live mapping with distributed LiDAR-frame 2D startup maps."""

from __future__ import annotations

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from co_3dto2d_mapping.startup_lidar_launch_icp import (
    gate_process,
    local_startup_map_nodes,
    map_topic_instead_of_cloud,
    startup_2d_map_icp_node,
)


def _load_plane_layer():
    share = get_package_share_directory("co_3dto2d_mapping")
    path = os.path.join(share, "launch", "two_live_plane_height_mapping.launch.py")
    spec = importlib.util.spec_from_file_location(
        "co3dto2d_two_live_plane_layer", path
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("could not load two-live plane layer: %s" % path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_LAYER = _load_plane_layer()
_SOURCE_CLOUD_TOPIC = _LAYER._prealignment_cloud_topic

# Reuse the current ground-plane/local-window/place-recognition launch, changing
# only the startup producer and the gate's diagnostic input topics.
_LAYER._prealignment_cloud_topic = map_topic_instead_of_cloud
_LAYER._gate_process = lambda topic, name: gate_process(_LAYER, topic, name)
_LAYER._startup_icp_node = lambda context: startup_2d_map_icp_node(
    _LAYER, _SOURCE_CLOUD_TOPIC, context
)


def _local_startup_maps(context, *args, **kwargs):
    del args, kwargs
    return local_startup_map_nodes(
        _LAYER,
        _SOURCE_CLOUD_TOPIC,
        context,
    )


def generate_launch_description():
    layered = _LAYER.generate_launch_description()
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "startup_map_slice_center_z_m",
                default_value="0.0",
                description="Center of the startup slice in raw LiDAR z.",
            ),
            DeclareLaunchArgument(
                "startup_map_slice_half_height_m",
                default_value="0.40",
                description=(
                    "Symmetric half-height around raw LiDAR z=0 for the startup 2D map."
                ),
            ),
            DeclareLaunchArgument(
                "startup_map_half_extent_m",
                default_value="0.0",
                description=(
                    "Startup map half-width. Zero uses occupancy.yaml range_max_m."
                ),
            ),
            DeclareLaunchArgument(
                "startup_map_min_occupied_cells",
                default_value="100",
                description="Minimum occupied cells before publishing a startup map.",
            ),
            DeclareLaunchArgument(
                "startup_map_publish_period_sec",
                default_value="1.0",
                description="Republish period for each latched startup map.",
            ),
            *list(layered.entities),
            # This comes after all inherited DeclareLaunchArgument actions, so it
            # can inspect enable_robotN_pipeline. Each distributed laptop creates
            # only its own startup map; the fusion host receives the two maps.
            OpaqueFunction(function=_local_startup_maps),
        ]
    )
