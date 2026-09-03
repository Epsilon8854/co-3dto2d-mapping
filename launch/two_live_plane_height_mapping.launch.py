"""Public two-live wrapper with shared-floor mapping and fixed 2-D alignment.

The base launch owns both robot pipelines and the record/merged-map outputs. This
wrapper keeps the plane-height topic contract and replaces the legacy startup
cloud aligner with CPU-only occupancy place recognition. The result is one fixed
``map <- r1/odom`` transform; no pose graph or submap optimization is introduced
at this stage.
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory


_FILTERED_SUFFIX = "/mapping/plane_height_filtered"


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
_ORIGINAL_ROBOT_ACTIONS = _BASE._robot_actions
_ORIGINAL_BOOL_VALUE = _BASE._bool_value
_ORIGINAL_NODE = _BASE.Node


def _plane_filtered_robot_actions(
    context,
    robot_id,
    live_launch_path,
    enable_rear_lidar_filter,
):
    republisher, pipeline, _raw_scan_topic = _ORIGINAL_ROBOT_ACTIONS(
        context,
        robot_id,
        live_launch_path,
        enable_rear_lidar_filter,
    )
    filtered_topic = "/r%d%s" % (robot_id, _FILTERED_SUFFIX)
    return republisher, pipeline, filtered_topic


def _plane_height_bool_value(context, name):
    if name == "alignment_use_z_filter":
        return False
    return _ORIGINAL_BOOL_VALUE(context, name)


def _place_recognition_node(*args, **kwargs):
    """Replace only the base launch's inter-robot alignment node."""

    if not (
        kwargs.get("package") == "co_3dto2d_mapping"
        and kwargs.get("executable") == "initial_xy_icp_alignment.py"
    ):
        return _ORIGINAL_NODE(*args, **kwargs)

    forwarded = {}
    for parameter_set in kwargs.get("parameters", []):
        if isinstance(parameter_set, dict):
            forwarded.update(parameter_set)

    package_share = get_package_share_directory("co_3dto2d_mapping")
    place_config = os.path.join(package_share, "config", "place_recognition.yaml")
    overrides = {
        "robot0_map_topic": forwarded.get(
            "robot0_map_topic", "/r0/toy/global_occupancy"
        ),
        "robot1_map_topic": forwarded.get(
            "robot1_map_topic", "/r1/toy/global_occupancy"
        ),
        "robot0_odom_topic": "/r0/toy/corrected_odometry",
        "robot1_odom_topic": "/r1/toy/corrected_odometry",
        "alignment_topic": forwarded.get(
            "alignment_topic", "/toy/initial_xy_alignment"
        ),
        "target_frame_id": forwarded.get("target_frame_id", "map"),
        "source_frame_id": forwarded.get("source_frame_id", "r1/odom"),
        "startup_delay_sec": forwarded.get("startup_delay_sec", 3.0),
        "occupied_threshold": forwarded.get("occupied_threshold", 50),
        "lock_after_consensus": forwarded.get(
            "lock_after_first_alignment", True
        ),
        "stop_processing_after_lock": forwarded.get(
            "lock_after_first_alignment", True
        ),
        "use_sim_time": forwarded.get("use_sim_time", False),
    }

    rewritten = dict(kwargs)
    rewritten["executable"] = "inter_robot_place_alignment.py"
    rewritten["name"] = "inter_robot_place_alignment"
    rewritten["parameters"] = [place_config, overrides]
    return _ORIGINAL_NODE(*args, **rewritten)


_BASE._robot_actions = _plane_filtered_robot_actions
_BASE._bool_value = _plane_height_bool_value
_BASE.Node = _place_recognition_node


def generate_launch_description():
    return _BASE.generate_launch_description()
