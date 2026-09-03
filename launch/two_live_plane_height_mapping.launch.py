"""Runtime wrapper for two_live_mapping using plane-height-filtered clouds.

The large base launch remains unchanged for source compatibility. CMake installs
it as ``two_live_mapping_base.launch.py`` and installs this file under the public
``two_live_mapping.launch.py`` name.
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
    # gravity_plane_height_cloud.py publishes this topic in each robot's
    # namespace before occupancy/local ICP and initial inter-robot alignment.
    filtered_topic = "/r%d%s" % (robot_id, _FILTERED_SUFFIX)
    return republisher, pipeline, filtered_topic


def _plane_height_bool_value(context, name):
    if name == "alignment_use_z_filter":
        # The input has already been selected by signed distance from the ground
        # plane. A second sensor-frame z_min/z_max/invert_z pass would undo it.
        return False
    return _ORIGINAL_BOOL_VALUE(context, name)


def _plane_height_node(*args, **kwargs):
    """Force symmetric filtered ICP inputs, including remote-only pipelines.

    The base launch only receives the topic returned by ``_robot_actions`` for a
    pipeline started in the current process. On the fusion laptop, the remote
    robot pipeline is disabled, so its fallback used to remain
    ``/rN/mapping/lidar`` while the local robot used the plane-filtered topic.
    That asymmetric preprocessing made startup ICP estimates unstable.
    """

    if (
        kwargs.get("package") == "co_3dto2d_mapping"
        and kwargs.get("executable") == "initial_xy_icp_alignment.py"
    ):
        rewritten_parameters = []
        for parameter_set in kwargs.get("parameters", []):
            if isinstance(parameter_set, dict):
                parameter_set = dict(parameter_set)
                parameter_set["robot0_cloud_topic"] = "/r0%s" % _FILTERED_SUFFIX
                parameter_set["robot1_cloud_topic"] = "/r1%s" % _FILTERED_SUFFIX
            rewritten_parameters.append(parameter_set)
        kwargs = dict(kwargs)
        kwargs["parameters"] = rewritten_parameters
    return _ORIGINAL_NODE(*args, **kwargs)


_BASE._robot_actions = _plane_filtered_robot_actions
_BASE._bool_value = _plane_height_bool_value
_BASE.Node = _plane_height_node


def generate_launch_description():
    return _BASE.generate_launch_description()
