"""Public two-live wrapper with config-driven initial XYZ alignment.

The base launch owns both robot pipelines and the common-frame outputs.  This
wrapper keeps the public installed launch name while ensuring the startup
``initial_xy_icp_alignment`` reads its geometric/ICP parameters from the same
occupancy YAML as the mappers.  Legacy inline z/invert/range defaults are removed
from the node override list, so the dedicated YAML section is authoritative.
"""

from __future__ import annotations

import importlib.util
import os
from typing import Optional

from ament_index_python.packages import get_package_share_directory


# These parameters are algorithm/config values and must come from the selected
# occupancy YAML.  Topic/frame/startup values remain explicit runtime overrides.
_ALIGNMENT_CONFIG_PARAMETERS = {
    "use_z_filter",
    "slice_z_in_cloud_frame",
    "z_min",
    "z_max",
    "invert_z_slice",
    "frame_count",
    "invert_result",
    "center_box_half_extent_m",
    "range_min_m",
    "range_max_m",
    "voxel_size",
    "max_points",
    "max_correspondence_distance",
    "min_correspondences",
    "min_fitness",
    "max_rmse",
    "max_iterations",
    "occupied_threshold",
    "required_consistent_results",
    "max_consistency_translation_m",
    "max_consistency_rotation_rad",
    "initialize_from_centroids",
    "enforce_heading_prior",
    "expected_yaw_rad",
    "max_yaw_deviation_rad",
    "initial_yaw_offsets_rad",
    "max_iteration_rotation_rad",
    "heading_prior_weight",
    "enforce_tilt_prior",
    "max_tilt_deviation_rad",
}


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
_ORIGINAL_LAUNCH_SETUP = _BASE.launch_setup
_ORIGINAL_NODE = _BASE.Node
_ORIGINAL_DECLARE_LAUNCH_ARGUMENT = _BASE.DeclareLaunchArgument
_ACTIVE_OCCUPANCY_CONFIG: Optional[str] = None


_PUBLIC_ALIGNMENT_ARGUMENT_DEFAULTS = {
    "alignment_use_z_filter": "false",
    "alignment_z_min": "-1000.0",
    "alignment_z_max": "1000.0",
    "alignment_invert_z_slice": "false",
}


def _configured_launch_argument(name, *args, **kwargs):
    """Expose non-inverted public defaults even though the base is retained."""

    if name in _PUBLIC_ALIGNMENT_ARGUMENT_DEFAULTS:
        replacement = _PUBLIC_ALIGNMENT_ARGUMENT_DEFAULTS[name]
        positional = list(args)
        if positional:
            positional[0] = replacement
        else:
            kwargs["default_value"] = replacement
        args = tuple(positional)
    return _ORIGINAL_DECLARE_LAUNCH_ARGUMENT(name, *args, **kwargs)


def _config_aware_launch_setup(context, *args, **kwargs):
    global _ACTIVE_OCCUPANCY_CONFIG
    _ACTIVE_OCCUPANCY_CONFIG = _BASE._value(context, "occupancy_config_file")
    return _ORIGINAL_LAUNCH_SETUP(context, *args, **kwargs)


def _configured_initial_alignment_node(*args, **kwargs):
    if not (
        kwargs.get("package") == "co_3dto2d_mapping"
        and kwargs.get("executable") == "initial_xy_icp_alignment.py"
    ):
        return _ORIGINAL_NODE(*args, **kwargs)

    config_file = _ACTIVE_OCCUPANCY_CONFIG
    if not config_file:
        config_file = os.path.join(
            get_package_share_directory("co_3dto2d_mapping"),
            "config",
            "occupancy.yaml",
        )

    runtime_overrides = {}
    for parameter_set in kwargs.get("parameters", []):
        if isinstance(parameter_set, dict):
            runtime_overrides.update(
                {
                    name: value
                    for name, value in parameter_set.items()
                    if name not in _ALIGNMENT_CONFIG_PARAMETERS
                }
            )

    rewritten = dict(kwargs)
    rewritten["parameters"] = [config_file, runtime_overrides]
    return _ORIGINAL_NODE(*args, **rewritten)


_BASE.launch_setup = _config_aware_launch_setup
_BASE.Node = _configured_initial_alignment_node
_BASE.DeclareLaunchArgument = _configured_launch_argument


def generate_launch_description():
    return _BASE.generate_launch_description()
