"""Public two-live wrapper with startup ICP gating and occupancy place recognition.

The raw live clouds are available before RTAB-Map odometry starts. This wrapper
uses those clouds for a one-shot cropped XYZ startup ICP and launches each local
odometry/mapping pipeline only after an accepted startup transform is observed.
The later CPU-only occupancy place-recognition node remains the final inter-robot
map alignment stage.

The physical live runner can make startup ICP consume each driver's LiDAR
directly, assuming the two robots use the same sensor extrinsic. This avoids
waiting for cross-host sensor TF while retaining the occupancy range/body crop.
Plane-height filtering remains in the post-odometry mapping pipeline because it
is synchronized with odometry by design.
"""

import importlib.util
import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


_FILTERED_SUFFIX = "/mapping/plane_height_filtered"
_TRUE_VALUES = {"1", "true", "yes", "on"}
_FALSE_VALUES = {"0", "false", "no", "off"}


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
_ORIGINAL_VALUE = _BASE._value
_ORIGINAL_NODE = _BASE.Node
_ORIGINAL_LAUNCH_SETUP = _BASE.launch_setup

_ACTIVE_STARTUP_GATE = False
_ACTIVE_STARTUP_TOPIC = "/toy/startup_xy_alignment"
_ACTIVE_STARTUP_TIMEOUT_SEC = 60.0
_ACTIVE_STARTUP_STATUS_PERIOD_SEC = 2.0
_ACTIVE_STARTUP_REQUIRED_RESULTS = 1
_ACTIVE_STARTUP_CLOUD_TOPICS = (
    "/r0/mapping/lidar",
    "/r1/mapping/lidar",
)
_active_alignment_config_file = ""


def _parse_bool(value, name):
    normalized = str(value).strip().lower()
    if normalized in _TRUE_VALUES:
        return True
    if normalized in _FALSE_VALUES:
        return False
    raise RuntimeError("%s must be a boolean value, got %r" % (name, value))


def _startup_gate_enabled(context):
    return _parse_bool(
        LaunchConfiguration("wait_for_initial_alignment").perform(context),
        "wait_for_initial_alignment",
    )


def _startup_topic(context):
    topic = LaunchConfiguration("startup_alignment_topic").perform(context).strip()
    if not topic.startswith("/"):
        raise RuntimeError(
            "startup_alignment_topic must be absolute (start with '/'), got %r"
            % topic
        )
    return topic


def _positive_float_argument(context, name):
    try:
        value = float(LaunchConfiguration(name).perform(context))
    except ValueError as exc:
        raise RuntimeError("%s must be numeric" % name) from exc
    if value <= 0.0:
        raise RuntimeError("%s must be greater than zero" % name)
    return value


def _positive_int_argument(context, name):
    try:
        value = int(LaunchConfiguration(name).perform(context))
    except ValueError as exc:
        raise RuntimeError("%s must be an integer" % name) from exc
    if value < 1:
        raise RuntimeError("%s must be at least one" % name)
    return value


def _startup_value(context, name):
    if _startup_gate_enabled(context):
        # The accepted alignment is the startup barrier. Disable the old fixed
        # 10 s timer so the local pipeline starts immediately after release.
        if name == "mapping_startup_delay_sec":
            return "0.0"
        # The static sensor TF must be available before startup ICP while the
        # rest of the local mapping pipeline is still gated.
        if name == "publish_sensor_static_tf":
            return "false"
    return _ORIGINAL_VALUE(context, name)


def _plane_height_bool_value(context, name):
    if name == "alignment_use_z_filter":
        return False
    return _ORIGINAL_BOOL_VALUE(context, name)


def _gate_process(topic, name):
    robot0_cloud_topic, robot1_cloud_topic = _ACTIVE_STARTUP_CLOUD_TOPICS
    return ExecuteProcess(
        cmd=[
            sys.executable,
            "-m",
            "co_3dto2d_mapping.alignment_startup_gate",
            "--ros-args",
            "-r",
            "__node:=%s" % name,
            "-p",
            "alignment_topic:=%s" % topic,
            "-p",
            "robot0_cloud_topic:=%s" % robot0_cloud_topic,
            "-p",
            "robot1_cloud_topic:=%s" % robot1_cloud_topic,
            "-p",
            "timeout_sec:=%.6f" % _ACTIVE_STARTUP_TIMEOUT_SEC,
            "-p",
            "status_period_sec:=%.6f" % _ACTIVE_STARTUP_STATUS_PERIOD_SEC,
        ],
        output="screen",
    )


def _sensor_static_tf_action(context, robot_id):
    return _ORIGINAL_NODE(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="startup_sensor_static_tf_r%d" % robot_id,
        output="screen",
        arguments=[
            _ORIGINAL_VALUE(context, "sensor_tf_x_%d" % robot_id),
            _ORIGINAL_VALUE(context, "sensor_tf_y_%d" % robot_id),
            _ORIGINAL_VALUE(context, "sensor_tf_z_%d" % robot_id),
            _ORIGINAL_VALUE(context, "sensor_tf_yaw_%d" % robot_id),
            _ORIGINAL_VALUE(context, "sensor_tf_pitch_%d" % robot_id),
            _ORIGINAL_VALUE(context, "sensor_tf_roll_%d" % robot_id),
            "r%d/base_link" % robot_id,
            "r%d/livox_frame" % robot_id,
        ],
    )


def _gate_exit_actions(event, success_message, released_actions):
    return_code = int(event.returncode)
    if return_code != 0:
        reason = (
            "startup alignment gate failed with exit code %d; odometry/mapping "
            "was not started" % return_code
        )
        return [
            LogInfo(msg=reason),
            EmitEvent(event=Shutdown(reason=reason)),
        ]
    return [LogInfo(msg=success_message), *released_actions]


def _release_group(gate, waiting_message, success_message, released_actions):
    # Register the exit handler before starting the gate process. The previous
    # order could miss a fast exit when a transient-local sample was already
    # available, leaving the launch in a permanent wait state.
    handler = RegisterEventHandler(
        OnProcessExit(
            target_action=gate,
            on_exit=lambda event, _context: _gate_exit_actions(
                event,
                success_message,
                released_actions,
            ),
        )
    )
    return GroupAction(
        actions=[
            LogInfo(msg=waiting_message),
            handler,
            gate,
        ]
    )


def _gated_pipeline(context, robot_id, pipeline):
    startup_topic = _startup_topic(context)
    gate = _gate_process(
        startup_topic,
        "startup_alignment_gate_r%d" % robot_id,
    )
    actions = []
    publish_static_tf = _parse_bool(
        _ORIGINAL_VALUE(context, "publish_sensor_static_tf"),
        "publish_sensor_static_tf",
    )
    if publish_static_tf:
        actions.append(_sensor_static_tf_action(context, robot_id))
    actions.append(
        _release_group(
            gate,
            (
                "[r%d] waiting for startup ICP alignment on %s; "
                "odometry/mapping has not started."
                % (robot_id, startup_topic)
            ),
            (
                "[r%d] startup ICP alignment received; starting "
                "odometry/mapping now." % robot_id
            ),
            [pipeline],
        )
    )
    return GroupAction(actions=actions)


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
    if _startup_gate_enabled(context):
        pipeline = _gated_pipeline(context, robot_id, pipeline)
    filtered_topic = "/r%d%s" % (robot_id, _FILTERED_SUFFIX)
    return republisher, pipeline, filtered_topic


def _place_recognition_parameters(forwarded):
    package_share = get_package_share_directory("co_3dto2d_mapping")
    place_config = os.path.join(package_share, "config", "place_recognition.yaml")
    alignment_parameters = [place_config]
    if _active_alignment_config_file:
        alignment_parameters.append(_active_alignment_config_file)
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
    return alignment_parameters, overrides


def _place_recognition_node(*args, **kwargs):
    """Rewrite final alignment and gate fusion consumers behind startup ICP."""

    is_alignment_node = (
        kwargs.get("package") == "co_3dto2d_mapping"
        and kwargs.get("executable") == "initial_xy_icp_alignment.py"
    )
    is_record_node = (
        kwargs.get("package") == "co_3dto2d_mapping"
        and kwargs.get("executable") == "record_republisher.py"
    )

    if is_alignment_node:
        forwarded = {}
        for parameter_set in kwargs.get("parameters", []):
            if isinstance(parameter_set, dict):
                forwarded.update(parameter_set)

        alignment_parameters, overrides = _place_recognition_parameters(forwarded)
        rewritten = dict(kwargs)
        rewritten["executable"] = "inter_robot_place_alignment.py"
        rewritten["name"] = "inter_robot_place_alignment"
        rewritten["parameters"] = [*alignment_parameters, overrides]
        node = _ORIGINAL_NODE(*args, **rewritten)
        gate_name = "startup_alignment_gate_place_recognition"
        waiting = (
            "Place recognition is waiting for startup ICP alignment on %s."
            % _ACTIVE_STARTUP_TOPIC
        )
        success = "Startup ICP is complete; starting occupancy place recognition."
    elif is_record_node:
        node = _ORIGINAL_NODE(*args, **kwargs)
        gate_name = "startup_alignment_gate_record_republisher"
        waiting = (
            "Record/merged-map publishing is waiting for startup ICP alignment "
            "on %s." % _ACTIVE_STARTUP_TOPIC
        )
        success = (
            "Startup ICP is complete; starting record and merged-map publishing."
        )
    else:
        return _ORIGINAL_NODE(*args, **kwargs)

    if not _ACTIVE_STARTUP_GATE:
        return node

    gate = _gate_process(_ACTIVE_STARTUP_TOPIC, gate_name)
    return _release_group(gate, waiting, success, [node])


def _load_occupancy_parameters(path):
    try:
        with open(path, "r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError("could not read occupancy config %s: %s" % (path, exc))

    wildcard = document.get("/**", {})
    parameters = wildcard.get("ros__parameters", {}) if isinstance(wildcard, dict) else {}
    if not isinstance(parameters, dict):
        raise RuntimeError(
            "occupancy config %s must contain /** -> ros__parameters" % path
        )
    return parameters


def _occupancy_float(parameters, name, default):
    try:
        return float(parameters.get(name, default))
    except (TypeError, ValueError) as exc:
        raise RuntimeError("occupancy parameter %s must be numeric" % name) from exc


def _startup_uses_direct_lidar():
    return _parse_bool(
        os.environ.get("CO3DTO2D_STARTUP_DIRECT_LIDAR", "false"),
        "CO3DTO2D_STARTUP_DIRECT_LIDAR",
    )


def _prealignment_cloud_topic(context, robot_id):
    if _startup_uses_direct_lidar():
        return _ORIGINAL_VALUE(context, "robot%d_lidar_topic" % robot_id)
    rear_filter = _parse_bool(
        _ORIGINAL_VALUE(context, "enable_rear_lidar_filter"),
        "enable_rear_lidar_filter",
    )
    suffix = "/mapping/lidar_unfiltered" if rear_filter else "/mapping/lidar"
    return "/r%d%s" % (robot_id, suffix)


def _startup_icp_node(context):
    occupancy_config_file = _ORIGINAL_VALUE(context, "occupancy_config_file")
    occupancy = _load_occupancy_parameters(occupancy_config_file)
    startup_topic = _startup_topic(context)

    transform_to_local = _parse_bool(
        occupancy.get("transform_cloud_to_local_frame", True),
        "transform_cloud_to_local_frame",
    ) and _parse_bool(
        _ORIGINAL_VALUE(context, "transform_cloud_to_local_frame"),
        "transform_cloud_to_local_frame",
    )

    # Startup ICP runs before odometry. The physical runner selects driver
    # streams directly so the remote robot only needs to deliver LiDAR DDS data.
    # Matching MID-360 extrinsics make those coordinates a valid local basis.
    overrides = {
        "robot0_cloud_topic": _prealignment_cloud_topic(context, 0),
        "robot1_cloud_topic": _prealignment_cloud_topic(context, 1),
        "robot0_map_topic": "/r0/toy/global_occupancy",
        "robot1_map_topic": "/r1/toy/global_occupancy",
        "input_mode": "cloud_initial",
        "alignment_topic": startup_topic,
        "target_frame_id": _ORIGINAL_VALUE(context, "common_frame_id"),
        "source_frame_id": "r1/odom",
        "local_frame_id": "base_link",
        "robot0_local_frame_id": "r0/base_link",
        "robot1_local_frame_id": "r1/base_link",
        "transform_cloud_to_local_frame": (
            transform_to_local and not _startup_uses_direct_lidar()
        ),
        "use_z_filter": False,
        "slice_z_in_cloud_frame": _parse_bool(
            occupancy.get("slice_z_in_cloud_frame", True),
            "slice_z_in_cloud_frame",
        ),
        "z_min": _occupancy_float(occupancy, "z_min", -1000.0),
        "z_max": _occupancy_float(occupancy, "z_max", 1000.0),
        "invert_z_slice": False,
        "frame_count": int(_ORIGINAL_VALUE(context, "alignment_frame_count")),
        "invert_result": _parse_bool(
            _ORIGINAL_VALUE(context, "alignment_invert_result"),
            "alignment_invert_result",
        ),
        "center_box_half_extent_m": _occupancy_float(
            occupancy, "center_box_filter_half_extent_m", 0.80
        ),
        "range_min_m": _occupancy_float(occupancy, "range_min_m", 0.80),
        "range_max_m": _occupancy_float(occupancy, "range_max_m", 12.0),
        "voxel_size": float(_ORIGINAL_VALUE(context, "alignment_voxel_size")),
        "max_points": int(_ORIGINAL_VALUE(context, "alignment_max_points")),
        "max_correspondence_distance": float(
            _ORIGINAL_VALUE(context, "alignment_max_correspondence_distance")
        ),
        "min_correspondences": int(
            _ORIGINAL_VALUE(context, "alignment_min_correspondences")
        ),
        "min_fitness": float(_ORIGINAL_VALUE(context, "alignment_min_fitness")),
        "max_rmse": float(_ORIGINAL_VALUE(context, "alignment_max_rmse")),
        "max_iterations": int(_ORIGINAL_VALUE(context, "alignment_max_iterations")),
        "recompute_period_sec": float(
            _ORIGINAL_VALUE(context, "alignment_recompute_period_sec")
        ),
        "startup_delay_sec": float(
            _ORIGINAL_VALUE(context, "alignment_startup_delay_sec")
        ),
        "retry_on_failure": True,
        "lock_after_first_alignment": True,
        # Startup alignment is only a release barrier. The later occupancy place
        # recognizer already performs multi-measurement consensus, so requiring
        # two independently stable startup ICPs can deadlock on small scan noise.
        "required_consistent_results": _ACTIVE_STARTUP_REQUIRED_RESULTS,
        "max_consistency_translation_m": float(
            _ORIGINAL_VALUE(context, "alignment_max_consistency_translation_m")
        ),
        "max_consistency_rotation_rad": float(
            _ORIGINAL_VALUE(context, "alignment_max_consistency_rotation_rad")
        ),
        "initialize_from_centroids": _parse_bool(
            _ORIGINAL_VALUE(context, "alignment_initialize_from_centroids"),
            "alignment_initialize_from_centroids",
        ),
        "enforce_tilt_prior": _parse_bool(
            _ORIGINAL_VALUE(context, "alignment_enforce_tilt_prior"),
            "alignment_enforce_tilt_prior",
        ),
        "max_tilt_deviation_rad": float(
            _ORIGINAL_VALUE(context, "alignment_max_tilt_deviation_rad")
        ),
        "publish_period_sec": 0.2,
        "use_sim_time": False,
    }
    return _ORIGINAL_NODE(
        package="co_3dto2d_mapping",
        executable="initial_xy_icp_alignment.py",
        name="startup_initial_xy_icp_alignment",
        output="screen",
        parameters=[occupancy_config_file, overrides],
    )


def _startup_launch_setup(context, *args, **kwargs):
    global _ACTIVE_STARTUP_GATE
    global _ACTIVE_STARTUP_TOPIC
    global _ACTIVE_STARTUP_TIMEOUT_SEC
    global _ACTIVE_STARTUP_STATUS_PERIOD_SEC
    global _ACTIVE_STARTUP_REQUIRED_RESULTS
    global _ACTIVE_STARTUP_CLOUD_TOPICS
    global _active_alignment_config_file

    _ACTIVE_STARTUP_GATE = _startup_gate_enabled(context)
    _ACTIVE_STARTUP_TOPIC = _startup_topic(context)
    _active_alignment_config_file = _ORIGINAL_VALUE(
        context, "alignment_config_file"
    ).strip()
    if _ACTIVE_STARTUP_GATE:
        _ACTIVE_STARTUP_TIMEOUT_SEC = _positive_float_argument(
            context, "startup_alignment_timeout_sec"
        )
        _ACTIVE_STARTUP_STATUS_PERIOD_SEC = _positive_float_argument(
            context, "startup_alignment_status_period_sec"
        )
        _ACTIVE_STARTUP_REQUIRED_RESULTS = _positive_int_argument(
            context, "startup_alignment_required_consistent_results"
        )
        _ACTIVE_STARTUP_CLOUD_TOPICS = (
            _prealignment_cloud_topic(context, 0),
            _prealignment_cloud_topic(context, 1),
        )

    actions = _ORIGINAL_LAUNCH_SETUP(context, *args, **kwargs)
    enable_fusion = _parse_bool(
        _ORIGINAL_VALUE(context, "enable_fusion"),
        "enable_fusion",
    )
    if _ACTIVE_STARTUP_GATE and enable_fusion:
        actions.insert(0, _startup_icp_node(context))
        actions.insert(
            1,
            LogInfo(
                msg=(
                    "Startup ICP is active on %s using clouds=(%s, %s), "
                    "timeout=%.1fs, accepted_results=%d. Legacy fixed/inverted "
                    "Z slicing is disabled; occupancy.yaml supplies the body/range "
                    "crop."
                    % (
                        _ACTIVE_STARTUP_TOPIC,
                        *_ACTIVE_STARTUP_CLOUD_TOPICS,
                        _ACTIVE_STARTUP_TIMEOUT_SEC,
                        _ACTIVE_STARTUP_REQUIRED_RESULTS,
                    )
                )
            ),
        )
    return actions


_BASE._value = _startup_value
_BASE._robot_actions = _plane_filtered_robot_actions
_BASE._bool_value = _plane_height_bool_value
_BASE.Node = _place_recognition_node
_BASE.launch_setup = _startup_launch_setup


def generate_launch_description():
    base_description = _BASE.generate_launch_description()
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "wait_for_initial_alignment",
                default_value="true",
                description=(
                    "Wait for one accepted startup cloud ICP before launching "
                    "each robot's odometry/mapping pipeline."
                ),
            ),
            DeclareLaunchArgument(
                "startup_alignment_topic",
                default_value="/toy/startup_xy_alignment",
                description=(
                    "Transient-local startup ICP topic used only as the "
                    "odometry/mapping release gate."
                ),
            ),
            DeclareLaunchArgument(
                "startup_alignment_timeout_sec",
                default_value="60.0",
                description=(
                    "Fail the launch instead of waiting forever when no accepted "
                    "startup alignment arrives."
                ),
            ),
            DeclareLaunchArgument(
                "startup_alignment_status_period_sec",
                default_value="2.0",
                description="Periodic diagnostic interval while the gate is waiting.",
            ),
            DeclareLaunchArgument(
                "startup_alignment_required_consistent_results",
                default_value="1",
                description=(
                    "Number of quality-qualified startup ICP results required "
                    "before releasing odometry."
                ),
            ),
            *list(base_description.entities),
        ]
    )
