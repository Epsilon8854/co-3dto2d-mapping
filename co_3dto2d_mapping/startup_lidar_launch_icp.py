"""Launch helpers wiring startup LiDAR occupancy maps into planar ICP."""

from __future__ import annotations

import sys

from launch.actions import ExecuteProcess, GroupAction

from co_3dto2d_mapping.startup_lidar_launch_map import (
    STARTUP_MAP_TOPICS,
    startup_map_node,
)


def map_topic_instead_of_cloud(_context, robot_id: int) -> str:
    """Make inherited gate diagnostics refer to maps, not PointCloud2."""

    return STARTUP_MAP_TOPICS[robot_id]


def gate_process(layer, topic: str, name: str):
    """Retain gate behavior while naming its startup inputs generically."""

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
            "robot0_input_topic:=%s" % STARTUP_MAP_TOPICS[0],
            "-p",
            "robot1_input_topic:=%s" % STARTUP_MAP_TOPICS[1],
            "-p",
            "timeout_sec:=%.6f" % layer._ACTIVE_STARTUP_TIMEOUT_SEC,
            "-p",
            "status_period_sec:=%.6f" % layer._ACTIVE_STARTUP_STATUS_PERIOD_SEC,
        ],
        output="screen",
    )


def startup_alignment_relay(layer, context):
    return layer._ORIGINAL_NODE(
        package="co_3dto2d_mapping",
        executable="startup_alignment_relay.py",
        name="startup_alignment_relay",
        output="screen",
        parameters=[
            {
                "input_topic": layer._ACTIVE_STARTUP_TOPIC,
                "output_topic": layer._ORIGINAL_VALUE(context, "alignment_topic"),
                "lock_after_first": True,
                "use_sim_time": False,
            }
        ],
    )


def startup_2d_map_icp_node(layer, source_cloud_topic, context):
    """Replace cropped-XYZ startup ICP with LiDAR-map SE(2) ICP."""

    occupancy_file = layer._ORIGINAL_VALUE(context, "occupancy_config_file")
    occupancy = layer._load_occupancy_parameters(occupancy_file)
    actions = [
        startup_map_node(layer, source_cloud_topic, context, 0, occupancy),
        startup_map_node(layer, source_cloud_topic, context, 1, occupancy),
        layer._ORIGINAL_NODE(
            package="co_3dto2d_mapping",
            executable="initial_xy_icp_alignment.py",
            name="startup_initial_xy_icp_alignment",
            output="screen",
            parameters=[
                occupancy_file,
                {
                    "robot0_map_topic": STARTUP_MAP_TOPICS[0],
                    "robot1_map_topic": STARTUP_MAP_TOPICS[1],
                    "input_mode": "global_occupancy",
                    "alignment_topic": layer._ACTIVE_STARTUP_TOPIC,
                    "target_frame_id": layer._ORIGINAL_VALUE(
                        context, "common_frame_id"
                    ),
                    "source_frame_id": "r1/odom",
                    "local_frame_id": "livox_frame",
                    "transform_cloud_to_local_frame": False,
                    "frame_count": 1,
                    "invert_result": layer._parse_bool(
                        layer._ORIGINAL_VALUE(context, "alignment_invert_result"),
                        "alignment_invert_result",
                    ),
                    # All filtering is already applied while constructing maps.
                    "center_box_half_extent_m": 0.0,
                    "voxel_size": float(
                        layer._ORIGINAL_VALUE(context, "alignment_voxel_size")
                    ),
                    "max_points": int(
                        layer._ORIGINAL_VALUE(context, "alignment_max_points")
                    ),
                    "max_correspondence_distance": float(
                        layer._ORIGINAL_VALUE(
                            context, "alignment_max_correspondence_distance"
                        )
                    ),
                    "min_correspondences": int(
                        layer._ORIGINAL_VALUE(
                            context, "alignment_min_correspondences"
                        )
                    ),
                    "min_fitness": float(
                        layer._ORIGINAL_VALUE(context, "alignment_min_fitness")
                    ),
                    "max_rmse": float(
                        layer._ORIGINAL_VALUE(context, "alignment_max_rmse")
                    ),
                    "max_iterations": int(
                        layer._ORIGINAL_VALUE(context, "alignment_max_iterations")
                    ),
                    "recompute_period_sec": float(
                        layer._ORIGINAL_VALUE(
                            context, "alignment_recompute_period_sec"
                        )
                    ),
                    "occupied_threshold": int(
                        layer._ORIGINAL_VALUE(
                            context, "alignment_occupied_threshold"
                        )
                    ),
                    # Settle delay and frame accumulation belong to map builders;
                    # init_xy now waits only for OccupancyGrid messages.
                    "startup_delay_sec": 0.0,
                    "retry_on_failure": True,
                    "lock_after_first_alignment": True,
                    "required_consistent_results": (
                        layer._ACTIVE_STARTUP_REQUIRED_RESULTS
                    ),
                    "max_consistency_translation_m": float(
                        layer._ORIGINAL_VALUE(
                            context, "alignment_max_consistency_translation_m"
                        )
                    ),
                    "max_consistency_rotation_rad": float(
                        layer._ORIGINAL_VALUE(
                            context, "alignment_max_consistency_rotation_rad"
                        )
                    ),
                    "initialize_from_centroids": layer._parse_bool(
                        layer._ORIGINAL_VALUE(
                            context, "alignment_initialize_from_centroids"
                        ),
                        "alignment_initialize_from_centroids",
                    ),
                    "enforce_tilt_prior": False,
                    "publish_period_sec": 0.2,
                    "use_sim_time": False,
                },
            ],
        ),
    ]

    place_recognition = layer._parse_bool(
        layer._ORIGINAL_VALUE(context, "enable_place_recognition"),
        "enable_place_recognition",
    )
    final_topic = layer._ORIGINAL_VALUE(context, "alignment_topic")
    if not place_recognition and layer._ACTIVE_STARTUP_TOPIC != final_topic:
        actions.append(startup_alignment_relay(layer, context))
    return GroupAction(actions=actions)
