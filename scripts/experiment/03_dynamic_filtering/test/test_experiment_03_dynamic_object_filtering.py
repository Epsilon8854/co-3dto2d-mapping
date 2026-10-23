from pathlib import Path
import re

import yaml


REPOSITORY = next(p for p in Path(__file__).resolve().parents if (p/'package.xml').is_file() and (p/'CMakeLists.txt').is_file())
EXPERIMENT_DIRECTORY = (
    REPOSITORY / "config" / "experiment" / "experiment_03_dynamic_object_filtering"
)
REPLAY_RUNNER = REPOSITORY / "scripts/experiment/03_dynamic_filtering" / "replay_experiment_03_dynamic_object_filtering.sh"
RVIZ_CONFIG = REPOSITORY / "rviz/experiment" / "experiment_03_dynamic_object_filtering_replay.rviz"


def _parameters(name: str):
    return yaml.safe_load((EXPERIMENT_DIRECTORY / name).read_text())["/**"][
        "ros__parameters"
    ]


def test_dynamic_profiles_change_only_the_temporal_filter_switch():
    disabled = _parameters("dynamic_filter_off.yaml")
    enabled = _parameters("dynamic_filter_on.yaml")

    assert disabled["dynamic_filter_enabled"] is False
    assert enabled["dynamic_filter_enabled"] is True
    assert {
        key: value for key, value in disabled.items() if key != "dynamic_filter_enabled"
    } == {
        key: value for key, value in enabled.items() if key != "dynamic_filter_enabled"
    }
    assert enabled["local_pht_submap_pose_graph_mode"] == "full_se2"
    assert enabled["local_pht_submap_history_keyframes"] == 5
    assert enabled["dynamic_free_clear_count"] == 30
    assert enabled["dynamic_evidence_timeout_sec"] == 5.0
    assert enabled["dynamic_clear_max_range_m"] == 6.0
    assert "dynamic_occupied_confirm_count" not in enabled
    assert "dynamic_counter_decay" not in enabled
    assert "dynamic_evidence_timeout_frames" not in enabled


def test_dynamic_replay_runner_selects_on_off_profiles_and_exports_topdown_png():
    script = REPLAY_RUNNER.read_text(encoding="utf-8")

    assert "--dynamic-filter off|on" in script
    assert "dynamic_filter_off.yaml" in script
    assert "dynamic_filter_on.yaml" in script
    assert "single_bag_mapping.launch.py" in script
    assert "export_experiment_02_topdown.py" in script
    assert "topdown_global_map_raw_cyan_corrected_orange.png" in script
    assert 'set +u\n# shellcheck disable=SC1091\nsource "${ROS_SETUP:-/opt/ros/humble/setup.bash}"' in script
    assert "--loop                     Repeat forever and restart mapping per pass" in script
    assert 'loop_playback="false"' in script
    assert 'if [[ "${loop_playback}" != "true" ]]' in script
    assert 'stop_process "${launch_pid}" TERM' in script


def test_experiment_03_rviz_uses_the_namespaced_map_frame_and_core_topics():
    text = RVIZ_CONFIG.read_text(encoding="utf-8")

    assert "Fixed Frame: r0/odom" in text
    for topic in (
        "/r0/mapping/global_occupancy",
        "/r0/mapping/plane_height_filtered",
        "/r0/mapping/slice_kept_points",
        "/r0/mapping/debug/raw_odometry_path",
        "/r0/mapping/debug/floor_based_odometry_path",
        "/r0/mapping/debug/local_submap_corrected_odometry_path",
        "/r0/odom",
        "/r0/mapping/floor_based_odometry",
        "/r0/mapping/occupancy_mapper_odometry",
    ):
        assert f"Value: {topic}" in text

    for display_name in (
        "Captured LiDAR Input (before gravity-height filter, gray)",
        "Rear + Height Filtered Cloud (green, mapper source)",
        "Raw RTAB-Map Odometry Path (red)",
        "Raw Odometry Axes (red)",
    ):
        assert re.search(
            rf"Enabled: true.*?Name: {re.escape(display_name)}", text, re.DOTALL
        )
