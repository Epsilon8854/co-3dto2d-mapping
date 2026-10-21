from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[1]
REPLAY_RUNNER = REPOSITORY / "scripts" / "replay_experiment_02_occupancy_mapping.sh"
ICP_ON_PROFILE = (
    REPOSITORY
    / "config"
    / "experiments"
    / "experiment_02_occupancy_mapping"
    / "local_window_icp_on.yaml"
)


def test_experiment_02_replay_rear_filters_occupancy_but_not_icp():
    script = REPLAY_RUNNER.read_text()

    assert "rear_sector_filter_enabled: true" in script
    assert "enable_rear_lidar_filter:=true" in script
    assert "bag_lidar_topic:=/r0/livox/lidar" in script
    assert "scan_cloud_topic:=/r0/mapping/lidar_rear_filtered" in script
    assert "icp_scan_cloud_topic:=/r0/livox/lidar" in script


def test_experiment_02_replay_can_select_a_dedicated_local_window_icp_profile():
    """The baseline profile stays immutable while replay selects the ICP ablation."""
    script = REPLAY_RUNNER.read_text(encoding="utf-8")

    assert ICP_ON_PROFILE.is_file()
    assert "enable_local_window_icp: true" in ICP_ON_PROFILE.read_text(encoding="utf-8")
    assert "--local-window-icp off|on" in script
    assert 'local_window_icp="off"' in script
    assert '"${local_window_icp}" == "on"' in script
    assert 'local_window_icp_on.yaml' in script


def test_experiment_02_replay_selects_local_submap_registration_profile():
    profile = (
        REPOSITORY / "config" / "experiments" / "experiment_02_occupancy_mapping"
        / "local_submap_registration.yaml"
    )
    script = REPLAY_RUNNER.read_text(encoding="utf-8")

    assert profile.is_file()
    text = profile.read_text(encoding="utf-8")
    assert "local_submap_registration_enabled: true" in text
    assert "enable_local_window_icp: false" in text
    assert "--local-submap-registration off|on" in script
    assert "local_submap_registration.yaml" in script
    assert "local_submap_registration.csv" in script


def test_line_pose_graph_profile_uses_conservative_pht_minimum_length():
    profile = (
        REPOSITORY / "config" / "experiments" / "experiment_02_occupancy_mapping"
        / "local_line_pose_graph.yaml"
    )

    text = profile.read_text(encoding="utf-8")
    assert "local_line_pht_min_length_m: 1.50" in text


def test_live_runner_selects_queue_one_and_history_five_full_se2_profile():
    runner = REPOSITORY / "scripts" / "run_experiment_02_occupancy_mapping.sh"
    profile = (
        REPOSITORY / "config" / "experiments" / "experiment_02_occupancy_mapping"
        / "live_pht_history_5_full_se2.yaml"
    )
    text = runner.read_text(encoding="utf-8")

    assert profile.is_file()
    profile_text = profile.read_text(encoding="utf-8")
    assert "local_pht_submap_pose_graph_mode: full_se2" in profile_text
    assert "local_pht_submap_history_keyframes: 5" in profile_text
    assert "icp_topic_queue_size:=1" in text
    assert "live_pht_history_5_full_se2.yaml" in text


def test_live_rviz_uses_the_namespaced_global_occupancy_frame():
    config = REPOSITORY / "rviz" / "experiment_02_occupancy_mapping.rviz"
    text = config.read_text(encoding="utf-8")

    assert "Value: /r0/mapping/global_occupancy" in text
    assert "Fixed Frame: r0/odom" in text
