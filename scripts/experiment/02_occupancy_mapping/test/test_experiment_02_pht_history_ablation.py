from pathlib import Path


REPOSITORY = next(p for p in Path(__file__).resolve().parents if (p/'package.xml').is_file() and (p/'CMakeLists.txt').is_file())
REPLAY_RUNNER = REPOSITORY / "scripts/experiment/02_occupancy_mapping" / "replay_experiment_02_occupancy_mapping.sh"
ABLATION_RUNNER = REPOSITORY / "scripts/experiment/02_occupancy_mapping" / "run_experiment_02_pht_history_ablation.sh"
TOPDOWN_EXPORTER = REPOSITORY / "scripts/experiment/02_occupancy_mapping" / "export_experiment_02_topdown.py"


def test_replay_runner_accepts_and_records_pht_history_override():
    text = REPLAY_RUNNER.read_text(encoding="utf-8")

    assert "--pht-history-keyframes COUNT" in text
    assert 'pht_history_keyframes="10"' in text
    assert "local_pht_submap_history_keyframes:=" in text
    assert "pht_history_keyframes}" in text
    assert "pht_history_keyframes: ${pht_history_keyframes}" in text


def test_topdown_exporter_captures_global_map_raw_and_corrected_trajectories():
    assert TOPDOWN_EXPORTER.is_file()
    text = TOPDOWN_EXPORTER.read_text(encoding="utf-8")

    assert "/r0/mapping/global_occupancy" in text
    assert "/r0/mapping/debug/raw_odometry_path" in text
    assert "/r0/odom" in text
    assert "/r0/mapping/occupancy_mapper_odometry" in text
    assert "icp_odometry_xy.csv" in text
    assert "topdown_global_map_raw_cyan_corrected_orange.png" in text


def test_history_ablation_runs_requested_hallway_conditions():
    assert ABLATION_RUNNER.is_file()
    text = ABLATION_RUNNER.read_text(encoding="utf-8")

    assert "--dataset hallway" in text
    assert "--pht-profile length_0_7" in text
    assert "--local-pht-submap-pose-graph full_se2" in text
    assert "for history in 1 3 5 10; do" in text
    assert "--pht-history-keyframes \"${history}\"" in text


def test_topdown_exporter_does_not_log_through_a_shutdown_ros_context():
    text = TOPDOWN_EXPORTER.read_text(encoding="utf-8")

    assert 'self.get_logger().info("Saved top-down overlay:' not in text
    assert 'print("Saved top-down overlay:' in text


def test_replay_runner_exposes_icp_scan_queue_size_and_exports_raw_trajectory_csv():
    replay = REPLAY_RUNNER.read_text(encoding="utf-8")
    exporter = TOPDOWN_EXPORTER.read_text(encoding="utf-8")

    assert "--icp-topic-queue-size COUNT" in replay
    assert 'icp_topic_queue_size="1"' in replay
    assert "icp_topic_queue_size:=${icp_topic_queue_size}" in replay
    assert "raw_odometry_xy.csv" in exporter


def test_icp_reproducibility_runner_repeats_a_fixed_raw_baseline():
    runner = REPOSITORY / "scripts/experiment/02_occupancy_mapping" / "run_experiment_02_icp_reproducibility.sh"
    assert runner.is_file()
    text = runner.read_text(encoding="utf-8")

    assert "--local-pht-submap-pose-graph off" in text
    assert "--rate RATE" in text
    assert 'replay_rate="1.0"' in text
    assert "--rate \"${replay_rate}\"" in text
    assert "--icp-topic-queue-size" in text
    assert "raw_odometry_xy.csv" in text
    assert "icp_odometry_xy.csv" in text
    assert "update_time" in text


def test_replay_runner_maps_pht_off_to_a_valid_disabled_mode():
    text = REPLAY_RUNNER.read_text(encoding="utf-8")

    assert 'pht_mode_for_launch="visualization_only"' in text
    assert 'if [[ "${local_pht_submap_pose_graph}" != "off" ]]; then' in text
    assert 'local_pht_submap_pose_graph_mode:="${pht_mode_for_launch}"' in text


def test_replay_runner_can_record_and_replay_canonical_mapper_inputs():
    text = REPLAY_RUNNER.read_text(encoding="utf-8")

    assert "--record-canonical-mapper-input-bag DIR" in text
    assert "--canonical-mapper-input" in text
    assert "/r0/mapping/plane_height_filtered" in text
    assert "/r0/mapping/floor_based_odometry" in text
    assert "canonical_mapper_input_mapping.launch.py" in text


def test_history_ablation_accepts_a_canonical_mapper_input_bag():
    text = ABLATION_RUNNER.read_text(encoding="utf-8")

    assert "--canonical-mapper-input-bag DIR" in text
    assert "--canonical-mapper-input" in text
