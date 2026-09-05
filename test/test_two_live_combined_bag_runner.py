from __future__ import annotations

from pathlib import Path
import subprocess


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
RUNNER = REPOSITORY_ROOT / "scripts" / "run_two_live_combined_bag.sh"


def _write_metadata(bag_path: Path, topics: dict[str, tuple[str, int]]) -> None:
    bag_path.mkdir()
    topic_entries = "\n".join(
        "    - topic_metadata:\n"
        f"        name: {topic}\n"
        f"        type: {message_type}\n"
        f"      message_count: {message_count}"
        for topic, (message_type, message_count) in topics.items()
    )
    (bag_path / "metadata.yaml").write_text(
        "rosbag2_bagfile_information:\n"
        "  storage_identifier: sqlite3\n"
        "  duration:\n"
        "    nanoseconds: 63704000000\n"
        "  topics_with_message_count:\n"
        + topic_entries
        + "\n",
        encoding="utf-8",
    )


def _dry_run(bag: Path, *extra_args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            "bash",
            str(RUNNER),
            "--dry-run",
            "--bag",
            str(bag),
            *extra_args,
        ],
        check=False,
        capture_output=True,
        text=True,
    )


def _representative_topics() -> dict[str, tuple[str, int]]:
    return {
        "/r0/livox/lidar": ("sensor_msgs/msg/PointCloud2", 106),
        "/r0/livox/imu": ("sensor_msgs/msg/Imu", 51),
        "/r0/mapping/imu_filtered": ("sensor_msgs/msg/Imu", 2088),
        "/r1/livox/lidar": ("sensor_msgs/msg/PointCloud2", 190),
        "/r1/livox/imu": ("sensor_msgs/msg/Imu", 660),
        "/r1/mapping/imu_filtered": ("sensor_msgs/msg/Imu", 3053),
        "/r0/odom": ("nav_msgs/msg/Odometry", 5),
        "/r1/odom": ("nav_msgs/msg/Odometry", 54),
        "/toy/initial_xy_alignment": (
            "geometry_msgs/msg/TransformStamped",
            66,
        ),
        "/tf": ("tf2_msgs/msg/TFMessage", 130),
        "/tf_static": ("tf2_msgs/msg/TFMessage", 2),
    }


def test_combined_replay_uses_one_player_and_excludes_recorded_outputs(
    tmp_path: Path,
) -> None:
    bag = tmp_path / "combined"
    _write_metadata(bag, _representative_topics())

    result = _dry_run(bag)

    assert result.returncode == 0, result.stderr
    player_lines = [
        line for line in result.stdout.splitlines() if line.startswith("BAG_PLAYER=")
    ]
    assert len(player_lines) == 1
    player = player_lines[0]
    assert "--topics /r0/livox/lidar /r0/mapping/imu_filtered" in player
    assert "/r1/livox/lidar /r1/mapping/imu_filtered" in player
    assert " /r0/odom" not in player
    assert " /r1/odom" not in player
    assert " /toy/initial_xy_alignment" not in player
    assert " /tf" not in player


def test_auto_imu_prefers_the_more_complete_filtered_stream(tmp_path: Path) -> None:
    bag = tmp_path / "combined"
    _write_metadata(bag, _representative_topics())

    result = _dry_run(bag)

    assert result.returncode == 0, result.stderr
    assert "INPUT_R0_IMU=/r0/mapping/imu_filtered (2088 messages, filtered)" in result.stdout
    assert "INPUT_R1_IMU=/r1/mapping/imu_filtered (3053 messages, filtered)" in result.stdout
    assert "two_live_mapping.launch.py" in result.stdout
    assert "two_live_combined_bag_mapping.launch.py" not in result.stdout
    assert "enable_robot0_pipeline:=true" in result.stdout
    assert "enable_robot1_pipeline:=true" in result.stdout
    assert "enable_fusion:=true" in result.stdout
    assert "publish_sensor_static_tf:=true" in result.stdout
    assert "wait_for_initial_alignment:=true" in result.stdout
    assert "robot0_imu_input_is_filtered:=true" in result.stdout
    assert "robot1_imu_input_is_filtered:=true" in result.stdout
    assert "R0_FILTERED_IMU_BYPASS=true" in result.stdout
    assert "R1_FILTERED_IMU_BYPASS=true" in result.stdout
    assert "wait_imu_to_init:=false" in result.stdout
    assert "expected_update_rate:=0.0" in result.stdout


def test_place_recognition_is_opt_in_for_combined_replay(tmp_path: Path) -> None:
    # Given: a valid combined bag at the shared-start location.
    bag = tmp_path / "combined"
    _write_metadata(bag, _representative_topics())

    # When: the default runner command is printed.
    default_result = _dry_run(bag)

    # Then: startup ICP supplies the initial transform without the later
    # occupancy place recognizer.
    assert default_result.returncode == 0, default_result.stderr
    assert "enable_place_recognition:=false" in default_result.stdout

    # When: an operator explicitly enables later place recognition.
    enabled_result = _dry_run(bag, "--enable-place-recognition")

    # Then: the public launch receives the opt-in boolean.
    assert enabled_result.returncode == 0, enabled_result.stderr
    assert "enable_place_recognition:=true" in enabled_result.stdout


def test_replay_rate_scales_mapping_and_alignment_wall_timers(tmp_path: Path) -> None:
    bag = tmp_path / "combined"
    _write_metadata(bag, _representative_topics())

    result = _dry_run(
        bag,
        "--rate",
        "0.5",
        "--startup-delay",
        "5",
        "--sensor-warmup",
        "10",
        "--alignment-warmup",
        "3",
        "--alignment-period",
        "2",
    )

    assert result.returncode == 0, result.stderr
    assert "MAPPING_STARTUP_DELAY_WALL=25.000s" in result.stdout
    assert "ALIGNMENT_STARTUP_DELAY_WALL=6.000s" in result.stdout
    assert "ALIGNMENT_RECOMPUTE_PERIOD_WALL=4.000s" in result.stdout
    assert "mapping_startup_delay_sec:=25.000" in result.stdout
    assert "alignment_startup_delay_sec:=6.000" in result.stdout
    assert "alignment_recompute_period_sec:=4.000" in result.stdout


def test_startup_gate_default_keeps_one_recorded_second_for_final_alignment(
    tmp_path: Path,
) -> None:
    bag = tmp_path / "combined"
    _write_metadata(bag, _representative_topics())

    result = _dry_run(bag)

    assert result.returncode == 0, result.stderr
    assert "RECORDED_ALIGNMENT_WARMUP=1s" in result.stdout
    assert "ALIGNMENT_STARTUP_DELAY_WALL=2.000s" in result.stdout
    assert "alignment_startup_delay_sec:=2.000" in result.stdout


def test_raw_imu_mode_can_be_forced(tmp_path: Path) -> None:
    bag = tmp_path / "combined"
    _write_metadata(bag, _representative_topics())

    result = _dry_run(bag, "--imu-source", "raw")

    assert result.returncode == 0, result.stderr
    assert "INPUT_R0_IMU=/r0/livox/imu (51 messages, raw)" in result.stdout
    assert "INPUT_R1_IMU=/r1/livox/imu (660 messages, raw)" in result.stdout
    assert "robot0_imu_input_is_filtered:=false" in result.stdout
    assert "robot1_imu_input_is_filtered:=false" in result.stdout


def test_prefiltered_imu_is_forwarded_through_the_public_live_launch() -> None:
    pipeline = (
        REPOSITORY_ROOT / "launch" / "mid360_mapping_pipeline.launch.py"
    ).read_text(encoding="utf-8")
    live = (REPOSITORY_ROOT / "launch" / "live_mapping.launch.py").read_text(
        encoding="utf-8"
    )
    two_live = (REPOSITORY_ROOT / "launch" / "two_live_mapping.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'DeclareLaunchArgument("imu_input_is_filtered", default_value="false")' in pipeline
    assert "if not imu_input_is_filtered:" in pipeline
    assert "imu_raw_topic if imu_input_is_filtered else imu_filter_output_topic" in pipeline
    assert '"imu_input_is_filtered": LaunchConfiguration(' in live
    assert "robot0_imu_input_is_filtered" in two_live
    assert "robot1_imu_input_is_filtered" in two_live


def test_public_live_launch_keeps_bag_alignment_profiles() -> None:
    two_live = (REPOSITORY_ROOT / "launch" / "two_live_mapping.launch.py").read_text(
        encoding="utf-8"
    )
    public_wrapper = (
        REPOSITORY_ROOT / "launch" / "two_live_plane_height_mapping.launch.py"
    ).read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("alignment_config_file", default_value="")' in two_live
    assert (
        'DeclareLaunchArgument(\n'
        '                "enable_place_recognition",\n'
        '                default_value="false",'
    ) in two_live
    assert "if enable_fusion and enable_place_recognition:" in two_live
    assert "_active_alignment_config_file" in public_wrapper
    assert "alignment_parameters.append(_active_alignment_config_file)" in public_wrapper


def test_extra_launch_arguments_are_forwarded_after_safe_defaults(tmp_path: Path) -> None:
    bag = tmp_path / "combined"
    _write_metadata(bag, _representative_topics())

    result = _dry_run(
        bag,
        "--launch-arg",
        "alignment_required_consistent_results:=3",
    )

    assert result.returncode == 0, result.stderr
    mapping_line = next(
        line for line in result.stdout.splitlines() if line.startswith("MAPPING=")
    )
    assert "alignment_required_consistent_results:=3" in mapping_line


def test_combined_replay_rejects_missing_robot_lidar(tmp_path: Path) -> None:
    bag = tmp_path / "combined"
    topics = _representative_topics()
    del topics["/r0/livox/lidar"]
    _write_metadata(bag, topics)

    result = _dry_run(bag)

    assert result.returncode != 0
    assert "missing or empty topic /r0/livox/lidar" in result.stderr


def test_custom_source_topics_are_remapped_into_the_two_robot_pipeline(
    tmp_path: Path,
) -> None:
    # Given: a combined bag using S3E robot topic names.
    bag = tmp_path / "S3E_Laboratory_1"
    _write_metadata(
        bag,
        {
            "/Alpha/velodyne_points": ("sensor_msgs/msg/PointCloud2", 100),
            "/Alpha/imu/data": ("sensor_msgs/msg/Imu", 1000),
            "/Carol/velodyne_points": ("sensor_msgs/msg/PointCloud2", 101),
            "/Carol/imu/data": ("sensor_msgs/msg/Imu", 1001),
        },
    )

    # When: exact source topics are selected for r0 and r1.
    result = _dry_run(
        bag,
        "--robot0-lidar-source",
        "/Alpha/velodyne_points",
        "--robot0-imu-source",
        "/Alpha/imu/data",
        "--robot1-lidar-source",
        "/Carol/velodyne_points",
        "--robot1-imu-source",
        "/Carol/imu/data",
    )

    # Then: only those streams are replayed into the isolated mapping topics.
    assert result.returncode == 0, result.stderr
    player = next(
        line for line in result.stdout.splitlines() if line.startswith("BAG_PLAYER=")
    )
    assert "--topics /Alpha/velodyne_points /Alpha/imu/data" in player
    assert "/Carol/velodyne_points /Carol/imu/data" in player
    assert "/Alpha/velodyne_points:=/co_3dto2d_replay/r0/lidar" in player
    assert "/Carol/velodyne_points:=/co_3dto2d_replay/r1/lidar" in player
    assert "robot0_imu_input_is_filtered:=false" in result.stdout
    assert "robot1_imu_input_is_filtered:=false" in result.stdout
