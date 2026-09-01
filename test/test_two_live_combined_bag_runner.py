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
    assert "wait_imu_to_init:=false" in result.stdout


def test_mapping_delay_preserves_ten_recorded_seconds_at_half_rate(
    tmp_path: Path,
) -> None:
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
    )

    assert result.returncode == 0, result.stderr
    assert "MAPPING_STARTUP_DELAY_WALL=25.000s" in result.stdout
    assert "mapping_startup_delay_sec:=25.000" in result.stdout


def test_raw_imu_mode_can_be_forced(tmp_path: Path) -> None:
    bag = tmp_path / "combined"
    _write_metadata(bag, _representative_topics())

    result = _dry_run(bag, "--imu-source", "raw")

    assert result.returncode == 0, result.stderr
    assert "INPUT_R0_IMU=/r0/livox/imu (51 messages, raw)" in result.stdout
    assert "INPUT_R1_IMU=/r1/livox/imu (660 messages, raw)" in result.stdout


def test_combined_replay_rejects_missing_robot_lidar(tmp_path: Path) -> None:
    bag = tmp_path / "combined"
    topics = _representative_topics()
    del topics["/r0/livox/lidar"]
    _write_metadata(bag, topics)

    result = _dry_run(bag)

    assert result.returncode != 0
    assert "missing or empty topic /r0/livox/lidar" in result.stderr
