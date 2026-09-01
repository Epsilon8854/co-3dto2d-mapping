from __future__ import annotations

import os
from pathlib import Path
import subprocess


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
RUNNER = REPOSITORY_ROOT / "scripts" / "run_two_live_airoom_bags.sh"


def _write_metadata(bag_path: Path, topics: tuple[str, ...]) -> None:
    bag_path.mkdir()
    topic_entries = "\n".join(
        "    - topic_metadata:\n"
        f"        name: {topic}\n"
        f"        type: {'sensor_msgs/msg/Imu' if topic.endswith('/imu') else 'sensor_msgs/msg/PointCloud2'}\n"
        "      message_count: 10"
        for topic in topics
    )
    (bag_path / "metadata.yaml").write_text(
        "rosbag2_bagfile_information:\n"
        "  duration:\n"
        "    nanoseconds: 100000000000\n"
        "  topics_with_message_count:\n"
        + topic_entries
        + "\n",
        encoding="utf-8",
    )


def _dry_run(bag0: Path, bag1: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            "bash",
            str(RUNNER),
            "--dry-run",
            "--bag0",
            str(bag0),
            "--bag1",
            str(bag1),
            "--workspace",
            str(REPOSITORY_ROOT),
        ],
        check=False,
        capture_output=True,
        text=True,
    )


def test_dry_run_selects_only_sensor_topics_and_isolates_namespaces(
    tmp_path: Path,
) -> None:
    # Given: two bags containing the shared recorded sensor topic names.
    bag0 = tmp_path / "bag0"
    bag1 = tmp_path / "bag1"
    topics = ("/r1/livox/lidar", "/r1/livox/imu", "/tf", "/r1/odom")
    _write_metadata(bag0, topics)
    _write_metadata(bag1, topics)

    # When: the runner resolves its commands without launching ROS.
    result = _dry_run(bag0, bag1)

    # Then: each player has a sensor-only allowlist and a unique destination.
    assert result.returncode == 0, result.stderr
    player_lines = [
        line for line in result.stdout.splitlines() if line.startswith("BAG_PLAYER_")
    ]
    assert len(player_lines) == 2
    assert all("--topics /r1/livox/lidar /r1/livox/imu" in line for line in player_lines)
    assert "/co_3dto2d_replay/r0/lidar" in player_lines[0]
    assert "/co_3dto2d_replay/r1/lidar" in player_lines[1]
    assert all(" /tf" not in line and " /r1/odom" not in line for line in player_lines)


def test_dry_run_rejects_bag_without_required_imu(tmp_path: Path) -> None:
    # Given: one valid bag and one bag missing the required raw IMU topic.
    bag0 = tmp_path / "bag0"
    bag1 = tmp_path / "bag1"
    _write_metadata(bag0, ("/r1/livox/lidar", "/r1/livox/imu"))
    _write_metadata(bag1, ("/r1/livox/lidar",))

    # When: the runner validates both bag boundaries.
    result = _dry_run(bag0, bag1)

    # Then: it fails before starting a partially connected live pipeline.
    assert result.returncode != 0
    assert "missing topic /r1/livox/imu" in result.stderr


def test_environment_check_sources_foxy_setup_with_unset_variables(
    tmp_path: Path,
) -> None:
    # Given: a Foxy-style setup script that reads an unset variable under nounset.
    bag0 = tmp_path / "bag0"
    bag1 = tmp_path / "bag1"
    _write_metadata(bag0, ("/r1/livox/lidar", "/r1/livox/imu"))
    _write_metadata(bag1, ("/r1/livox/lidar", "/r1/livox/imu"))
    workspace = tmp_path / "workspace"
    (workspace / "install").mkdir(parents=True)
    (workspace / "install" / "local_setup.bash").write_text("true\n", encoding="utf-8")
    ros_setup = tmp_path / "setup.bash"
    ros_setup.write_text(
        'if [ -z "$AMENT_TRACE_SETUP_FILES" ]; then :; fi\n',
        encoding="utf-8",
    )
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    fake_ros2 = fake_bin / "ros2"
    fake_ros2.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
    fake_ros2.chmod(0o755)
    fake_python = fake_bin / "python3"
    fake_python.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
    fake_python.chmod(0o755)
    environment = os.environ.copy()
    environment["PATH"] = f"{fake_bin}:{environment['PATH']}"
    environment["ROS_SETUP"] = str(ros_setup)
    environment.pop("AMENT_TRACE_SETUP_FILES", None)

    # When: the runner performs its non-mutating environment check.
    result = subprocess.run(
        [
            "bash",
            str(RUNNER),
            "--check-environment",
            "--bag0",
            str(bag0),
            "--bag1",
            str(bag1),
            "--workspace",
            str(workspace),
        ],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )

    # Then: setup sourcing succeeds instead of aborting on the unset variable.
    assert result.returncode == 0, result.stderr
    assert "Environment check passed" in result.stdout
    assert "Python=/usr/bin/python3" in result.stdout
