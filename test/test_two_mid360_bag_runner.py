from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
RUNNER = REPOSITORY_ROOT / "scripts" / "run_two_mid360_2d_mapping_bag.sh"
AIROOM_ALIGNMENT_PROFILE = (
    REPOSITORY_ROOT / "config" / "airoom_chair_replay_place_recognition.yaml"
)
ALIGNMENT_EXECUTABLE = (
    REPOSITORY_ROOT / "co_3dto2d_mapping" / "inter_robot_place_alignment.py"
)


def _write_combined_bag_metadata(bag_path: Path) -> None:
    bag_path.mkdir()
    topic_entries = "\n".join(
        f"""    - topic_metadata:
        name: {name}
        type: {message_type}
      message_count: {message_count}"""
        for name, message_type, message_count in (
            ("/r0/livox/lidar", "sensor_msgs/msg/PointCloud2", 10),
            ("/r0/livox/imu", "sensor_msgs/msg/Imu", 10),
            ("/r1/livox/lidar", "sensor_msgs/msg/PointCloud2", 10),
            ("/r1/livox/imu", "sensor_msgs/msg/Imu", 10),
        )
    )
    metadata = f"""rosbag2_bagfile_information:
  storage_identifier: sqlite3
  topics_with_message_count:
{topic_entries}
"""
    _ = (bag_path / "metadata.yaml").write_text(metadata, encoding="utf-8")


def _dry_run(default_bag: Path, *arguments: str) -> subprocess.CompletedProcess[str]:
    environment = os.environ.copy()
    environment["TWO_MID360_BAG_PATH"] = str(default_bag)
    return subprocess.run(
        ["bash", str(RUNNER), "--dry-run", *arguments],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )


def test_dry_run_replays_one_combined_bag_without_driver_processes(
    tmp_path: Path,
) -> None:
    # Given: a combined bag supplied through the wrapper's default-bag boundary.
    bag = tmp_path / "combined"
    _write_combined_bag_metadata(bag)

    # When: the MID-360 bag shortcut is resolved without launching ROS.
    result = _dry_run(bag, "--domain-id", "174", "--rate", "1.0")

    # Then: it delegates to one combined-bag player and never requests robot hardware.
    assert result.returncode == 0, result.stderr
    assert f"BAG={bag}" in result.stdout
    assert "ROS_DOMAIN_ID=174" in result.stdout
    assert "two_live_mapping.launch.py" in result.stdout
    assert "two_live_combined_bag_mapping.launch.py" not in result.stdout
    assert "wait_for_initial_alignment:=true" in result.stdout
    assert re.search(
        rf"OCCUPANCY_PNG_OUTPUT={re.escape(str(REPOSITORY_ROOT))}/results/"
        r"\d{8}_\d{6}_\d{9}/output",
        result.stdout,
    )
    assert "BAG_PLAYER= ros2 bag play" in result.stdout
    assert (
        "--topics /r0/livox/lidar /r0/livox/imu /r1/livox/lidar /r1/livox/imu"
        in result.stdout
    )
    assert "run_livox_robot.sh" not in result.stdout
    assert f"alignment_config_file:={AIROOM_ALIGNMENT_PROFILE}" in result.stdout


def test_explicit_bag_overrides_the_wrapper_default(tmp_path: Path) -> None:
    # Given: a default bag and a different explicit bag.
    default_bag = tmp_path / "default"
    explicit_bag = tmp_path / "explicit"
    _write_combined_bag_metadata(default_bag)
    _write_combined_bag_metadata(explicit_bag)

    # When: an operator selects the explicit bag.
    result = _dry_run(default_bag, "--bag", str(explicit_bag))

    # Then: the child runner receives the explicit source, not the default.
    assert result.returncode == 0, result.stderr
    assert f"BAG={explicit_bag}" in result.stdout


def test_shortcut_is_installed_with_the_combined_bag_runner() -> None:
    # Given: the package's CMake install contract.
    cmake = (REPOSITORY_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    # When: the package is built for ros2 run.
    # Then: the MID-360 bag shortcut is installed alongside its delegate.
    assert "scripts/run_two_mid360_2d_mapping_bag.sh" in cmake
    assert "scripts/run_two_live_combined_bag.sh" in cmake


def test_runtime_checks_the_current_combined_alignment_acceptance_message() -> None:
    # Given: the combined-bag launch selects the occupancy aligner.
    runtime = (REPOSITORY_ROOT / "scripts" / "lib" / "two_live_runtime.bash").read_text(
        encoding="utf-8"
    )
    aligner = ALIGNMENT_EXECUTABLE.read_text(encoding="utf-8")

    # When: bag replay decides whether a usable shared alignment was produced.
    # Then: it recognizes the success message emitted by that aligner.
    accepted_message = "Inter-robot occupancy alignment accepted"
    assert accepted_message in aligner
    assert accepted_message in runtime


def test_combined_bag_aligner_is_executable_by_ros_launch() -> None:
    assert os.access(ALIGNMENT_EXECUTABLE, os.X_OK)


def test_default_mid360_profile_accepts_the_static_second_robot_snapshot() -> None:
    profile = AIROOM_ALIGNMENT_PROFILE.read_text(encoding="utf-8")

    assert "min_known_ratio: 0.09" in profile
    assert "consensus_min_measurements: 1" in profile
    assert "consensus_min_distinct_keyframes: 1" in profile
