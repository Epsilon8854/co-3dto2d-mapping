from __future__ import annotations

import os
from pathlib import Path
import subprocess


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
COMBINED_RUNNER = REPOSITORY_ROOT / "scripts" / "run_two_live_combined_bag.sh"
S3EV1_RUNNER = REPOSITORY_ROOT / "scripts" / "run_s3ev1_mapping.sh"
S3EV1_CONFIG = REPOSITORY_ROOT / "config" / "s3e_sparse_strict.yaml"
DEFAULT_CONFIG = REPOSITORY_ROOT / "config" / "occupancy.yaml"


def _write_metadata(
    bag_path: Path,
    topics: dict[str, tuple[str, int]],
    *,
    message_count_first: bool = False,
) -> None:
    bag_path.mkdir()
    if message_count_first:
        topic_entries = "\n".join(
            f"  - message_count: {message_count}\n"
            + "    topic_metadata:\n"
            + f"      name: {topic}\n"
            + f"      type: {message_type}"
            for topic, (message_type, message_count) in topics.items()
        )
    else:
        topic_entries = "\n".join(
            "  - topic_metadata:\n"
            + f"      name: {topic}\n"
            + f"      type: {message_type}\n"
            + f"    message_count: {message_count}"
            for topic, (message_type, message_count) in topics.items()
        )
    _ = (bag_path / "metadata.yaml").write_text(
        "rosbag2_bagfile_information:\n"
        + "  storage_identifier: sqlite3\n"
        + "  topics_with_message_count:\n"
        + topic_entries
        + "\n",
        encoding="utf-8",
    )


def _s3e_topics() -> dict[str, tuple[str, int]]:
    return {
        "/Alpha/velodyne_points": ("sensor_msgs/msg/PointCloud2", 100),
        "/Alpha/imu/data": ("sensor_msgs/msg/Imu", 1000),
        "/Bob/velodyne_points": ("sensor_msgs/msg/PointCloud2", 101),
        "/Bob/imu/data": ("sensor_msgs/msg/Imu", 1001),
    }


def test_custom_sources_support_rosbag2_message_count_first_metadata(
    tmp_path: Path,
) -> None:
    # Given: metadata using the field order emitted by the S3Ev1 rosbag2 files.
    bag = tmp_path / "S3E_Laboratory_1"
    _write_metadata(bag, _s3e_topics(), message_count_first=True)

    # When: the combined runner parses exact S3E source topics.
    result = subprocess.run(
        [
            "bash",
            str(COMBINED_RUNNER),
            "--dry-run",
            "--bag",
            str(bag),
            "--robot0-lidar-source",
            "/Alpha/velodyne_points",
            "--robot0-imu-source",
            "/Alpha/imu/data",
            "--robot1-lidar-source",
            "/Bob/velodyne_points",
            "--robot1-imu-source",
            "/Bob/imu/data",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    # Then: it recognizes each topic and preserves its recorded message count.
    assert result.returncode == 0, result.stderr
    assert "INPUT_R0_LIDAR=/Alpha/velodyne_points (100 messages)" in result.stdout
    assert "INPUT_R1_IMU=/Bob/imu/data (1001 messages, raw)" in result.stdout


def test_s3ev1_runner_selects_a_sequence_and_two_named_robots(
    tmp_path: Path,
) -> None:
    # Given: an S3Ev1 dataset root containing one representative sequence.
    sequence = tmp_path / "S3E_Laboratory_1"
    _write_metadata(sequence, _s3e_topics())

    # When: the dataset runner is resolved without launching ROS.
    result = subprocess.run(
        [
            "bash",
            str(S3EV1_RUNNER),
            "--dry-run",
            "--dataset-root",
            str(tmp_path),
            "--sequence",
            "S3E_Laboratory_1",
            "--robot0",
            "Bob",
            "--robot1",
            "Alpha",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    # Then: the selected S3E streams and Velodyne sensor orientation are forwarded.
    assert result.returncode == 0, result.stderr
    assert "S3E_SEQUENCE=S3E_Laboratory_1" in result.stdout
    assert "S3E_R0=Bob" in result.stdout
    assert "S3E_R1=Alpha" in result.stdout
    assert f"S3E_CONFIG={S3EV1_CONFIG}" in result.stdout
    assert "/Bob/velodyne_points" in result.stdout
    assert "/Alpha/imu/data" in result.stdout
    assert f"occupancy_config_file:={S3EV1_CONFIG}" in result.stdout
    assert f"alignment_config_file:={S3EV1_CONFIG}" in result.stdout
    assert "sensor_tf_roll_0:=0.0" in result.stdout
    assert "sensor_tf_roll_1:=0.0" in result.stdout


def test_s3ev1_profile_is_strict_while_accepting_sparse_evidence() -> None:
    # Given: the S3Ev1 mapping profile.
    inspection = (
        "import sys, yaml; "
        "profile = yaml.safe_load(open(sys.argv[1], encoding='utf-8')); "
        "occupancy = profile['/**']['ros__parameters']; "
        "alignment = profile['/initial_xy_icp_alignment']['ros__parameters']; "
        "print("
        "alignment['min_fitness'] >= 0.25, "
        "alignment['max_rmse'] <= 0.25, "
        "alignment['required_consistent_results'] >= 3, "
        "alignment['min_correspondences'] <= 75, "
        "alignment['max_correspondence_distance'] >= 0.75, "
        "occupancy['grid_resolution'] >= 0.10, "
        "occupancy['icp_window_size'] >= 12, "
        "occupancy['ground_plane_min_inliers'] <= 60, "
        "occupancy['dynamic_filter_enabled'] is False, "
        "occupancy['ground_plane_pose_enabled'] is False"
        ")"
    )

    # When: the machine-consumed thresholds are loaded from YAML.
    result = subprocess.run(
        ["/usr/bin/python3", "-c", inspection, str(S3EV1_CONFIG)],
        check=False,
        capture_output=True,
        text=True,
    )

    # Then: strict sparse-input settings remain, with feature ablations off.
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == (
        "True True True True True True True True True True"
    )


def test_master_baseline_defaults_disable_dynamic_and_plane_pose() -> None:
    inspection = (
        "import sys, yaml; "
        "profile = yaml.safe_load(open(sys.argv[1], encoding='utf-8')); "
        "params = profile['/**']['ros__parameters']; "
        "print(params['dynamic_filter_enabled'] is False, "
        "params['ground_plane_pose_enabled'] is False)"
    )
    result = subprocess.run(
        ["/usr/bin/python3", "-c", inspection, str(DEFAULT_CONFIG)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "True True"

    mapper_parameters = (
        REPOSITORY_ROOT
        / "include"
        / "co_3dto2d_mapping"
        / "occupancy_mapper"
        / "parameters.inc"
    ).read_text(encoding="utf-8")
    merged_temporal = (
        REPOSITORY_ROOT
        / "co_3dto2d_mapping"
        / "record_republisher_temporal.py"
    ).read_text(encoding="utf-8")
    assert 'declare_parameter<bool>("dynamic_filter_enabled", false);' in mapper_parameters
    assert 'declare_parameter("merged_temporal_filter_enabled", False)' in merged_temporal


def test_environment_check_ignores_an_inherited_ros1_distro(
    tmp_path: Path,
) -> None:
    # Given: a valid bag and a shell whose active distribution is ROS 1 Noetic.
    bag = tmp_path / "combined"
    _write_metadata(
        bag,
        {
            "/r0/livox/lidar": ("sensor_msgs/msg/PointCloud2", 100),
            "/r0/livox/imu": ("sensor_msgs/msg/Imu", 1000),
            "/r1/livox/lidar": ("sensor_msgs/msg/PointCloud2", 101),
            "/r1/livox/imu": ("sensor_msgs/msg/Imu", 1001),
        },
    )
    environment = os.environ.copy()
    _ = environment.pop("ROS_SETUP", None)
    environment["ROS_DISTRO"] = "noetic"
    environment["ROS_VERSION"] = "1"

    # When: the ROS 2 runner resolves and checks its environment.
    result = subprocess.run(
        [
            "bash",
            str(COMBINED_RUNNER),
            "--check-environment",
            "--bag",
            str(bag),
            "--workspace",
            str(REPOSITORY_ROOT),
        ],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )

    # Then: it selects an installed ROS 2 setup instead of sourcing Noetic.
    assert result.returncode == 0, result.stderr
    assert "Environment check passed: ROS=/opt/ros/foxy/setup.bash" in result.stdout
