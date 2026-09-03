from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]


def test_legacy_alignment_executable_remains_odom_aware():
    cmake = (PACKAGE / "CMakeLists.txt").read_text()
    implementation = (
        PACKAGE
        / "co_3dto2d_mapping"
        / "odom_aware_cropped_xyz_alignment.py"
    ).read_text()

    assert "odom_aware_cropped_xyz_alignment.py" in cmake
    assert "RENAME initial_xy_icp_alignment.py" in cmake
    assert "world_from_source_odom" in implementation
    assert "/r0/toy/planar_odometry" in implementation
    assert "/r1/toy/planar_odometry" in implementation
    assert "max_submap_motion_translation_m" in implementation


def test_public_record_republisher_outputs_stable_common_frame_odometry():
    cmake = (PACKAGE / "CMakeLists.txt").read_text()
    implementation = (
        PACKAGE / "co_3dto2d_mapping" / "record_republisher_world.py"
    ).read_text()

    assert "record_republisher_world.py" in cmake
    assert "RENAME record_republisher.py" in cmake
    assert "suppress_unaligned_world_odometry" in implementation
    assert "lock_world_alignment" in implementation
    assert "output.header.frame_id = self.common_frame_id" in implementation


def test_two_live_public_launch_selects_occupancy_place_recognition():
    wrapper = (
        PACKAGE / "launch" / "two_live_plane_height_mapping.launch.py"
    ).read_text()
    cmake = (PACKAGE / "CMakeLists.txt").read_text()

    assert "_place_recognition_node" in wrapper
    assert 'rewritten["executable"] = "inter_robot_place_alignment.py"' in wrapper
    assert '"config", "place_recognition.yaml"' in wrapper
    assert '"robot0_odom_topic": "/r0/toy/planar_odometry"' in wrapper
    assert '"robot1_odom_topic": "/r1/toy/planar_odometry"' in wrapper
    assert "_BASE.Node = _place_recognition_node" in wrapper
    assert "co_3dto2d_mapping/inter_robot_place_alignment.py" in cmake


def test_place_recognition_defaults_require_consensus_and_lock():
    config = (PACKAGE / "config" / "place_recognition.yaml").read_text()

    assert "consensus_min_measurements: 3" in config
    assert "consensus_min_distinct_keyframes: 2" in config
    assert "lock_after_consensus: true" in config
    assert "stop_processing_after_lock: true" in config
