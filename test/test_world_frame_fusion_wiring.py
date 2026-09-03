from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]


def test_public_alignment_executable_is_odom_aware():
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


def test_remote_and_local_alignment_use_the_same_filtered_cloud_type():
    wrapper = (
        PACKAGE / "launch" / "two_live_plane_height_mapping.launch.py"
    ).read_text()

    assert "_plane_height_node" in wrapper
    assert 'parameter_set["robot0_cloud_topic"] = "/r0%s"' in wrapper
    assert 'parameter_set["robot1_cloud_topic"] = "/r1%s"' in wrapper
    assert "_BASE.Node = _plane_height_node" in wrapper
