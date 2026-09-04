from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]


def test_single_mapping_uses_one_floor_state_for_cloud_and_projection():
    text = (PACKAGE / "launch" / "single_bag_mapping.launch.py").read_text()
    required = (
        'executable="gravity_plane_pose_fusion.py"',
        '"ground_plane_pointcloud_topic": raw_scan_cloud_topic',
        '"ground_plane_filtered_cloud_topic": plane_filtered_cloud_topic',
        '"ground_plane_planar_odometry_topic": "odom"',
        '"ground_plane_output_odometry_topic": mapping_odometry_topic',
        '"scan_cloud_topic": plane_filtered_cloud_topic',
        '"odom_topic": mapping_odometry_topic',
        '"corrected_odometry_topic": corrected_odometry_topic',
        'DeclareLaunchArgument(\n                "mapping_odometry_topic"',
        'default_value="mapping/floor_odometry"',
        'DeclareLaunchArgument(\n                "corrected_odometry_topic"',
    )
    missing = [fragment for fragment in required if fragment not in text]
    assert missing == []


def test_mapping_odometry_topic_cannot_create_a_feedback_loop():
    text = (PACKAGE / "launch" / "single_bag_mapping.launch.py").read_text()
    assert 'mapping_odometry_topic in {"odom", corrected_odometry_topic}' in text
    assert "mapping_odometry_topic must differ from raw odom" in text


def test_namespaced_ground_plane_node_uses_global_tf_topics():
    text = (PACKAGE / "launch" / "single_bag_mapping.launch.py").read_text()
    assert 'remappings=[("tf", "/tf"), ("tf_static", "/tf_static")]' in text


def test_default_config_uses_one_meter_plane_height_band_with_pose_fusion_off():
    text = (PACKAGE / "config" / "occupancy.yaml").read_text()
    assert 'corrected_odometry_topic: "toy/planar_odometry"' in text
    assert 'ground_plane_planar_odometry_topic: "toy/planar_odometry"' in text
    assert 'ground_plane_output_odometry_topic: "toy/corrected_odometry"' in text
    assert 'ground_plane_filtered_cloud_topic: "mapping/plane_height_filtered"' in text
    assert "ground_plane_pose_enabled: false" in text
    assert "ground_plane_height_filter_enabled: true" in text
    assert "ground_plane_filter_min_height_m: 0.05" in text
    assert "ground_plane_filter_max_height_m: 1.00" in text
    assert "z_min: -1000.0" in text
    assert "z_max: 1000.0" in text
    assert "invert_z_slice: false" in text
    assert 'ground_plane_z_mode: "height_above_plane"' in text


def test_cmake_installs_single_floor_module_and_shared_wrapper():
    cmake = (PACKAGE / "CMakeLists.txt").read_text()
    wrapper = (
        PACKAGE
        / "co_3dto2d_mapping"
        / "gravity_plane_height_cloud.py"
    ).read_text()
    assert "ament_python_install_package" in cmake
    assert "gravity_plane_height_cloud.py" in cmake
    assert "RENAME gravity_plane_pose_fusion.py" in cmake
    assert "single_floor_plane" in wrapper


def test_public_record_republisher_uses_fixed_common_world_frame():
    cmake = (PACKAGE / "CMakeLists.txt").read_text()
    wrapper = (
        PACKAGE
        / "co_3dto2d_mapping"
        / "record_republisher_ground_fused.py"
    ).read_text()
    assert "record_republisher_ground_fused.py" in cmake
    assert "record_republisher_world.py" in cmake
    assert "RENAME record_republisher.py" in cmake
    assert "/r{robot_id}/toy/corrected_odometry" in wrapper
    assert "TemporalToyRecordRepublisher" in wrapper
    assert 'declare_parameter("ground_fused_odometry_timeout_sec", 3.0)' in wrapper
    assert "falling back to /r%d/odom" in wrapper
    assert "self._refresh_odometry_selection()" in wrapper


def test_two_live_public_launch_aligns_plane_filtered_clouds():
    cmake = (PACKAGE / "CMakeLists.txt").read_text()
    wrapper = (
        PACKAGE
        / "launch"
        / "two_live_plane_height_mapping.launch.py"
    ).read_text()
    assert "RENAME two_live_mapping_base.launch.py" in cmake
    assert "RENAME two_live_mapping.launch.py" in cmake
    assert 'filtered_topic = "/r%d%s"' in wrapper
    assert 'name == "alignment_use_z_filter"' in wrapper
    assert "return False" in wrapper
