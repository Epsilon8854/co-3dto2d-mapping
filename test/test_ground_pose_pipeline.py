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


def test_shared_floor_wrapper_has_one_fit_callback_per_cloud_odom_pair():
    wrapper = (
        PACKAGE / "co_3dto2d_mapping" / "gravity_plane_height_cloud.py"
    ).read_text()
    assert "def _synchronized_callback(" in wrapper
    assert "This is the only floor-fitting callback" in wrapper
    assert "def _height_cloud_callback(" not in wrapper
    assert "_height_cloud_subscription" not in wrapper
    assert "estimate_single_floor_plane(" in wrapper
    assert "_publish_mapping_cloud(" in wrapper
    assert "_fused_mapping_odometry(" in wrapper


def test_projection_ablation_keeps_filtered_cloud_but_passes_raw_pose():
    wrapper = (
        PACKAGE / "co_3dto2d_mapping" / "gravity_plane_height_cloud.py"
    ).read_text()
    config = (PACKAGE / "config" / "occupancy.yaml").read_text()
    assert "if not self.enabled or normal is None or height_m is None:" in wrapper
    assert "ground_plane_pose_enabled: true" in config
    assert "Set ground_plane_pose_enabled=false" in config
    assert 'ground_plane_planar_odometry_topic: "odom"' in config
    assert 'ground_plane_output_odometry_topic: "mapping/floor_odometry"' in config
    assert 'corrected_odometry_topic: "toy/corrected_odometry"' in config


def test_default_config_encodes_flat_single_floor_mode():
    text = (PACKAGE / "config" / "occupancy.yaml").read_text()
    assert 'ground_plane_estimation_mode: "single_floor"' in text
    assert "ground_plane_single_floor_bin_size_m: 0.025" in text
    assert "ground_plane_single_floor_lowest_support_ratio: 0.55" in text
    assert "ground_plane_max_normal_deviation_deg: 8.0" in text
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
        PACKAGE / "co_3dto2d_mapping" / "gravity_plane_height_cloud.py"
    ).read_text()
    assert "ament_python_install_package" in cmake
    assert "gravity_plane_height_cloud.py" in cmake
    assert "RENAME gravity_plane_pose_fusion.py" in cmake
    assert "single_floor_plane" in wrapper


def test_public_record_republisher_uses_fixed_common_world_frame():
    cmake = (PACKAGE / "CMakeLists.txt").read_text()
    world = (
        PACKAGE / "co_3dto2d_mapping" / "record_republisher_world.py"
    ).read_text()
    assert "record_republisher_ground_fused.py" in cmake
    assert "record_republisher_world.py" in cmake
    assert "RENAME record_republisher.py" in cmake
    assert "lock_world_alignment" in world
    assert "output.header.frame_id = self.common_frame_id" in world
