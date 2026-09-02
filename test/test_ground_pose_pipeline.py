from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]


def test_single_mapping_launches_ground_plane_fusion_after_planar_mapper():
    text = (PACKAGE / "launch" / "single_bag_mapping.launch.py").read_text()
    required = (
        'executable="gravity_plane_pose_fusion.py"',
        '"corrected_odometry_topic": planar_odometry_topic',
        '"ground_plane_planar_odometry_topic": planar_odometry_topic',
        '"ground_plane_output_odometry_topic": corrected_odometry_topic',
        'DeclareLaunchArgument(\n                "planar_odometry_topic"',
        'DeclareLaunchArgument(\n                "corrected_odometry_topic"',
    )
    missing = [fragment for fragment in required if fragment not in text]
    assert missing == []


def test_default_config_separates_planar_and_final_odometry_topics():
    text = (PACKAGE / "config" / "occupancy.yaml").read_text()
    assert 'corrected_odometry_topic: "toy/planar_odometry"' in text
    assert 'ground_plane_planar_odometry_topic: "toy/planar_odometry"' in text
    assert 'ground_plane_output_odometry_topic: "toy/corrected_odometry"' in text
    assert "ground_plane_pose_enabled: true" in text
    assert 'ground_plane_z_mode: "height_above_plane"' in text


def test_record_republisher_prefers_final_ground_fused_pose():
    cmake = (PACKAGE / "CMakeLists.txt").read_text()
    wrapper = (
        PACKAGE / "co_3dto2d_mapping" / "record_republisher_ground_fused.py"
    ).read_text()
    assert "record_republisher_ground_fused.py" in cmake
    assert "RENAME record_republisher.py" in cmake
    assert "/r{robot_id}/toy/corrected_odometry" in wrapper
    assert "TemporalToyRecordRepublisher" in wrapper
    assert "raw /rN/odom remains the startup fallback" in wrapper
