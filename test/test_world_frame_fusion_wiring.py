from pathlib import Path
import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def test_public_two_live_launch_selects_fixed_occupancy_alignment():
    wrapper = (
        PACKAGE / "launch" / "two_live_plane_height_mapping.launch.py"
    ).read_text()
    cmake = (PACKAGE / "CMakeLists.txt").read_text()

    assert "_place_recognition_node" in wrapper
    assert 'rewritten["executable"] = "inter_robot_place_alignment.py"' in wrapper
    assert '"config", "place_recognition.yaml"' in wrapper
    assert '"robot0_odom_topic": "/r0/toy/corrected_odometry"' in wrapper
    assert '"robot1_odom_topic": "/r1/toy/corrected_odometry"' in wrapper
    assert "_BASE.Node = _place_recognition_node" in wrapper
    assert "co_3dto2d_mapping/inter_robot_place_alignment.py" in cmake


def test_combined_bag_launch_uses_profile_then_explicit_overrides():
    text = (
        PACKAGE / "launch" / "two_live_combined_bag_mapping.launch.py"
    ).read_text()
    assert 'executable="inter_robot_place_alignment.py"' in text
    assert '"robot0_odom_topic": "/r0/toy/corrected_odometry"' in text
    assert '"robot1_odom_topic": "/r1/toy/corrected_odometry"' in text
    assert "alignment_parameters = [place_config_file]" in text
    assert "alignment_parameters.append(alignment_config_file)" in text
    assert "alignment_parameters.append(alignment_overrides)" in text
    assert '"processing_period_sec": float(' in text


def test_default_consensus_requires_motion_separated_support_and_locks():
    profile = yaml.safe_load(
        (PACKAGE / "config" / "place_recognition.yaml").read_text()
    )["/**"]["ros__parameters"]
    assert profile["stationary_keyframe_period_sec"] >= 3600.0
    assert profile["keyframe_translation_m"] > 0.0
    assert profile["keyframe_rotation_rad"] > 0.0
    assert profile["require_mutual_best_match"] is True
    assert profile["descriptor_ratio_test"] < 1.0
    assert profile["consensus_min_measurements"] >= 3
    assert profile["consensus_min_distinct_keyframes"] >= 2
    assert profile["lock_after_consensus"] is True
    assert profile["stop_processing_after_lock"] is True


def test_s3e_profile_selects_single_floor_and_stricter_fixed_alignment():
    profile = yaml.safe_load(
        (PACKAGE / "config" / "s3e_sparse_strict.yaml").read_text()
    )
    mapping = profile["/**"]["ros__parameters"]
    alignment = profile["/inter_robot_place_alignment"]["ros__parameters"]

    assert mapping["ground_plane_estimation_mode"] == "single_floor"
    assert mapping["ground_plane_planar_odometry_topic"] == "odom"
    assert mapping["ground_plane_output_odometry_topic"] == "mapping/floor_odometry"
    assert mapping["corrected_odometry_topic"] == "toy/corrected_odometry"
    assert alignment["robot0_odom_topic"] == "/r0/toy/corrected_odometry"
    assert alignment["robot1_odom_topic"] == "/r1/toy/corrected_odometry"
    assert alignment["stationary_keyframe_period_sec"] >= 3600.0
    assert alignment["consensus_min_measurements"] >= 4
    assert alignment["consensus_min_distinct_keyframes"] >= 2
    assert alignment["consensus_max_translation_rms_m"] <= 0.12
    assert alignment["lock_after_consensus"] is True


def test_public_record_output_latches_alignment_and_maps_in_common_frame():
    cmake = (PACKAGE / "CMakeLists.txt").read_text()
    implementation = (
        PACKAGE / "co_3dto2d_mapping" / "record_republisher_world.py"
    ).read_text()

    assert "record_republisher_world.py" in cmake
    assert "RENAME record_republisher.py" in cmake
    assert "suppress_unaligned_world_odometry" in implementation
    assert "publish_world_maps" in implementation
    assert "suppress_unaligned_world_maps" in implementation
    assert "transform_grid_to_common_frame" in implementation
    assert "lock_world_alignment" in implementation
    assert "output.header.frame_id = self.common_frame_id" in implementation


def test_local_observation_grid_bakes_corrected_odometry_into_cell_coordinates():
    implementation = (
        PACKAGE
        / "include"
        / "co_3dto2d_mapping"
        / "occupancy_mapper"
        / "sensor_and_local_grid.inc"
    ).read_text()

    assert "latest_local_grid_ = buildLocalGrid(\n        global_hits" in implementation
    assert "global_free_ray_hits, cloud->header.stamp, aligned_t" in implementation
    assert "const Vec3 &sensor_position" in implementation
    assert "grid.info.origin.orientation.w = 1.0" in implementation
    assert "x - sensor_position.x" in implementation
    assert "Map projection applied:" in implementation
