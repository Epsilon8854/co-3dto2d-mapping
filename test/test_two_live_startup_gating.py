from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH_DIR = PACKAGE / "launch"


def test_initial_alignment_uses_same_occupancy_yaml_without_inverse_z():
    profile = yaml.safe_load((PACKAGE / "config" / "occupancy.yaml").read_text())
    alignment = profile["/initial_xy_icp_alignment"]["ros__parameters"]

    assert alignment["use_z_filter"] is False
    assert alignment["invert_z_slice"] is False
    assert alignment["z_min"] == -1000.0
    assert alignment["z_max"] == 1000.0
    assert alignment["center_box_half_extent_m"] == 0.80
    assert alignment["range_min_m"] == 0.80
    assert alignment["range_max_m"] == 12.0
    assert alignment["voxel_size"] == 0.10
    assert alignment["required_consistent_results"] >= 2

    launch = (LAUNCH_DIR / "initial_alignment_live.launch.py").read_text()
    assert 'parameters=[\n                config_file,' in launch
    assert '"use_z_filter": False' in launch
    assert '"invert_z_slice": False' in launch
    assert '"robot0_cloud_topic": _value(context, "robot0_cloud_topic")' in launch
    assert '"robot1_cloud_topic": _value(context, "robot1_cloud_topic")' in launch
    assert '"robot0_local_frame_id": "r0/base_link"' in launch
    assert '"robot1_local_frame_id": "r1/base_link"' in launch


def test_public_two_live_wrapper_removes_legacy_inline_alignment_filters():
    wrapper = (LAUNCH_DIR / "two_live_plane_height_mapping.launch.py").read_text()

    assert "_config_aware_launch_setup" in wrapper
    assert "_configured_initial_alignment_node" in wrapper
    assert "_configured_launch_argument" in wrapper
    assert '"alignment_use_z_filter": "false"' in wrapper
    assert '"alignment_z_min": "-1000.0"' in wrapper
    assert '"alignment_z_max": "1000.0"' in wrapper
    assert '"alignment_invert_z_slice": "false"' in wrapper
    assert '"use_z_filter"' in wrapper
    assert '"invert_z_slice"' in wrapper
    assert 'rewritten["parameters"] = [config_file, runtime_overrides]' in wrapper
    assert "_place_recognition_node" not in wrapper
    assert "inter_robot_place_alignment.py" not in wrapper


def test_two_live_runner_blocks_odom_until_initial_alignment_message():
    runner = (PACKAGE / "scripts" / "run_two_mid360_2d_mapping.sh").read_text()

    assert 'WAIT_FOR_INITIAL_ALIGNMENT="${TWO_LIVE_WAIT_FOR_INITIAL_ALIGNMENT:-true}"' in runner
    assert "initial_alignment_live.launch.py" in runner
    assert "ODOM/MAPPING BLOCKED" in runner
    assert "--qos-durability transient_local" in runner
    assert "geometry_msgs/msg/TransformStamped" in runner
    assert "INITIAL ALIGNMENT READY" in runner
    assert 'MAPPING_STARTUP_DELAY_SEC=0.0' in runner
    assert 'MAPPING_ENABLE_FUSION=false' in runner
    assert '"enable_fusion:=${MAPPING_ENABLE_FUSION}"' in runner
    assert '"mapping_startup_delay_sec:=${MAPPING_STARTUP_DELAY_SEC}"' in runner
    assert "post_alignment_fusion.launch.py" in runner

    # The blocking topic wait must occur textually before the mapping process is
    # assembled and started, preventing misleading RTAB-Map startup logs.
    wait_index = runner.index('alignment_wait_command=(')
    ready_index = runner.index('INITIAL ALIGNMENT READY')
    mapping_index = runner.index('mapping_command=(')
    assert wait_index < ready_index < mapping_index


def test_initial_alignment_launch_starts_no_odometry_or_mapper():
    launch = (LAUNCH_DIR / "initial_alignment_live.launch.py").read_text()
    assert 'executable="initial_xy_icp_alignment.py"' in launch
    assert "rtabmap_odom" not in launch
    assert "occupancy_mapper" not in launch
    assert "initial_alignment_sensor_static_tf_r%d" in launch


def test_post_alignment_launch_contains_only_common_frame_fusion():
    launch = (LAUNCH_DIR / "post_alignment_fusion.launch.py").read_text()
    assert 'executable="record_republisher.py"' in launch
    assert '"publish_world_odometry": True' in launch
    assert '"publish_world_maps": True' in launch
    assert "initial_xy_icp_alignment.py" not in launch
    assert "rtabmap_odom" not in launch


def test_cropped_xyz_aligner_retains_full_3d_registration():
    aligner = (
        PACKAGE / "co_3dto2d_mapping" / "cropped_xyz_initial_icp_alignment.py"
    ).read_text()
    for fragment in (
        "Cropped XYZ startup ICP",
        "robot0_local_frame_id",
        "use_z_filter",
        "estimate_rigid_transform",
        "Collecting a fresh pair",
        "published_planar",
        "max_tilt_deviation_rad",
    ):
        assert fragment in aligner
