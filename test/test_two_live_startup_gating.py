from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH_DIR = PACKAGE / "launch"


def test_two_live_mode_has_a_sensor_warmup_before_mapping():
    two_live = (LAUNCH_DIR / "two_live_mapping.launch.py").read_text()
    assert '"mapping_startup_delay_sec"' in two_live
    assert 'default_value="10.0"' in two_live

    forwarding = {
        "live_mapping.launch.py": '"mapping_startup_delay_sec": LaunchConfiguration(',
        "single_bag_mapping.launch.py": '"mapping_startup_delay_sec": str(mapping_startup_delay_sec)',
        "mid360_mapping_pipeline.launch.py": '"startup_delay_sec": LaunchConfiguration("mapping_startup_delay_sec")',
    }
    for filename, fragment in forwarding.items():
        assert fragment in (LAUNCH_DIR / filename).read_text()

    odometry = (LAUNCH_DIR / "rtabmap_mid360_odometry.launch.py").read_text()
    assert "TimerAction(period=startup_delay_sec" in odometry


def test_public_two_live_waits_for_actual_startup_icp_before_odom_mapping():
    public_wrapper = (
        LAUNCH_DIR / "two_live_plane_height_mapping.launch.py"
    ).read_text()

    required_fragments = (
        '"wait_for_initial_alignment"',
        'default_value="true"',
        '"startup_alignment_topic"',
        'default_value="/toy/startup_xy_alignment"',
        'name="startup_initial_xy_icp_alignment"',
        '"robot0_cloud_topic": _prealignment_cloud_topic(context, 0)',
        '"robot1_cloud_topic": _prealignment_cloud_topic(context, 1)',
        'parameters=[occupancy_config_file, overrides]',
        '"use_z_filter": False',
        '"invert_z_slice": False',
        'occupancy, "center_box_filter_half_extent_m", 0.80',
        'occupancy, "range_min_m", 0.80',
        'occupancy, "range_max_m", 12.0',
        '"ros2",',
        '"topic",',
        '"echo",',
        '"--once",',
        'OnProcessExit(',
        'odometry/mapping has not started.',
        'starting odometry/mapping now.',
        'inter_robot_place_alignment.py',
    )
    for fragment in required_fragments:
        assert fragment in public_wrapper

    cmake = (PACKAGE / "CMakeLists.txt").read_text()
    assert "launch/two_live_plane_height_mapping.launch.py" in cmake
    assert "RENAME two_live_mapping.launch.py" in cmake


def test_two_live_alignment_uses_cropped_xyz_rtabmap_clouds():
    two_live = (LAUNCH_DIR / "two_live_mapping.launch.py").read_text()
    required_launch_fragments = (
        '"robot0_cloud_topic": robot0_scan_topic',
        '"robot1_cloud_topic": robot1_scan_topic',
        '"input_mode": "cloud_initial"',
        '"robot0_local_frame_id": "r0/base_link"',
        '"robot1_local_frame_id": "r1/base_link"',
        '"alignment_use_z_filter"',
        '"alignment_range_min_m"',
        '"alignment_range_max_m"',
        '"alignment_enforce_tilt_prior"',
        '"alignment_required_consistent_results"',
        'default_value="2"',
        '"alignment_lock_after_first"',
        '"alignment_initialize_from_centroids"',
    )
    for fragment in required_launch_fragments:
        assert fragment in two_live

    aligner = (
        PACKAGE
        / "co_3dto2d_mapping"
        / "cropped_xyz_initial_icp_alignment.py"
    ).read_text()
    required_aligner_fragments = (
        "Cropped XYZ startup ICP",
        "robot0_local_frame_id",
        "slice_z_in_cloud_frame",
        "range_min_m",
        "estimate_rigid_transform",
        "Collecting a fresh pair",
        "published_planar",
        "max_tilt_deviation_rad",
    )
    for fragment in required_aligner_fragments:
        assert fragment in aligner

    registration = (
        PACKAGE / "co_3dto2d_mapping" / "pointcloud_registration.py"
    ).read_text()
    for fragment in (
        "estimate_rigid_transform",
        "yaw_rotation_matrix",
        "rotation_tilt",
        "voxel_downsample",
    ):
        assert fragment in registration

    cmake = (PACKAGE / "CMakeLists.txt").read_text()
    assert "co_3dto2d_mapping/cropped_xyz_initial_icp_alignment.py" in cmake
