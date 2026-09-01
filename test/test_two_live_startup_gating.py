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


def test_two_live_alignment_waits_for_maps_and_requires_stability():
    two_live = (LAUNCH_DIR / "two_live_mapping.launch.py").read_text()
    required = (
        '"alignment_startup_delay_sec"',
        'default_value="3.0"',
        '"alignment_required_consistent_results"',
        'default_value="2"',
        '"alignment_lock_after_first"',
        '"alignment_initialize_from_centroids"',
    )
    for fragment in required:
        assert fragment in two_live

    aligner = (
        PACKAGE / "co_3dto2d_mapping" / "initial_xy_icp_alignment.py"
    ).read_text()
    required_aligner_fragments = (
        "Both ICP inputs are available",
        "required_consistent_results",
        "candidate_is_consistent",
        "lock_after_first_alignment",
        "Collecting a fresh pair",
        'initializations.append(("centroid", centroid_translation))',
    )
    for fragment in required_aligner_fragments:
        assert fragment in aligner
