from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH_DIR = PACKAGE / "launch"
LAUNCH_NAMES = (
    "bag_mid360.launch.py",
    "rtabmap_mid360_odometry.launch.py",
    "mid360_mapping_pipeline.launch.py",
    "single_bag_mapping.launch.py",
    "two_bag_mapping.launch.py",
    "rerun_mapping.launch.py",
)
FORBIDDEN = (
    "cslam_experiments",
    "cslam_visualization",
    "enable_cslam",
    "cslam_config_file",
    "/home/user/Swarm-SLAM",
)


def test_mapping_launch_files_exist():
    missing = [name for name in LAUNCH_NAMES if not (LAUNCH_DIR / name).is_file()]
    assert missing == []


def test_mapping_launch_files_have_no_forbidden_dependencies():
    violations = []
    for name in LAUNCH_NAMES:
        path = LAUNCH_DIR / name
        if not path.exists():
            continue
        text = path.read_text()
        for token in FORBIDDEN:
            if token in text:
                violations.append(f"{name}: {token}")
    assert violations == []


def test_mapping_nodes_use_standalone_package_name():
    expected = (
        "mid360_mapping_pipeline.launch.py",
        "single_bag_mapping.launch.py",
        "two_bag_mapping.launch.py",
        "rerun_mapping.launch.py",
    )
    missing = []
    for name in expected:
        path = LAUNCH_DIR / name
        if path.exists() and 'package="co_3dto2d_mapping"' not in path.read_text():
            missing.append(name)
    assert missing == []
