from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH_DIR = PACKAGE / "launch"
LAUNCH_NAMES = (
    "bag_mid360.launch.py",
    "live_mapping.launch.py",
    "two_live_mapping.launch.py",
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
        "two_live_mapping.launch.py",
        "two_bag_mapping.launch.py",
        "rerun_mapping.launch.py",
    )
    missing = []
    for name in expected:
        path = LAUNCH_DIR / name
        if path.exists() and 'package="co_3dto2d_mapping"' not in path.read_text():
            missing.append(name)
    assert missing == []


def test_two_live_mapping_isolates_robot_topics_and_frames():
    text = (LAUNCH_DIR / "two_live_mapping.launch.py").read_text()
    required = (
        "robot0_lidar_topic",
        "robot1_lidar_topic",
        "robot0_imu_topic",
        "robot1_imu_topic",
        "r%d/base_link",
        "r%d/livox_frame",
        "r%d/odom",
        "pointcloud_frame_republisher.py",
        "initial_xy_icp_alignment.py",
        "record_republisher.py",
        '"publish_tf_odom": "false"',
        "reserved_internal_topics",
        "enable_robot0_pipeline",
        "enable_robot1_pipeline",
        "enable_fusion",
    )
    missing = [token for token in required if token not in text]
    assert missing == []


def test_two_live_laptop_runner_starts_only_its_local_mapping_pipeline():
    text = (PACKAGE / "scripts" / "run_two_mid360_2d_mapping.sh").read_text()
    required = (
        'ENABLE_ROBOT0_PIPELINE="true"',
        'ENABLE_ROBOT1_PIPELINE="true"',
        '"enable_robot0_pipeline:=${ENABLE_ROBOT0_PIPELINE}"',
        '"enable_robot1_pipeline:=${ENABLE_ROBOT1_PIPELINE}"',
        '"enable_fusion:=${RUN_FUSION}"',
        'RUN_LOCAL_MAPPING="${TWO_LIVE_LOCAL_MAPPING:-true}"',
        'EXPECTED_UPDATE_RATE="${EXPECTED_UPDATE_RATE:-11.0}"',
    )
    missing = [fragment for fragment in required if fragment not in text]
    assert missing == []


def test_two_live_pointcloud_republisher_is_installed():
    script = PACKAGE / "co_3dto2d_mapping" / "pointcloud_frame_republisher.py"
    cmake = (PACKAGE / "CMakeLists.txt").read_text()
    assert script.is_file()
    assert "co_3dto2d_mapping/pointcloud_frame_republisher.py" in cmake


def test_two_live_forwards_wait_imu_to_init_to_icp_odometry():
    expected_fragments = {
        "mid360_mapping_pipeline.launch.py": (
            'DeclareLaunchArgument("wait_imu_to_init", default_value="true")',
            '"wait_imu_to_init": LaunchConfiguration("wait_imu_to_init")',
        ),
        "single_bag_mapping.launch.py": (
            'DeclareLaunchArgument("wait_imu_to_init", default_value="true")',
            '"wait_imu_to_init": LaunchConfiguration("wait_imu_to_init")',
        ),
        "live_mapping.launch.py": (
            'DeclareLaunchArgument("wait_imu_to_init", default_value="true")',
            '"wait_imu_to_init": LaunchConfiguration("wait_imu_to_init")',
        ),
        "two_live_mapping.launch.py": (
            'DeclareLaunchArgument("wait_imu_to_init", default_value="true")',
            '"wait_imu_to_init": _value(context, "wait_imu_to_init")',
        ),
    }
    missing = []
    for launch_name, fragments in expected_fragments.items():
        text = (LAUNCH_DIR / launch_name).read_text()
        missing.extend(
            f"{launch_name}: {fragment}" for fragment in fragments if fragment not in text
        )
    assert missing == []
