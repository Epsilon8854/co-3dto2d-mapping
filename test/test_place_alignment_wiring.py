from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]


def test_public_alignment_executable_uses_place_recognition():
    cmake = (PACKAGE / "CMakeLists.txt").read_text()
    assert "co_3dto2d_mapping/inter_robot_place_alignment.py" in cmake
    assert "RENAME initial_xy_icp_alignment.py" in cmake
    assert "RENAME legacy_initial_xy_icp_alignment.py" in cmake


def test_place_alignment_uses_existing_runtime_topics_and_cpu_core():
    node = (
        PACKAGE / "co_3dto2d_mapping" / "inter_robot_place_alignment.py"
    ).read_text()
    required = (
        '"/r0/toy/global_occupancy"',
        '"/r1/toy/global_occupancy"',
        '"/r0/toy/corrected_odometry"',
        '"/r1/toy/corrected_odometry"',
        '"/toy/initial_xy_alignment"',
        '"/toy/inter_robot_relative_transform"',
        "match_polar_descriptors",
        "register_submaps",
        "map_alignment_from_keyframes",
        "estimate_se2_consensus",
    )
    for fragment in required:
        assert fragment in node


def test_std_msgs_runtime_dependency_is_declared():
    manifest = ET.parse(PACKAGE / "package.xml").getroot()
    dependencies = {element.text for element in manifest.findall("depend")}
    assert "std_msgs" in dependencies
