from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def test_stationary_startup_does_not_publish_tentative_alignment():
    source = (PACKAGE / "co_3dto2d_mapping" / "stable_inter_robot_place_alignment.py").read_text()
    required = (
        "initial_stationary_alignment_enabled",
        "startup_anchor_pose",
        "initial_required_estimates",
        "estimate_se2_consensus",
        "stationary startup lock",
        "motion_keyframe_allowed",
        "/toy/place_alignment/markers",
    )
    for fragment in required:
        assert fragment in source
    stationary = source[source.index("def _process_stationary_startup") : source.index("def _motion_is_enough")]
    assert "self._publish_alignment()" not in stationary
    assert "self._accept_consensus" in stationary


def test_robust_compositor_has_single_robot_reference_and_unknown_only_seed():
    source = (PACKAGE / "co_3dto2d_mapping" / "record_republisher_robust.py").read_text()
    required = (
        "auto_single_robot_fallback",
        "active_reference_robot",
        "choose_fusion_reference",
        "seed_unknown_observations",
        "Common map reference is now r%d/odom",
        "/toy_record/fusion_status",
        "/toy_record/fusion_markers",
        "odom_in_map",
    )
    for fragment in required:
        assert fragment in source


def test_build_installs_fixed_nodes_and_visualization_dependency():
    cmake = (PACKAGE / "CMakeLists.txt").read_text()
    assert "co_3dto2d_mapping/stable_inter_robot_place_alignment.py" in cmake
    assert "co_3dto2d_mapping/record_republisher_robust.py" in cmake
    assert "RENAME initial_xy_icp_alignment.py" in cmake
    assert "RENAME record_republisher.py" in cmake
    assert "find_package(visualization_msgs REQUIRED)" in cmake

    manifest = ET.parse(PACKAGE / "package.xml").getroot()
    dependencies = {element.text for element in manifest.findall("depend")}
    assert "visualization_msgs" in dependencies


def test_rviz_enables_alignment_and_fusion_debug_views():
    config = yaml.safe_load((PACKAGE / "rviz" / "two_robot_mapping.rviz").read_text())
    displays = config["Visualization Manager"]["Displays"]
    names = {display.get("Name") for display in displays}
    required = {
        "Merged Global Occupancy",
        "R0 Global Occupancy (raw frame)",
        "R1 Global Occupancy (raw frame)",
        "Alignment Debug",
        "Fusion Contributions",
        "R0 Odometry in Map",
        "R1 Odometry in Map",
        "Robot TF",
    }
    assert required <= names
    assert config["Visualization Manager"]["Global Options"]["Fixed Frame"] == "map"
