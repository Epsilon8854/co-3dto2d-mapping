"""Execute the real launch setup with lightweight ROS launch stand-ins.

These tests validate action wiring, not DDS transport or LiDAR hardware. No ROS
installation is needed, so a remote-cloud startup dependency cannot hide behind
a skipped integration test.
"""

import importlib.util
from pathlib import Path
import sys
import types

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH_DIR = PACKAGE / "launch"


class LaunchDescription:
    def __init__(self, entities):
        self.entities = list(entities)


class DeclareLaunchArgument:
    def __init__(self, name, default_value=None, **kwargs):
        self.name = name
        self.default_value = default_value


class LaunchConfiguration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context[self.name]


class OpaqueFunction:
    def __init__(self, function):
        self.function = function


class LogInfo:
    def __init__(self, msg):
        self.msg = msg


class IncludeLaunchDescription:
    def __init__(self, source, launch_arguments):
        self.source = source
        self.launch_arguments = dict(launch_arguments)


class PythonLaunchDescriptionSource:
    def __init__(self, path):
        self.path = path


class Node:
    def __init__(self, **kwargs):
        self.kwargs = kwargs


@pytest.fixture
def load_launch(monkeypatch, tmp_path):
    installed = tmp_path / "share" / "co_3dto2d_mapping"
    (installed / "launch").mkdir(parents=True)
    (installed / "launch" / "two_live_mapping_base.launch.py").write_text(
        (LAUNCH_DIR / "two_live_mapping.launch.py").read_text()
    )
    (installed / "config").mkdir()
    (installed / "config" / "occupancy.yaml").write_text(
        (PACKAGE / "config" / "occupancy.yaml").read_text()
    )
    modules = {
        "ament_index_python": {},
        "ament_index_python.packages": {
            "get_package_share_directory": lambda _: str(installed),
        },
        "launch": {"LaunchDescription": LaunchDescription},
        "launch.actions": {
            "DeclareLaunchArgument": DeclareLaunchArgument,
            "IncludeLaunchDescription": IncludeLaunchDescription,
            "LogInfo": LogInfo,
            "OpaqueFunction": OpaqueFunction,
        },
        "launch.launch_description_sources": {
            "PythonLaunchDescriptionSource": PythonLaunchDescriptionSource,
        },
        "launch.substitutions": {"LaunchConfiguration": LaunchConfiguration},
        "launch_ros": {},
        "launch_ros.actions": {"Node": Node},
    }
    for name, attributes in modules.items():
        module = types.ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)

    def load(public=True, **overrides):
        filename = ("two_live_plane_height_mapping.launch.py" if public
                    else "two_live_mapping.launch.py")
        spec = importlib.util.spec_from_file_location("launch_under_test", LAUNCH_DIR / filename)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        description = module.generate_launch_description()
        context = {
            action.name: action.default_value
            for action in description.entities
            if isinstance(action, DeclareLaunchArgument)
        }
        context.update(overrides)
        actions = []
        for action in description.entities:
            if isinstance(action, OpaqueFunction):
                actions.extend(action.function(context) or [])
        return actions, context

    return load


def nodes(actions, executable):
    return [action for action in actions
            if isinstance(action, Node) and action.kwargs["executable"] == executable]


def assert_map_only(actions):
    aligners = nodes(actions, "inter_robot_place_alignment.py")
    assert len(aligners) == 1
    settings = aligners[0].kwargs["parameters"][-1]
    assert settings["robot0_map_topic"] == "/r0/toy/global_occupancy"
    assert settings["robot1_map_topic"] == "/r1/toy/global_occupancy"
    assert settings["robot0_odom_topic"] == "/r0/toy/corrected_odometry"
    assert settings["robot1_odom_topic"] == "/r1/toy/corrected_odometry"
    assert settings["alignment_topic"] == "/toy/initial_xy_alignment"
    assert settings["target_frame_id"] == "map"
    assert settings["source_frame_id"] == "r1/odom"
    assert not any("cloud" in name for name in settings)
    assert "input_mode" not in settings
    assert not nodes(actions, "initial_xy_icp_alignment.py")
    assert not nodes(actions, "cropped_xyz_initial_icp_alignment.py")
    # A process gate or event-handler wrapper would violate this action contract.
    assert all(isinstance(action, (Node, IncludeLaunchDescription, LogInfo))
               for action in actions)
    return settings


@pytest.mark.parametrize("public", [False, True])
def test_both_entry_points_use_2d_maps_without_startup_gate(load_launch, public):
    actions, context = load_launch(
        public=public, enable_place_recognition="true"
    )
    assert_map_only(actions)
    pipelines = [a for a in actions if isinstance(a, IncludeLaunchDescription)]
    assert len(pipelines) == 2
    assert {p.launch_arguments["robot_id"] for p in pipelines} == {"0", "1"}
    assert len(nodes(actions, "record_republisher.py")) == 1
    for pipeline in pipelines:
        arguments = pipeline.launch_arguments
        rid = arguments["robot_id"]
        assert arguments["global_frame_id"] == f"r{rid}/odom"
        assert arguments["sensor_parent_frame"] == f"r{rid}/base_link"
        assert arguments["sensor_child_frame"] == f"r{rid}/livox_frame"
        assert arguments["publish_sensor_static_tf"] == "true"
        assert arguments["mapping_startup_delay_sec"] == "10.0"
        assert arguments["wait_imu_to_init"] == "true"
        assert "wait_for_initial_alignment" not in arguments


@pytest.mark.parametrize("rid", [0, 1])
def test_peer_starts_without_a_remote_robot_or_fusion_host(load_launch, rid):
    actions, _ = load_launch(**{
        "enable_robot0_pipeline": str(rid == 0).lower(),
        "enable_robot1_pipeline": str(rid == 1).lower(),
        "enable_fusion": "false",
    })
    assert len(actions) == 2
    pipeline, relay = actions
    assert isinstance(pipeline, IncludeLaunchDescription)
    assert pipeline.launch_arguments["robot_id"] == str(rid)
    assert relay.kwargs["parameters"][0]["input_topic"] == f"/r{rid}/livox/lidar"
    assert not nodes(actions, "inter_robot_place_alignment.py")
    assert not nodes(actions, "record_republisher.py")


@pytest.mark.parametrize("rid", [0, 1])
def test_distributed_fusion_only_reads_its_own_sensor(load_launch, rid):
    actions, _ = load_launch(**{
        "enable_robot0_pipeline": str(rid == 0).lower(),
        "enable_robot1_pipeline": str(rid == 1).lower(),
        "enable_place_recognition": "true",
    })
    assert_map_only(actions)
    relays = nodes(actions, "pointcloud_frame_republisher.py")
    assert len(relays) == 1
    assert relays[0].kwargs["parameters"][0]["input_topic"] == f"/r{rid}/livox/lidar"


def test_fusion_only_host_does_not_create_sensor_consumers(load_launch):
    actions, _ = load_launch(
        enable_robot0_pipeline="false",
        enable_robot1_pipeline="false",
        enable_place_recognition="true",
    )
    assert_map_only(actions)
    assert not nodes(actions, "pointcloud_frame_republisher.py")
    assert not any(isinstance(a, IncludeLaunchDescription) for a in actions)


def test_place_recognition_false_does_not_start_an_alignment_node(
    load_launch, monkeypatch
):
    monkeypatch.setenv("CO3DTO2D_STARTUP_DIRECT_LIDAR", "true")
    actions, _ = load_launch(
        wait_for_initial_alignment="true",
        startup_alignment_timeout_sec="0.0",
        enable_place_recognition="false",
    )
    assert not nodes(actions, "inter_robot_place_alignment.py")
    assert not nodes(actions, "initial_xy_icp_alignment.py")
    assert len([a for a in actions if isinstance(a, IncludeLaunchDescription)]) == 2
    notices = " ".join(a.msg for a in actions if isinstance(a, LogInfo))
    assert "wait_for_initial_alignment is deprecated" in notices
    assert "enable_place_recognition:=false is deprecated" not in notices


def test_place_recognition_defaults_to_disabled(load_launch):
    actions, context = load_launch()

    assert context["enable_place_recognition"] == "false"
    assert not nodes(actions, "inter_robot_place_alignment.py")


def test_profile_precedes_frame_contract_and_preserves_consensus(load_launch):
    actions, _ = load_launch(
        alignment_config_file="/profiles/strict.yaml",
        enable_place_recognition="true",
    )
    settings = assert_map_only(actions)
    parameters = nodes(actions, "inter_robot_place_alignment.py")[0].kwargs["parameters"]
    assert parameters[0].endswith("/config/place_recognition.yaml")
    assert parameters[1] == "/profiles/strict.yaml"
    assert settings["lock_after_consensus"] is False
    assert settings["stop_processing_after_lock"] is False
    assert "consensus_min_measurements" not in settings
    assert "registration_min_symmetric_overlap" not in settings


def test_enabled_place_recognition_updates_alignment_and_merge_continuously(
    load_launch,
):
    actions, context = load_launch(enable_place_recognition="true")

    alignment = nodes(actions, "inter_robot_place_alignment.py")[0]
    alignment_settings = alignment.kwargs["parameters"][-1]
    record = nodes(actions, "record_republisher.py")[0]
    record_settings = record.kwargs["parameters"][0]
    assert context["alignment_lock_after_first"] == "false"
    assert alignment_settings["lock_after_consensus"] is False
    assert alignment_settings["stop_processing_after_lock"] is False
    assert record_settings["lock_world_alignment"] is False


def test_disable_record_does_not_disable_map_alignment(load_launch):
    actions, _ = load_launch(
        enable_record_republisher="false",
        enable_place_recognition="true",
    )
    assert_map_only(actions)
    assert not nodes(actions, "record_republisher.py")


@pytest.mark.parametrize("overrides", [
    {"robot1_lidar_topic": "/r0/livox/lidar"},
    {"robot1_imu_topic": "/r0/livox/imu"},
    {"robot1_imu_topic": "/r0/livox/lidar"},
    {"robot0_lidar_topic": "relative/lidar"},
    {"robot0_lidar_topic": "/r1/mapping/lidar"},
    {"robot0_imu_topic": "/r1/mapping/imu_filtered"},
    {"enable_robot0_pipeline": "false", "enable_robot1_pipeline": "false", "enable_fusion": "false"},
    {"alignment_startup_delay_sec": "-1"},
    {"alignment_startup_delay_sec": "nan"},
    {"enable_fusion": "invalid"},
    {"wait_for_initial_alignment": "invalid"},
])
def test_input_validation_is_preserved(load_launch, overrides):
    with pytest.raises(RuntimeError):
        load_launch(**overrides)


def test_local_warmup_is_configurable_and_not_an_alignment_barrier(load_launch):
    actions, _ = load_launch(
        mapping_startup_delay_sec="0.0",
        enable_place_recognition="true",
    )
    assert_map_only(actions)
    for pipeline in (a for a in actions if isinstance(a, IncludeLaunchDescription)):
        assert pipeline.launch_arguments["mapping_startup_delay_sec"] == "0.0"
