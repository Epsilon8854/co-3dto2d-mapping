import importlib.util
from pathlib import Path
import subprocess
import sys
import types

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


class LaunchDescription:
    def __init__(self, entities):
        self.entities = list(entities)


class DeclareLaunchArgument:
    def __init__(self, name, default_value=None, **_kwargs):
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


class IncludeLaunchDescription:
    def __init__(self, source, launch_arguments):
        self.source = source
        self.launch_arguments = dict(launch_arguments)


class PythonLaunchDescriptionSource:
    def __init__(self, path):
        self.path = path


class TimerAction:
    def __init__(self, period, actions):
        self.period = period
        self.actions = list(actions)


class Node:
    def __init__(self, **kwargs):
        self.kwargs = kwargs


def load_launch(monkeypatch, filename, **overrides):
    modules = {
        "ament_index_python": {},
        "ament_index_python.packages": {
            "get_package_share_directory": lambda _: str(PACKAGE),
        },
        "launch": {"LaunchDescription": LaunchDescription},
        "launch.actions": {
            "DeclareLaunchArgument": DeclareLaunchArgument,
            "IncludeLaunchDescription": IncludeLaunchDescription,
            "OpaqueFunction": OpaqueFunction,
            "TimerAction": TimerAction,
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

    spec = importlib.util.spec_from_file_location(
        "lidar_odom_launch_under_test", PACKAGE / "launch" / filename
    )
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
        else:
            actions.append(action)
    return actions, context


def includes(actions):
    return [action for action in actions if isinstance(action, IncludeLaunchDescription)]


def test_default_lidar_odometry_profile_contains_tunable_icp_parameters():
    profile_path = PACKAGE / "config" / "lidar_odometry.yaml"

    assert profile_path.exists()
    parameters = yaml.safe_load(profile_path.read_text())["/**"]["ros__parameters"]
    assert parameters["Icp/VoxelSize"] == "0.05"
    assert parameters["Icp/Iterations"] == "10"
    assert parameters["Icp/MaxCorrespondenceDistance"] == "1"
    assert parameters["OdomF2M/ScanMaxSize"] == "15000"


def test_rtabmap_odometry_loads_profile_before_runtime_overrides(
    monkeypatch, tmp_path
):
    profile = tmp_path / "robot1_odom.yaml"
    profile.write_text("/**:\n  ros__parameters: {}\n")
    actions, _ = load_launch(
        monkeypatch,
        "rtabmap_mid360_odometry.launch.py",
        odometry_config_file=str(profile),
        expected_update_rate="17.0",
    )
    odometry = next(action for action in actions if isinstance(action, Node))

    assert odometry.kwargs["parameters"][0] == str(profile)
    runtime = odometry.kwargs["parameters"][1]
    assert runtime["expected_update_rate"] == 17.0
    assert "Icp/VoxelSize" not in runtime
    assert "OdomF2M/ScanMaxSize" not in runtime


def test_profile_path_propagates_through_each_mapping_launch(monkeypatch):
    profile = "/profiles/robot1_odom.yaml"
    for filename in (
        "live_mapping.launch.py",
        "single_bag_mapping.launch.py",
        "mid360_mapping_pipeline.launch.py",
    ):
        actions, context = load_launch(
            monkeypatch,
            filename,
            odometry_config_file=profile,
        )
        matching = [
            action
            for action in includes(actions)
            if "odometry_config_file" in action.launch_arguments
        ]
        assert len(matching) == 1
        include = matching[0]
        value = include.launch_arguments["odometry_config_file"]
        if isinstance(value, LaunchConfiguration):
            value = value.perform(context)
        assert value == profile


def test_physical_runner_exposes_odometry_profile_option():
    result = subprocess.run(
        ["bash", str(PACKAGE / "scripts" / "run_two_mid360_2d_mapping.sh"), "--help"],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    assert "--odometry-config FILE" in result.stdout
