"""Config-to-launch and real temporal-callback regressions, without ROS/DDS.

Launch/ROS service objects are stand-ins; the configuration loader, launch_setup
functions, TemporalToyRecordRepublisher callbacks and fusion engine are real.
"""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS

import pytest
import yaml

from co_3dto2d_mapping.dynamic_filter_config import merged_dynamic_parameters
from co_3dto2d_mapping.temporal_fusion import (
    FREE, OCCUPIED, UNKNOWN, TemporalFusionConfig, TemporalFusionGrid,
    expand_free_observations,
)

ROOT = Path(__file__).resolve().parents[1]
LAUNCHES = (
    "two_live_mapping.launch.py",
    "two_live_combined_bag_mapping.launch.py",
    "two_bag_mapping.launch.py",
)


def write_config(tmp_path, **params):
    path = tmp_path / "occupancy with spaces.yaml"
    path.write_text(yaml.safe_dump({"/**": {"ros__parameters": params}}))
    return str(path)


@pytest.mark.parametrize("enabled", [False, True])
def test_single_switch_overrides_obsolete_merged_alias(tmp_path, enabled):
    path = write_config(
        tmp_path, dynamic_filter_enabled=enabled,
        merged_temporal_filter_enabled=not enabled,
        dynamic_free_clear_count=10, dynamic_occupied_confirm_count=1,
        merged_dynamic_free_clear_count=99,
    )
    result = merged_dynamic_parameters(path)
    assert result["merged_temporal_filter_enabled"] is enabled
    assert result["merged_dynamic_free_clear_count"] == 10
    assert result["merged_dynamic_occupied_confirm_count"] == 1
    assert result["merged_free_observation_inflation_m"] == 0.0


def test_missing_optional_values_match_mapper_defaults(tmp_path):
    result = merged_dynamic_parameters(write_config(tmp_path))
    assert result["merged_temporal_filter_enabled"] is False
    assert result["merged_dynamic_free_clear_count"] == 4
    assert result["merged_dynamic_occupied_confirm_count"] == 3
    assert result["merged_dynamic_counter_decay"] == 1
    assert result["merged_dynamic_evidence_timeout_frames"] == 30


def test_repository_config_is_forwarded_without_retuning():
    path = ROOT / "config" / "occupancy.yaml"
    raw = yaml.safe_load(path.read_text())["/**"]["ros__parameters"]
    result = merged_dynamic_parameters(str(path))
    assert result["merged_temporal_filter_enabled"] == raw["dynamic_filter_enabled"]
    for suffix in ("free_clear_count", "occupied_confirm_count", "counter_decay", "evidence_timeout_frames"):
        assert result["merged_dynamic_" + suffix] == raw["dynamic_" + suffix]


@pytest.mark.parametrize("key,value", [
    ("dynamic_filter_enabled", "false"),
    ("dynamic_filter_enabled", 1),
    ("enable_raycast_free_space", "true"),
    ("dynamic_free_clear_count", 0),
    ("dynamic_free_clear_count", 65536),
    ("dynamic_free_clear_count", True),
    ("dynamic_occupied_confirm_count", 1.5),
    ("dynamic_counter_decay", -1),
    ("dynamic_evidence_timeout_frames", 4294967296),
    ("merged_free_observation_inflation_m", float("nan")),
    ("merged_free_observation_inflation_m", float("inf")),
    ("merged_alignment_reset_yaw_deg", -1.0),
])
def test_bad_settings_fail_explicitly(tmp_path, key, value):
    with pytest.raises(ValueError, match=key):
        merged_dynamic_parameters(write_config(tmp_path, **{key: value}))


@pytest.mark.parametrize("text", ["", "[]", "null", "/**: null", "/**: {ros__parameters: []}", "[broken"])
def test_bad_yaml_structure_is_not_silently_disabled(tmp_path, text):
    path = tmp_path / "bad.yaml"
    path.write_text(text)
    with pytest.raises(ValueError):
        merged_dynamic_parameters(str(path))


def test_missing_config_is_not_silently_disabled(tmp_path):
    with pytest.raises(ValueError, match="Cannot read"):
        merged_dynamic_parameters(str(tmp_path / "missing.yaml"))


def test_enabled_filter_requires_free_rays(tmp_path):
    with pytest.raises(ValueError, match="requires enable_raycast_free_space"):
        merged_dynamic_parameters(write_config(
            tmp_path, dynamic_filter_enabled=True, enable_raycast_free_space=False,
        ))
    assert not merged_dynamic_parameters(write_config(
        tmp_path, dynamic_filter_enabled=False, enable_raycast_free_space=False,
    ))["merged_temporal_filter_enabled"]


def test_unrelated_occupancy_settings_do_not_leak_into_record_node(tmp_path):
    result = merged_dynamic_parameters(write_config(
        tmp_path, dynamic_filter_enabled=True, scan_cloud_topic="/private/lidar",
        robot_ids=[9], output_prefix="/wrong", publish_period_ms=99,
    ))
    assert not {"scan_cloud_topic", "robot_ids", "output_prefix", "publish_period_ms"} & result.keys()


def install_module(monkeypatch, name, **attrs):
    module = ModuleType(name)
    module.__dict__.update(attrs)
    monkeypatch.setitem(sys.modules, name, module)
    return module


class Action:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs
        if "launch_arguments" in kwargs:
            self.kwargs["launch_arguments"] = dict(kwargs["launch_arguments"])


class Declare(Action):
    pass


class LaunchDescription:
    def __init__(self, entities):
        self.entities = entities


class Configuration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context[self.name]


def load_launch(monkeypatch, filename):
    install_module(monkeypatch, "ament_index_python")
    install_module(monkeypatch, "ament_index_python.packages",
                   get_package_share_directory=lambda name: str(ROOT))
    install_module(monkeypatch, "launch", LaunchDescription=LaunchDescription)
    install_module(monkeypatch, "launch.actions", DeclareLaunchArgument=Declare,
                   IncludeLaunchDescription=Action, LogInfo=Action,
                   OpaqueFunction=Action, TimerAction=Action)
    install_module(monkeypatch, "launch.launch_description_sources", PythonLaunchDescriptionSource=Action)
    install_module(monkeypatch, "launch.substitutions", LaunchConfiguration=Configuration)
    install_module(monkeypatch, "launch_ros")
    install_module(monkeypatch, "launch_ros.actions", Node=Action)
    spec = importlib.util.spec_from_file_location("_dynamic_launch", ROOT / "launch" / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = {
        entity.args[0]: entity.kwargs["default_value"]
        for entity in module.generate_launch_description().entities
        if isinstance(entity, Declare)
    }
    return module, context


def flatten(actions):
    for action in actions:
        yield action
        yield from flatten(action.kwargs.get("actions", []))


@pytest.mark.parametrize("filename", LAUNCHES)
@pytest.mark.parametrize("enabled", [False, True])
def test_launch_passes_switch_and_counts_to_merger_and_same_yaml_to_robots(monkeypatch, tmp_path, filename, enabled):
    module, context = load_launch(monkeypatch, filename)
    context["occupancy_config_file"] = write_config(
        tmp_path, dynamic_filter_enabled=enabled, dynamic_free_clear_count=10,
        dynamic_occupied_confirm_count=1, dynamic_counter_decay=2,
        dynamic_evidence_timeout_frames=45,
    )
    actions = list(flatten(module.launch_setup(context)))
    records = [a for a in actions if a.kwargs.get("executable") == "record_republisher.py"]
    assert len(records) == 1
    params = {}
    for block in records[0].kwargs["parameters"]:
        params.update(block)
    assert params["merged_temporal_filter_enabled"] is enabled
    assert params["merged_dynamic_free_clear_count"] == 10
    assert params["merged_dynamic_occupied_confirm_count"] == 1
    assert params["merged_dynamic_counter_decay"] == 2
    assert params["merged_dynamic_evidence_timeout_frames"] == 45
    assert params["output_prefix"] == "/toy_record"
    robot_includes = [a.kwargs["launch_arguments"] for a in actions if "launch_arguments" in a.kwargs]
    assert len(robot_includes) == 2
    assert {a["robot_id"] for a in robot_includes} == {"0", "1"}
    assert all(a["occupancy_config_file"] == context["occupancy_config_file"] for a in robot_includes)
    if filename == "two_live_mapping.launch.py":
        assert not any("initial_xy_icp_alignment" in a.kwargs.get("executable", "") for a in actions)


@pytest.mark.parametrize("robot_id", [0, 1])
@pytest.mark.parametrize("fusion", [False, True])
def test_distributed_roles_still_start_only_the_local_robot(monkeypatch, tmp_path, robot_id, fusion):
    module, context = load_launch(monkeypatch, LAUNCHES[0])
    context.update({
        "enable_robot0_pipeline": str(robot_id == 0).lower(),
        "enable_robot1_pipeline": str(robot_id == 1).lower(),
        "enable_fusion": str(fusion).lower(),
        "occupancy_config_file": write_config(tmp_path, dynamic_filter_enabled=True),
    })
    actions = list(flatten(module.launch_setup(context)))
    includes = [a for a in actions if "launch_arguments" in a.kwargs]
    assert len(includes) == 1
    assert includes[0].kwargs["launch_arguments"]["robot_id"] == str(robot_id)
    assert sum(a.kwargs.get("executable") == "record_republisher.py" for a in actions) == int(fusion)


def cell_value(grid, col=0, row=0):
    return int(grid.data[row - grid.origin_row, col - grid.origin_col])


def test_late_global_bootstrap_cannot_resurrect_a_live_cleared_cell():
    grid = TemporalFusionGrid(1.0, TemporalFusionConfig(free_clear_count=10), 0.0)
    grid.seed([0], [0], [OCCUPIED])
    for _ in range(10):
        grid.observe([0], [0], [FREE])
    grid.seed([0, 5], [0, 0], [OCCUPIED, OCCUPIED])
    assert cell_value(grid) == FREE
    assert cell_value(grid, 5) == OCCUPIED  # New, unobserved area still seeds.
    assert grid.free_counts[0, 0] == 10


def test_bootstrap_conflicts_still_use_occupied_priority_before_live_observation():
    grid = TemporalFusionGrid(1.0, padding_m=0.0)
    grid.seed([0], [0], [FREE])
    grid.seed([0], [0], [OCCUPIED])
    assert cell_value(grid) == OCCUPIED


@pytest.mark.parametrize("radius", [0, 1])
def test_unknown_is_not_a_free_observation(radius):
    grid = TemporalFusionGrid(1.0, padding_m=0.0)
    grid.seed([0], [0], [OCCUPIED])
    for _ in range(20):
        grid.observe(*expand_free_observations([0], [0], [UNKNOWN], radius))
    grid.observe([0], [0], [UNKNOWN])
    assert cell_value(grid) == OCCUPIED
    assert grid.frame_index == 0


class GridMessage:
    def __init__(self):
        self.header = NS(stamp=NS(sec=0, nanosec=0), frame_id="r0/odom")
        self.info = NS(width=1, height=1, resolution=1.0, origin=NS(
            position=NS(x=0.0, y=0.0, z=0.0),
            orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0),
        ))
        self.data = [UNKNOWN]


def grid_message(value, stamp):
    msg = GridMessage()
    msg.data = [value]
    msg.header.stamp.sec = stamp
    return msg


@pytest.fixture
def record_factory(monkeypatch, tmp_path):
    overrides = {}

    class BaseRecord:
        """Only ROS parameter/clock/log services and superclass caches are fake."""
        def __init__(self):
            self.params = dict(overrides)
            self.alignment = (0.0, 0.0, 0.0)
            self.latest_global_maps, self.latest_local_maps = {}, {}
            self.occupied_threshold, self.merged_padding_m = 50, 0.0
            self.common_frame_id = "map"

        def declare_parameter(self, name, default):
            self.params.setdefault(name, default)

        def get_parameter(self, name):
            return NS(value=self.params[name])

        def get_logger(self):
            return NS(info=lambda *a: None, warning=lambda *a: None)

        def get_clock(self):
            return NS(now=lambda: NS(nanoseconds=10**10))

        def global_map_callback(self, msg, rid):
            self.latest_global_maps[rid] = msg

        def local_map_callback(self, msg, rid):
            self.latest_local_maps[rid] = msg

        def alignment_callback(self, msg):
            self.alignment = msg

        def build_merged_global(self, stamp):
            return "baseline_union"

    install_module(monkeypatch, "rclpy")
    install_module(monkeypatch, "nav_msgs")
    install_module(monkeypatch, "nav_msgs.msg", OccupancyGrid=GridMessage)
    install_module(monkeypatch, "co_3dto2d_mapping.record_republisher", ToyRecordRepublisher=BaseRecord)
    spec = importlib.util.spec_from_file_location(
        "_dynamic_record", ROOT / "co_3dto2d_mapping" / "record_republisher_temporal.py",
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    def create(enabled=True, **settings):
        overrides.clear()
        overrides.update(merged_dynamic_parameters(write_config(
            tmp_path, dynamic_filter_enabled=enabled, **settings,
        )))
        return module.TemporalToyRecordRepublisher()
    return create


def test_yaml_on_clears_stale_other_robot_map_after_ten_fresh_observations(record_factory):
    node = record_factory(dynamic_free_clear_count=10, dynamic_occupied_confirm_count=1)
    node.global_map_callback(grid_message(OCCUPIED, 1), 0)
    for stamp in range(2, 11):
        node.local_map_callback(grid_message(FREE, stamp), 1)
        assert cell_value(node.temporal_fusion) == OCCUPIED
    node.local_map_callback(grid_message(FREE, 11), 1)
    assert cell_value(node.temporal_fusion) == FREE
    node.global_map_callback(grid_message(OCCUPIED, 12), 0)
    assert cell_value(node.temporal_fusion) == FREE
    assert node.build_merged_global(NS(sec=12, nanosec=0)).data == [FREE]


def test_yaml_off_does_not_run_temporal_fusion(record_factory):
    node = record_factory(enabled=False)
    node.global_map_callback(grid_message(OCCUPIED, 1), 0)
    for stamp in range(2, 30):
        node.local_map_callback(grid_message(FREE, stamp), 1)
    assert node.temporal_fusion is None
    assert node.build_merged_global(None) == "baseline_union"


def test_republished_same_map_is_not_counted_as_new_evidence(record_factory):
    node = record_factory(dynamic_free_clear_count=4)
    node.global_map_callback(grid_message(OCCUPIED, 1), 0)
    same = grid_message(FREE, 2)
    for _ in range(100):
        node.local_map_callback(same, 1)
    assert node.temporal_fusion.frame_index == 1
    assert cell_value(node.temporal_fusion) == OCCUPIED


def test_late_second_robot_global_cannot_undo_callback_clear(record_factory):
    node = record_factory(dynamic_free_clear_count=2)
    node.global_map_callback(grid_message(OCCUPIED, 1), 0)
    node.local_map_callback(grid_message(FREE, 2), 0)
    node.local_map_callback(grid_message(FREE, 3), 0)
    node.global_map_callback(grid_message(OCCUPIED, 1), 1)
    assert cell_value(node.temporal_fusion) == FREE


def test_unknown_observations_do_not_clear_wall(record_factory):
    node = record_factory(dynamic_free_clear_count=2)
    node.global_map_callback(grid_message(OCCUPIED, 1), 0)
    for stamp in range(2, 20):
        node.local_map_callback(grid_message(UNKNOWN, stamp), 1)
    assert cell_value(node.temporal_fusion) == OCCUPIED


def test_fresh_occupied_observation_resets_pending_free_evidence(record_factory):
    node = record_factory(dynamic_free_clear_count=3, dynamic_occupied_confirm_count=1)
    node.global_map_callback(grid_message(OCCUPIED, 1), 0)
    for value, stamp in [(FREE, 2), (FREE, 3), (OCCUPIED, 4), (FREE, 5), (FREE, 6)]:
        node.local_map_callback(grid_message(value, stamp), 1)
        assert cell_value(node.temporal_fusion) == OCCUPIED
    node.local_map_callback(grid_message(FREE, 7), 1)
    assert cell_value(node.temporal_fusion) == FREE
