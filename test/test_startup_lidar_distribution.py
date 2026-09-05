from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]


def test_public_launch_uses_distributed_startup_maps():
    layer = (
        PACKAGE / "co_3dto2d_mapping" / "two_live_lidar_slice_launch.py"
    ).read_text()
    helper = (
        PACKAGE / "co_3dto2d_mapping" / "startup_lidar_launch_icp.py"
    ).read_text()
    cmake = (PACKAGE / "CMakeLists.txt").read_text()

    assert "OpaqueFunction(function=_local_startup_maps)" in layer
    assert "local_startup_map_nodes" in layer
    assert '"enable_robot%d_pipeline" % robot_id' in helper
    assert "startup_map_node(" in helper
    assert '"input_mode": "global_occupancy"' in helper
    assert '"robot0_map_topic": STARTUP_MAP_TOPICS[0]' in helper
    assert '"robot1_map_topic": STARTUP_MAP_TOPICS[1]' in helper
    assert '"startup_delay_sec": 0.0' in helper
    assert "launch/two_live_lidar_slice_mapping.launch.py" in cmake
    assert "RENAME two_live_mapping.launch.py" in cmake


def test_startup_planar_icp_does_not_launch_cloud_builders_on_fusion_host():
    helper = (
        PACKAGE / "co_3dto2d_mapping" / "startup_lidar_launch_icp.py"
    ).read_text()
    function = helper[
        helper.index("def startup_2d_map_icp_node") :
    ]

    # Startup map construction belongs to local_startup_map_nodes. The fusion
    # action itself should subscribe only to the two distributed OccupancyGrids.
    before_relay = function.split("def ", 1)[0] if "def " in function[4:] else function
    assert "startup_map_node(" not in before_relay
    assert '"input_mode": "global_occupancy"' in function
    assert '"transform_cloud_to_local_frame": False' in function


def test_lidar_slice_defaults_are_symmetric_and_filter_before_projection():
    mapper = (
        PACKAGE / "co_3dto2d_mapping" / "startup_lidar_occupancy.py"
    ).read_text()
    filtering = (
        PACKAGE / "co_3dto2d_mapping" / "startup_lidar_slice.py"
    ).read_text()

    assert 'self.declare_parameter("slice_center_z_m", 0.0)' in mapper
    assert 'self.declare_parameter("slice_half_height_m", 0.40)' in mapper
    assert "z_distance = np.abs(" in filtering
    assert "range_keep" in filtering
    assert "inside_center" in filtering
    assert "kept_below_center" in filtering
    assert "kept_at_or_above_center" in filtering
