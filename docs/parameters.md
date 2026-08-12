# Parameter reference

This is the authoritative reference for the current package. Values come from node declarations, `config/occupancy.yaml`, `config/rerun.yaml`, and the shipped launch files. The old Swarm-SLAM `docs/toy-occupancy-map.md` is historical background, not this package's interface.

## Precedence and conventions

For `occupancy_mapper`, effective values are resolved in this order: **node declaration default → YAML file → launch override**. `single_bag_mapping` supplies `occupancy_config_file` first and an explicit parameter dictionary second, so its dictionary wins over `config/occupancy.yaml`. Other Python nodes use declaration defaults unless a launch dictionary or YAML supplies a value.

Distances are metres; periods are milliseconds or seconds as named; `_deg` is degrees; angular convergence is radians. Topics beginning with `/` are absolute; relative mapper outputs resolve in the node namespace. The occupancy contract is **unknown `-1`**, **free `0`**, **occupied `100`**.

## C++ occupancy mapper

Node: `occupancy_mapper` (node name `occupancy_mapper`). It approximately synchronizes PointCloud2 and Odometry and publishes `toy/local_occupancy`, `toy/global_occupancy`, `toy/slice_kept_points`, and `toy/slice_rejected_points`. Under `/r0`, those are `/r0/toy/*`.

| Parameter | Declaration default | Effect and validation |
| --- | --- | --- |
| `scan_cloud_topic` | `/livox/lidar` | PointCloud2 input topic. |
| `odom_topic` | `odom` | Odometry input topic. |
| `local_frame_id` | `base_link` | Frame for local cloud transformation and mapping. |
| `global_frame_id` | `odom` | Grid frame; can be replaced by first odometry header frame. |
| `use_odom_header_frame` | `true` | Take global frame from first Odometry header when available. |
| `grid_resolution` | `0.10` | Cell edge length; must be positive. |
| `local_map_size_m` | `20.0` | Local grid side; must exceed `grid_resolution`. |
| `z_min`, `z_max` | `0.4`, `1.2` | Z slice bounds; `z_min <= z_max` required. |
| `slice_in_global_frame` | `false` | Select Z in global rather than local/cloud coordinates. |
| `slice_z_in_cloud_frame` | `true` | When not global, select in source cloud rather than corrected local coordinates. |
| `invert_z_slice` | `false` | Keep points outside rather than inside the selected Z band. |
| `log_z_slice_stats` | `true` | Log slice counts/ranges. |
| `publish_slice_debug_points` | `true` | Publish kept/rejected debug PointCloud2 topics. |
| `slice_debug_points_max_points` | `80000` | Debug-point cap; clamped to at least 1. |
| `transform_cloud_to_local_frame` | `true` | Use TF to transform cloud points into `local_frame_id`. |
| `center_box_filter_half_extent_m` | `0.0` | Remove center square; clamped non-negative. |
| `range_min_m`, `range_max_m` | `0.0`, `0.0` | XY range for **occupied endpoints only**; zero max disables upper bound. Both clamp non-negative; enabled max cannot be below min. |
| `enable_raycast_free_space` | `true` | Trace free evidence from sensor origin toward eligible returns. |
| `raycast_free_value` | `0` | Grid value for free cells. |
| `raycast_occupied_value` | `100` | Grid value for hit cells. |
| `raycast_unknown_value` | `-1` | Initial/unobserved grid value. |
| `raycast_max_range_m`, `raycast_min_range_m` | `12.0`, `0.80` | Independent free-ray range; non-negative, and enabled max cannot be below min. |
| `raycast_clear_occupied` | `false` | Allow a free ray to overwrite occupied cells; false protects hits. |
| `occupied_threshold_points` | `1` | Points needed to mark a cell occupied; clamped to at least 1. |
| `publish_period_ms` | `200` | Grid publication period; clamped to at least 1 ms. |
| `global_map_padding_m` | `5.0` | Border when accumulated grid grows; clamped non-negative. |
| `sync_queue_size` | `100` | Approximate-time queue depth; clamped to at least 1. |
| `alignment_required` | `false` | Subscribe for initial XY alignment as configured by mapper. |
| `alignment_topic` | `/toy/initial_xy_alignment` | Transient-local reliable TransformStamped input when required. |

The three raycast values must be in `[-1, 100]` and mutually distinct. Do not conflate `range_min_m/range_max_m` with `raycast_min_range_m/raycast_max_range_m`: a return outside the occupied-hit range can still create free evidence before its endpoint, but its endpoint is not recorded as free or occupied.

### `config/occupancy.yaml`

```yaml
/**:
  ros__parameters:
    scan_cloud_topic: "/livox/lidar"
    odom_topic: "odom"
    local_frame_id: "base_link"
    global_frame_id: "odom"
    grid_resolution: 0.10
    local_map_size_m: 20.0
    z_min: 0.4
    z_max: 1.2
    slice_in_global_frame: false
    slice_z_in_cloud_frame: true
    invert_z_slice: true
    log_z_slice_stats: true
    publish_slice_debug_points: true
    slice_debug_points_max_points: 80000
    transform_cloud_to_local_frame: true
    center_box_filter_half_extent_m: 0.80
    range_min_m: 0.80
    range_max_m: 12.0
    enable_raycast_free_space: true
    raycast_free_value: 0
    raycast_occupied_value: 100
    raycast_unknown_value: -1
    raycast_max_range_m: 12.0
    raycast_min_range_m: 0.80
    raycast_clear_occupied: false
    occupied_threshold_points: 1
    publish_period_ms: 200
    global_map_padding_m: 5.0
    sync_queue_size: 100
```

YAML changes declared `invert_z_slice` from `false` to `true` and `center_box_filter_half_extent_m` from `0.0` to `0.80`, and supplies occupied ranges where declarations use `0.0`. It omits `use_odom_header_frame`, `alignment_required`, and `alignment_topic`, so direct-node use retains declaration defaults. Single/two-bag launch also explicitly override `center_box_filter_half_extent_m=0.80` and `slice_z_in_cloud_frame=true`.

## Python nodes

### Initial XY ICP alignment

Node: `initial_xy_icp_alignment.py`. It consumes initial clouds (`input_mode=cloud_initial`) or global grids (`global_occupancy`) and publishes a transient-local TransformStamped.

| Parameter | Default | Effect / clamp |
| --- | --- | --- |
| `input_mode` | `cloud_initial` | `cloud_initial` or `global_occupancy`; another value raises an error. |
| `robot0_cloud_topic`, `robot1_cloud_topic` | `/r0/livox/lidar`, `/r1/livox/lidar` | Cloud inputs for cloud mode. |
| `robot0_map_topic`, `robot1_map_topic` | `/r0/toy/global_occupancy`, `/r1/toy/global_occupancy` | OccupancyGrid inputs for map mode. |
| `alignment_topic` | `/toy/initial_xy_alignment` | Published alignment topic. |
| `target_frame_id`, `source_frame_id` | `odom`, `r1/odom` | Parent/child result frame IDs. |
| `local_frame_id` | `base_link` | Target of optional cloud TF conversion. |
| `transform_cloud_to_local_frame` | `true` | Enable cloud-frame TF conversion. |
| `z_min`, `z_max`, `invert_z_slice` | `0.4`, `1.2`, `true` | Cloud Z slice. |
| `frame_count` | `5` | Initial frames cached; clamped to at least 1. |
| `invert_result` | `false` | Invert estimated planar transform. |
| `center_box_half_extent_m` | `0.0` | Reject center square; clamped non-negative. |
| `voxel_size` | `0.10` | XY voxel size; clamped non-negative. |
| `max_points` | `30000` | Point cap; clamped to at least 100. |
| `max_correspondence_distance` | `0.75` | ICP correspondence distance. |
| `min_correspondences` | `100` | Minimum accepted correspondences; clamped to at least 3. |
| `min_fitness`, `max_rmse` | `0.05`, `0.40` | Acceptance thresholds. |
| `max_iterations` | `80` | ICP iteration cap; clamped to at least 1. |
| `recompute_period_sec` | `5.0` | Recompute interval; clamped to at least 0.1 s. |
| `occupied_threshold` | `50` | Grid values at or above this become alignment points. |
| `convergence_translation_m`, `convergence_rotation_rad` | `1e-4`, `1e-4` | ICP convergence tolerances. |
| `publish_period_sec` | `1.0` | Result publication timer; uses at least 0.1 s. |

### Record republisher

Node: `record_republisher.py`. It consumes per-robot `/rN/odom`, `/rN/toy/*`, and alignment, then republishes under `output_prefix`. Per-robot map frames become `robot_odom_frame_format`; a merged map uses `common_frame_id`.

| Parameter | Default | Effect / clamp |
| --- | --- | --- |
| `target_frame_id`, `common_frame_id` | `odom`, `map` | Incoming target expectation and shared merged/TF frame. |
| `alignment_topic` | `/toy/initial_xy_alignment` | Alignment TransformStamped input. |
| `publish_period_ms` | `200` | Output timer; clamped to at least 1 ms. |
| `occupied_threshold` | `50` | Grid occupied cutoff for merging. |
| `merged_padding_m` | `1.0` | Merged-grid border; clamped non-negative. |
| `robot_ids` | `[0, 1]` | Integer robot IDs to subscribe/publish. |
| `output_prefix` | `/toy_record` | Output root; trailing slash removed. |
| `robot_odom_frame_format` | `r{robot_id}/odom` | Per-robot map-frame template. |
| `robot_base_frame_format` | `r{robot_id}/base_link` | Per-robot base-frame template for TF. |
| `publish_tf` | `true` | Publish alignment/robot TF. |
| `publish_merged_global` | `false` | Publish `<prefix>/merged_global_occupancy`. |

### Rear-sector filter and IMU republisher

| Node | Parameter | Default | Effect / validation |
| --- | --- | --- | --- |
| `pointcloud_rear_sector_filter.py` | `input_topic`, `output_topic` | `/livox/lidar_raw`, `/livox/lidar` | PointCloud2 source and destination. |
|  | `enabled` | `true` | Pass unchanged when false. |
|  | `rear_filter_angle_deg` | `120.0` | Removed-sector width; clamped to `[0, 360]` before halving. |
|  | `rear_axis` | `-x` | Sector center: `x`, `-x`, `y`, or `-y`; another value errors. |
|  | `min_xy_range_m` | `0.0` | Points at/below this range are kept. |
|  | `log_period` | `100` | Log cumulative statistics every N messages when positive. |
|  | `output_frame_id` | empty | Replace cloud frame only when non-empty. |
| `imu_frame_republisher.py` | `input_topic`, `output_topic` | `/livox/imu_filtered_raw_frame`, `/livox/imu_filtered` | IMU source and destination. |
|  | `output_frame_id` | empty | Preserve input frame when empty; otherwise replace it. |

### Rerun mapping node

Node: `rerun_mapping_node.py` (`co_3dto2d_rerun`). It subscribes to `/toy_record/r0/*`, `/toy_record/r1/*`, optional merged occupancy, and `/toy/initial_xy_alignment`; it is optional.

| Parameter | Declaration / `config/rerun.yaml` default | Effect |
| --- | --- | --- |
| `spawn_viewer` | `true` | Start a compatible `rerun` viewer process. |
| `rerun_port` | `9876` | Viewer/proxy port. |
| `occupancy_point_radius` | `0.045` | Occupied-cell render radius. |
| `slice_point_radius` | `0.025` | Slice-point render radius. |
| `odometry_point_radius` | `0.04` | Odometry render radius. |

## Launch-argument matrix

| Launch file | Key arguments (defaults) | Namespace/topic/frame behavior |
| --- | --- | --- |
| `bag_mid360.launch.py` | `bag_path` required, `rate=1.0`, `storage_id=sqlite3`, `lidar_topic=/livox/lidar`, `imu_topic=/livox/imu`, `play_tf_static=true` | Validates `metadata.yaml`; plays LiDAR, IMU, optional `/tf_static`, then remaps. |
| `single_bag_mapping.launch.py` | `robot_id=0`, `bag_path` required, `rate=1.0`, `storage_id=sqlite3`, `occupancy_config_file=config/occupancy.yaml` | Namespace is `/r<robot_id>`; default mapper frames are `base_link` and `odom`. |
|  | `enable_rear_lidar_filter=true`, `rear_filter_angle_deg=120`, `rear_filter_axis=-x`, `rear_filter_min_xy_range_m=0` | Configures filtered LiDAR routing; IMU raw/filtered defaults are `/livox/imu` and `/livox/imu_filtered`. |
|  | `center_box_filter_half_extent_m=0.80`, `slice_z_in_cloud_frame=true`, `transform_cloud_to_local_frame=true` | Explicit mapper launch overrides after YAML. |
| `two_bag_mapping.launch.py` | `bag_path_0`, `bag_path_1` required, `rate=1.0`, `storage_id=sqlite3`, `robot_delay_s=20` | Creates `/r0` and `/r1`, with `rN/base_link`, `rN/livox_frame`, `rN/odom`. |
|  | `enable_record_republisher=true`, `record_output_prefix=/toy_record`, `record_publish_merged_global=true` | Enables the `map`-frame RViz/recording surface. |
|  | `alignment_*` | Alignment reads global grids; `alignment_voxel_size=0.05` overrides the ICP declaration default `0.10`. |
| `rerun_mapping.launch.py` | `config_file=config/rerun.yaml`, `spawn_viewer=true`, `rerun_port=9876` | Starts optional non-namespaced Rerun subscriber; launch values override YAML. |

Shared sensor-static-transform arguments are `sensor_tf_x/y/z`, `sensor_tf_yaw/pitch/roll`, `sensor_parent_frame`, `sensor_child_frame`, and `publish_sensor_static_tf`. Defaults are zero translation/angles except `sensor_tf_roll=3.141592653589793`; replace them with measured MID-360 extrinsics for real data.
