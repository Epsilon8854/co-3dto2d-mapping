# Startup inter-robot 2D ICP

The public two-live launch builds one startup-only 2D occupancy map on each
robot laptop and aligns those two maps before releasing odometry and mapping.
The fusion host does not need the remote robot's full PointCloud2 stream for
startup registration.

## Data path

```text
robot N raw LiDAR PointCloud2
  -> local startup_lidar_occupancy
  -> /rN/startup/lidar_slice_occupancy
  -> fusion-host initial_xy_icp_alignment (global_occupancy mode)
  -> /toy/startup_xy_alignment
  -> odometry/mapping gate release
```

Each local map is constructed without odometry, a ground-plane estimate, or a
sensor-to-base transform. Points are filtered in the raw LiDAR coordinate
system in this order:

1. Keep `abs(z - startup_map_slice_center_z_m) <=
   startup_map_slice_half_height_m`.
2. Apply the configured XY range limits.
3. Remove the centered robot-body box.
4. Apply the optional rear-sector filter.
5. Project the remaining points to XY and rasterize occupied cells.

The defaults use a slice centered at LiDAR `z=0` with a half-height of `0.40 m`.
Because the predicate is symmetric, both positive and negative z returns are
included and reversing the LiDAR z-axis sign does not change the selected XY
points.

After startup, the current ground-plane filtering, per-robot local-window ICP,
and optional occupancy place-recognition pipeline are unchanged.

## Main parameters

- `startup_map_slice_center_z_m` (default `0.0`)
- `startup_map_slice_half_height_m` (default `0.40`)
- `alignment_frame_count` (number of stationary scans accumulated locally)
- `alignment_startup_delay_sec` (local sensor settle time)
- `alignment_center_box_half_extent_m`
- `alignment_range_min_m`, `alignment_range_max_m`
- `enable_rear_lidar_filter`, `rear_filter_angle_deg`, `rear_filter_axis`
- `startup_map_min_occupied_cells`

The startup map publishers use transient-local QoS and periodically republish
the accepted map. `initial_xy_icp_alignment.py` subscribes only to the two
OccupancyGrid topics in this public path and estimates an SE(2) transform
(`x`, `y`, `yaw`).
