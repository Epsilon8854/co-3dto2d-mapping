# Plane-relative mapping visualization

The default visualization follows the active runtime pipeline instead of legacy
fixed-Z and raw-odometry topics.

## RViz

### Two robots

```bash
rviz2 -d install/co_3dto2d_mapping/share/co_3dto2d_mapping/rviz/two_robot_mapping.rviz
```

The default enabled layers are:

- `/toy_record/merged_global_occupancy`
- `/toy_record/r0/odom` and `/toy_record/r1/odom` (ground-fused pose when fresh)
- `/r0/mapping/plane_height_filtered`
- `/r1/mapping/plane_height_filtered`
- `r0/base_link` and `r1/base_link` axes

The camera is intentionally oblique rather than exactly top-down so `z`, roll,
and pitch are visible. The plane-height PointCloud2 displays use **Best Effort**
QoS, matching their sensor-data publishers.

Raw mapping clouds and legacy integrated endpoint clouds remain available as
disabled diagnostic layers. Enabling both raw and filtered clouds is useful for
checking the 0.05-1.00 m plane-relative band.

### Single robot

```bash
rviz2 -d install/co_3dto2d_mapping/share/co_3dto2d_mapping/rviz/single_robot_mapping.rviz
```

The main odometry display uses `/r0/toy/corrected_odometry`. Its `Axes` shape
shows the final roll/pitch/yaw directly from the message. Raw RTAB-Map odometry
and planar x/y/yaw odometry are retained as disabled comparison layers.

## Rerun

```bash
ros2 launch co_3dto2d_mapping rerun_mapping.launch.py
```

Rerun subscribes to each robot's local plane-height cloud and transforms it with
the full final odometry quaternion before applying the inter-robot planar
alignment. It therefore visualizes:

- full `x/y/z/roll/pitch/yaw` robot poses,
- RGB body axes,
- final trajectories,
- plane-relative obstacle clouds,
- per-robot and merged occupancy.

The old `/toy_record/rN/slice_*_points` views are disabled by default because
they are mapper-global debug endpoints and may be confusing beside the final
pose-fused local cloud. They can be restored in `config/rerun.yaml`:

```yaml
visualize_legacy_slice_points: true
```

Relevant Rerun parameters:

```yaml
visualize_plane_height_cloud: true
plane_height_cloud_topic_format: "/r{robot_id}/mapping/plane_height_filtered"
plane_height_point_radius: 0.025
robot_axis_length_m: 0.60
max_trajectory_points: 5000
visualize_legacy_slice_points: false
slice_point_radius: 0.025
```
