# Single-floor shared-state mapping with fixed inter-robot alignment

This branch targets two ground robots operating on one approximately horizontal
floor. It intentionally stops at a fixed inter-robot `SE(2)` alignment: no pose
graph, submap graph, or trajectory optimization is introduced in this stage.

## Data flow

For each robot, RTAB-Map still provides the local 3-D LiDAR/IMU odometry. One
raw point-cloud stamp is then processed by the shared floor node:

```text
raw cloud + filtered IMU + raw odom
                 |
                 v
     one shared floor observation
        |                       |
        v                       v
plane-height obstacle cloud   mapping/floor_odometry
        |                       |
        +-----------+-----------+
                    v
             occupancy mapper
      local-window x/y/yaw refinement
                    |
                    v
           toy/corrected_odometry
           local/global occupancy
```

The occupancy mapper therefore projects the filtered cloud with the same
floor-derived `z/roll/pitch` state that was produced from that cloud stamp. The
single synchronized callback prevents the filtering path and odometry path from
fitting the same cloud independently.

## Single-floor estimator

`ground_plane_estimation_mode: single_floor` is the default. Under the flat,
single-floor assumption, the IMU-derived up vector fixes the plane normal. The
LiDAR fit is reduced to a robust one-dimensional search for floor offset:

1. select points below the robot in a gravity-aligned candidate region;
2. histogram their perpendicular heights;
3. choose the lowest sufficiently supported mode during initialization;
4. prefer the previous floor-height neighbourhood after initialization;
5. apply the existing jump gate, low-pass filter, hold timeout, and reset timeout.

This path avoids per-frame three-point RANSAC. Set
`ground_plane_estimation_mode: ransac` to retain the previous
IMU-constrained RANSAC estimator for comparison.

## Projection ablation switch

`ground_plane_pose_enabled` is the integration switch.

- `true`: the shared floor state replaces `z/roll/pitch` before occupancy
  projection; `x/y/yaw` remain from odometry and the mapper's bounded local ICP.
- `false`: raw odometry is passed through on `mapping/floor_odometry`; the same
  plane-height-filtered cloud is retained so the map-projection state is the
  controlled variable.

The floor filter and estimator remain active in both cases. To disable the
height filter separately, set `ground_plane_height_filter_enabled: false`.

## Fixed inter-robot alignment

The public two-live launch and combined-bag launch use the CPU-only occupancy
place-recognition frontend:

```text
growing global occupancy maps
  -> robot-centred occupancy keyframes
  -> polar occupied/free/observed descriptor retrieval
  -> symmetric distance-field search
  -> trimmed 2-D ICP
  -> free/occupied contradiction check
  -> pairwise-consistent SE(2) maximal-clique consensus
  -> fixed map <- r1/odom transform
```

The default consensus requires at least three verified measurements and at
least two distinct keyframes from each robot. Periodic stationary snapshots are
practically disabled (`stationary_keyframe_period_sec: 3600.0`), so repeated
copies of one startup pose do not count as independent support. The S3E profile
uses four verified measurements and tighter transform spread limits. Once
accepted, the alignment is latched and reused by the existing world-frame and
temporal merged-map compositor.

## S3Ev1 replay

Use the existing runner; it automatically supplies the sparse S3E profile to
both mapping and alignment:

```bash
bash scripts/run_s3ev1_mapping.sh \
  --sequence S3E_Laboratory_1 \
  --robot0 Alpha \
  --robot1 Bob \
  --rviz
```

Useful diagnostics:

```bash
ros2 topic echo /toy/place_recognition/status
ros2 topic echo --once /toy/initial_xy_alignment
ros2 topic echo --once /r0/mapping/floor_odometry
ros2 topic echo --once /r0/toy/corrected_odometry
ros2 run tf2_ros tf2_echo map r1/odom
```

## Scope retained for later stages

This integration deliberately retains growing global occupancy snapshots and
frame-count temporal evidence. Descriptor-first/patch-on-demand exchange,
immutable submaps, time-based evidence, temporal-static ICP masks, and sparse
anchor optimization remain separate follow-up changes so the fixed-alignment
baseline can be tested first.
