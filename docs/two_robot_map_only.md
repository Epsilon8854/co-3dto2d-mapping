# Two-robot registration: 2-D occupancy maps only

## Runtime contract

Each robot starts its own LiDAR/IMU odometry and occupancy mapper independently.
The local `mapping_startup_delay_sec` warm-up remains 10 seconds by default;
there is no remote-cloud or accepted-alignment startup barrier.

On the fusion host, `inter_robot_place_alignment.py` consumes:

- `/r0/toy/global_occupancy` and `/r1/toy/global_occupancy` (`OccupancyGrid`).
- `/r0/toy/corrected_odometry` and `/r1/toy/corrected_odometry` (`Odometry`).

Geometry is registered using 2-D occupancy patches: polar candidate retrieval,
correlative matching, trimmed SE(2) ICP, and multi-keyframe consensus. Odometry
locates each patch and converts its transform into the robot odometry frames;
no LiDAR PointCloud2 is consumed by the inter-robot aligner.

The accepted transform is still published on `/toy/initial_xy_alignment`
(`map <- r1/odom`), preserving the existing map compositor interface. No
`/toy/startup_xy_alignment` publisher or startup gate is launched. The compositor
starts independently and does not invent an identity alignment for robot 1.
Until map registration is accepted, robot 1 must not be shown as already aligned.

Local 3-D LiDAR odometry and floor-height filtering are intentionally unchanged.
This change is an **ICP input/dependency contract**, not a DDS network firewall:
record/debug consumers still have their existing `slice_kept_points` and
`slice_rejected_points` cloud subscriptions. Disable/remove those separately
when a deployment must exchange absolutely no 3-D debug traffic.

## Running

Rebuild and source the changed package on **both** robot PCs. A rebuild is
required because CMake installs the public launch under a different filename.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select co_3dto2d_mapping
source install/local_setup.bash
```

Physical robot 1 (`r0`):

```bash
bash scripts/run_two_mid360_2d_mapping.sh --robot-number 1
```

Physical robot 2 (`r1`), also the fusion host:

```bash
bash scripts/run_two_mid360_2d_mapping.sh --robot-number 2 --mapping-host
```

Exactly one PC should enable fusion. `--enable-place-recognition` is no longer
required: 2-D registration is the mandatory alignment stage whenever
`enable_fusion:=true`. `enable_fusion:=false` still selects local mapping only.

## Compatibility and configuration

Old runner arguments remain accepted:

- `enable_place_recognition:=false` is deprecated and ignored, with a notice.
  Previously it disabled an optional post-startup stage; now that stage is the
  only inter-robot alignment path, so honoring the old false value would leave
  the fusion host with no alignment publisher.
- `wait_for_initial_alignment` and the `startup_alignment_*` arguments no longer
  gate anything. Passing `wait_for_initial_alignment:=true` prints a notice.
- `CO3DTO2D_STARTUP_DIRECT_LIDAR` and legacy cloud-ICP tuning arguments cannot
  restore 3-D registration in either the public or base two-live entry point.

`alignment_config_file` overlays `config/place_recognition.yaml`; map topics,
frame names, alignment output, and lock settings remain explicit launch
overrides. Tune 2-D registration and consensus in that YAML, not through the old
cloud-ICP threshold arguments. The existing conservative consensus rules remain
unchanged: sufficient overlap and distinct keyframes are needed. Local mapping
continues even when alignment is searching or rejecting candidates.

The separate two-bag path already uses `input_mode=global_occupancy`, and the
combined-bag path already selects the occupancy aligner. Neither requires the
removed live startup barrier.

## Checks on ROS 2 hardware

```bash
# On either PC, its own odom/map should appear even before the other robot starts.
ros2 topic hz /r0/odom
ros2 topic hz /r0/toy/global_occupancy

# On the fusion host, confirm only map/odom application inputs for this node.
ros2 node info /inter_robot_place_alignment
ros2 topic echo /toy/place_recognition/status --once
ros2 topic echo /toy/initial_xy_alignment --once
```

While testing, do not start a rosbag recorder or remote RViz PointCloud2 display
and mistake its cloud subscriptions for subscriptions from the aligner.

## Automated checks

```bash
python3 -m pytest -q test/test_two_live_startup_gating.py
```

The 22 launch regression cases execute the real launch setup with lightweight
launch stand-ins. They cover both installed/public and base entry points,
distributed roles, fusion-only execution, legacy flags, config precedence,
topic isolation, and absence of startup gates or inter-robot cloud aligners.
They do not replace ROS 2/DDS, RTAB-Map, rosbag, or physical robot validation.
