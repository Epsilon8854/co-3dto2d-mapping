# co_3dto2d_mapping

Standalone ROS 2 package for converting Livox MID-360 point clouds and odometry into local and accumulated 2D occupancy grids. It includes a C++ mapper, MID-360 bag/IMU/odometry launch pipeline, two-robot alignment and record republishing, optional Rerun visualization, configuration, RViz, and unit tests.

The occupancy contract is deliberate: unknown is `-1`, ray-observed free space is `0`, and hit cells are `100`. It does not turn unobserved space into free space.

## Prerequisites

For a new public machine, use Ubuntu 22.04 with ROS 2 Humble. Install ROS 2 Humble from the [official instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html), then install the common workspace tools:

```bash
sudo apt update
sudo apt install -y python3-rosdep python3-colcon-common-extensions
sudo rosdep init  # first machine only
rosdep update

mkdir -p /path/to/ros2_ws/src
cd /path/to/ros2_ws/src
git clone https://github.com/Epsilon8854/co-3dto2d-mapping.git co_3dto2d_mapping
```

The package manifest declares the ROS dependencies, including `rtabmap_odom`, `imu_filter_madgwick`, and `rviz2`; `rosdep` below resolves them for the chosen ROS distribution. The Rerun Python package and `rerun` viewer binary are optional.

### Clean shell discipline

Start each new shell with only one ROS distribution sourced. On a new Humble machine:

```bash
bash --noprofile --norc
source /opt/ros/humble/setup.bash
```

Do not source a ROS 1 setup file in the same shell. Source
`/path/to/ros2_ws/install/local_setup.bash` only after the first successful
build.

This host has an existing Foxy/Livox setup. Its machine-specific alternative is:

```bash
bash --noprofile --norc
source /opt/ros/foxy/setup.bash
source /home/user/ws_livox/install/local_setup.bash
source /home/user/Swarm-SLAM/install/local_setup.bash
cd /home/user/Swarm-SLAM
```

On this host, do not rely on `~/.bashrc`, do not use the default `real_cslam` Conda shell, and do not source the workspace `setup.bash` after Foxy: those paths can reintroduce ROS 1 Noetic libraries.

## Build and test

From the new workspace root:

```bash
cd /path/to/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select co_3dto2d_mapping
source install/local_setup.bash
colcon test --packages-select co_3dto2d_mapping
colcon test-result --verbose
```

The test suite covers the occupancy utility contracts, record-republisher helpers, visualization helpers, and static package/launch isolation. It does not replace a live sensor or rosbag integration check.

## Bag requirements

`bag_path` must name a rosbag2 directory containing `metadata.yaml`; the launch code rejects an empty path or a directory without that file. The default storage plugin is SQLite:

```bash
ros2 bag info /path/to/bag
```

Use `storage_id:=sqlite3` for the normal SQLite bag layout. A supported MCAP environment can use `storage_id:=mcap`, but the bag metadata and installed ROS plugin must be compatible with the running distribution.

The playback pipeline reads `/livox/lidar`, `/livox/imu`, and optionally `/tf_static` from the bag. A bag without `/livox/imu` cannot initialize the supplied MID-360 odometry path.

## Run one robot

The single pipeline plays the bag, optionally filters the rear LiDAR sector, filters/republishes IMU data, runs RTAB-Map odometry, and starts the occupancy mapper. This exact command uses cloneable paths:

```bash
ros2 launch co_3dto2d_mapping single_bag_mapping.launch.py \
  bag_path:=/path/to/bag \
  storage_id:=sqlite3 \
  rate:=1.0 \
  use_sim_time:=true
```

Its default robot is `r0`. A real MID-360 needs measured `base_link -> livox_frame` extrinsics; the launch default identity translation and fixed roll are a convenient starting point, not a calibrated sensor transform.

## Run two bags

```bash
ros2 launch co_3dto2d_mapping two_bag_mapping.launch.py \
  bag_path_0:=/path/to/bag_robot_0 \
  bag_path_1:=/path/to/bag_robot_1 \
  storage_id:=sqlite3 \
  rate:=0.5 \
  robot_delay_s:=20.0
```

This launches independent `/r0` and `/r1` pipelines, computes an initial XY alignment from their global occupancy grids, and by default publishes a recording-friendly `/toy_record` surface. The maps remain independently accumulated in `r0/odom` and `r1/odom`; the record republisher and alignment transform provide the shared `map` visualization contract.

## Topics and frames

| Surface | One robot | Two robots |
| --- | --- | --- |
| Bag LiDAR input | `/livox/lidar_raw` then `/livox/lidar` | `/r0/livox/lidar_raw` / `/r1/livox/lidar_raw`, then filtered `/rN/livox/lidar` |
| Bag IMU input | `/livox/imu` then `/livox/imu_filtered` | `/rN/livox/imu` then `/rN/livox/imu_filtered` |
| Odometry | `/r0/odom` | `/r0/odom`, `/r1/odom` |
| Mapper output | `/r0/toy/local_occupancy`, `/r0/toy/global_occupancy` | the same pair for each `rN` |
| Slice debug output | `/r0/toy/slice_kept_points`, `/r0/toy/slice_rejected_points` | the same pair for each `rN` |
| Alignment / recording | not needed by default | `/toy/initial_xy_alignment` and `/toy_record/rN/{odom,local_occupancy,global_occupancy,slice_kept_points,slice_rejected_points}`; optional `/toy_record/merged_global_occupancy` |

The single pipeline uses `base_link`, `livox_frame`, and `odom` by default. The two-bag pipeline prefixes those frames as `r0/base_link`, `r0/livox_frame`, `r0/odom` and `r1/base_link`, `r1/livox_frame`, `r1/odom`. Keep the point-cloud frame, static transform, and `local_frame_id` consistent when changing extrinsics.

## RViz

After launching the two-bag pipeline, open the included layout:

```bash
rviz2 -d "$(ros2 pkg prefix --share co_3dto2d_mapping)/rviz/two_robot_mapping.rviz"
```

The checked-in RViz layout is a `/toy_record/*` layout and uses fixed frame `map`. Therefore leave the record republisher enabled (the two-bag default), retain its `map` common frame and TF publication, and make sure `/toy/initial_xy_alignment` is available. To view raw per-robot mapping without that republisher, create displays for `/rN/toy/*` and use the appropriate `rN/odom` frame instead.

## Optional Rerun visualization

Install compatible `rerun-sdk` and `rerun` tooling only if you want the external viewer, then run:

```bash
ros2 launch co_3dto2d_mapping rerun_mapping.launch.py \
  spawn_viewer:=true \
  rerun_port:=9876
```

The node subscribes to the same `/toy_record/*` outputs. Set `spawn_viewer:=false` when a compatible viewer is already running. Core mapping does not require Rerun.

## Configuration

See [`docs/parameters.md`](docs/parameters.md) for every declared mapper and Python-node parameter, YAML defaults, launch-argument overrides, units, validation, and the important difference between occupied-hit ranges and free-space raycast ranges.

## Legacy routing

The source monorepo's historical `docs/toy-occupancy-map.md` text describes a Foxy-era `cslam_experiments` / `cslam_visualization` route. Treat it as historical context only; this standalone subtree does not include that monorepo document. For this repository, use the commands and package names in this README, `launch/`, `config/`, and [`docs/parameters.md`](docs/parameters.md).

## Troubleshooting

- **“bag metadata not found” or `bag_path is required`:** pass the rosbag2 directory itself, not a parent or database file, and verify `/path/to/bag/metadata.yaml` exists.
- **No odometry or IMU initialization:** inspect the bag with `ros2 bag info /path/to/bag`; the supplied pipeline requires `/livox/imu`. Change `imu_raw_topic` only when the bag truly uses another IMU topic.
- **Library errors or ROS 1 messages appear:** open a fresh `bash --noprofile --norc` shell and source only the selected Humble setup, or use the exact Foxy sequence above on this host.
- **Cloud-to-base TF warnings or a rotated/offset map:** measure and supply `sensor_tf_x`, `sensor_tf_y`, `sensor_tf_z`, `sensor_tf_yaw`, `sensor_tf_pitch`, and `sensor_tf_roll`; do not treat the default transform as real MID-360 calibration.
