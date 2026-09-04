# co_3dto2d_mapping

Livox MID-360의 PointCloud2와 IMU를 받아 3D LiDAR odometry, single-floor 기반
2.5D occupancy mapping, 두 로봇의 초기 좌표계 정렬 및 merged map을 생성하는
ROS 2 패키지입니다. 현재 다중 로봇 실행 범위는 `r0`, `r1` 두 대입니다.

## 주요 구성

- RTAB-Map point-to-plane ICP odometry
- IMU gravity와 single-floor 가정을 이용한 floor height/attitude 추정
- plane-height obstacle cloud와 동일 state를 사용하는 occupancy projection
- bounded local scan-to-submap ICP
- raycasting 기반 free-space 및 temporal occupancy update
- 두 로봇 raw XYZ initial ICP와 fixed `map <- r1/odom` alignment
- 공통 `map` frame의 로봇별 map과 merged occupancy

## 환경

이 문서는 ROS 2 Humble, Livox-SDK2, Livox ROS Driver 2가 이미 설치되어 있다고
가정합니다.

- ROS 2 Humble
- Livox-SDK2
- Livox ROS Driver 2
- `rtabmap_odom`

단일 로봇 기본 입력은 `/livox/lidar`, `/livox/imu`입니다. 두 로봇 입력은 반드시
서로 다른 절대 토픽으로 분리되어야 합니다.

| 로봇 | LiDAR | IMU |
| --- | --- | --- |
| r0 | `/r0/livox/lidar` | `/r0/livox/imu` |
| r1 | `/r1/livox/lidar` | `/r1/livox/imu` |

## 빌드

```bash
git clone https://github.com/Epsilon8854/co-3dto2d-mapping.git ~/co_3dto2d_mapping
cd ~/co_3dto2d_mapping

git fetch origin
git switch fix/initial-alignment-config-startup-gate

source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install --packages-select co_3dto2d_mapping
source install/local_setup.bash
```

새 shell에서는 다음 순서로 source합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/co_3dto2d_mapping/install/local_setup.bash
```

## 운용 가정 및 주의사항

- 기본 연구 범위는 하나의 approximately horizontal single floor입니다.
- 두 로봇 live mode에서는 두 로봇을 **같은 바닥에 나란히 놓고 같은 방향을 보게 한
  상태로 시작**합니다.
- 두 LiDAR가 충분한 공통 구조를 관측할 수 있도록 간격을 너무 넓히지 않습니다.
- **두 노트북 모두에서 `STAGE 2/2` 로그가 나온 뒤에만 로봇을 움직입니다.**
- 두 PC는 같은 `ROS_DOMAIN_ID`를 사용하며 `ROS_LOCALHOST_ONLY=0`이어야 합니다.
- multi-PC 환경에서는 chrony/NTP로 시스템 시간을 동기화하는 것이 좋습니다.
- `r0/base_link`, `r0/livox_frame`, `r1/base_link`, `r1/livox_frame`처럼 TF frame도
  로봇별로 분리합니다.
- 외부에서 같은 sensor static TF를 제공하면 `publish_sensor_static_tf:=false`로
  중복 발행을 끕니다.

## 단일 로봇 live mode

Livox driver가 `/livox/lidar`, `/livox/imu`를 publish하는 상태에서 실행합니다.

```bash
ros2 launch co_3dto2d_mapping live_mapping.launch.py
```

실제 토픽 이름이 다르면 다음처럼 지정합니다.

```bash
ros2 launch co_3dto2d_mapping live_mapping.launch.py \
  scan_cloud_topic:=/actual/lidar \
  imu_raw_topic:=/actual/imu
```

주요 출력:

```text
/r0/odom
/r0/mapping/floor_odometry
/r0/toy/corrected_odometry
/r0/toy/local_occupancy
/r0/toy/global_occupancy
```

## 두 로봇 live mode

### 중요한 startup 순서

`run_two_mid360_2d_mapping.sh`의 기본 동작은 initial alignment가 끝나기 전까지
RTAB-Map odometry와 occupancy mapper를 시작하지 않습니다.

```text
STAGE 1
  Livox driver only
  raw /r0/livox/lidar + /r1/livox/lidar
       -> initial_xy_icp_alignment
       -> /toy/initial_xy_alignment

STAGE 2
  r0/r1 RTAB-Map odometry
  single-floor mapping
  occupancy maps
  common-frame republisher + merged map
```

대기 중에는 다음 로그가 표시됩니다.

```text
[STAGE 1/2] ODOM/MAPPING BLOCKED: waiting for /toy/initial_xy_alignment ...
```

정렬이 accepted되면 양쪽 노트북에서 다음 로그가 나온 뒤 odometry가 시작됩니다.

```text
[STAGE 1/2] INITIAL ALIGNMENT READY.
[STAGE 2/2] Releasing RTAB-Map odometry and occupancy mapping now.
```

### 분산 실행

두 노트북 중 정확히 한 곳만 `--mapping-host`로 지정합니다.

물리 로봇 1 (`r0`) 노트북:

```bash
cd ~/co_3dto2d_mapping
bash scripts/run_two_mid360_2d_mapping.sh --robot-number 1
```

물리 로봇 2 (`r1`)를 mapping host로 사용하는 경우:

```bash
cd ~/co_3dto2d_mapping
bash scripts/run_two_mid360_2d_mapping.sh \
  --robot-number 2 \
  --mapping-host
```

r0를 mapping host로 사용해도 됩니다. 단, 두 곳에서 동시에 `--mapping-host`를
지정하면 alignment와 merged-map publisher가 중복됩니다. 어느 곳에도 지정하지
않으면 두 로봇 모두 alignment를 기다립니다.

alignment가 일정 시간 안에 나오지 않으면 종료하도록 설정할 수 있습니다.

```bash
bash scripts/run_two_mid360_2d_mapping.sh \
  --robot-number 2 \
  --mapping-host \
  --alignment-timeout 120
```

초기 정지 시간을 늘리려면:

```bash
--alignment-startup-delay 15
```

기존처럼 alignment와 odometry를 동시에 시작하는 방식은 디버깅용으로만 남아
있습니다.

```bash
--no-initial-alignment-gate
```

### 중앙집중 실행

두 센서 stream을 한 PC에서 모두 처리할 수도 있습니다.

```bash
ros2 launch co_3dto2d_mapping two_live_mapping.launch.py
```

이 direct launch는 호환성을 위해 하나의 launch graph에서 alignment와 odometry를
함께 시작합니다. **엄격한 alignment-first gate가 필요하면 위 shell script를
사용합니다.**

## Initial XY/XYZ alignment 설정

실제로 설치되는 `initial_xy_icp_alignment.py`는
`cropped_xyz_initial_icp_alignment.py`입니다. 대응점, fitness, RMSE는 XYZ 3차원에서
계산하고, downstream 2D map fusion에는 accepted transform의 `x`, `y`, `yaw`를
publish합니다.

aligner는 mapper와 같은 `occupancy_config_file`을 읽습니다. 기본 설정은
`config/occupancy.yaml`의 다음 exact node block에 있습니다.

```yaml
/initial_xy_icp_alignment:
  ros__parameters:
    use_z_filter: false
    slice_z_in_cloud_frame: true
    z_min: -1000.0
    z_max: 1000.0
    invert_z_slice: false

    center_box_half_extent_m: 0.80
    range_min_m: 0.80
    range_max_m: 12.0
    voxel_size: 0.10
    max_points: 15000
    max_correspondence_distance: 0.75
    min_correspondences: 100
    min_fitness: 0.05
    max_rmse: 0.40
    max_iterations: 40
    frame_count: 5
    required_consistent_results: 2
    lock_after_first_alignment: true
```

과거 mapper용 inverse-Z 처리와 혼동되지 않도록 startup aligner의 기본 live 경로는
다음과 같습니다.

```yaml
use_z_filter: false
invert_z_slice: false
```

`initial_alignment_live.launch.py`도 두 값을 false로 안전하게 고정합니다. 나머지
crop/range/ICP threshold는 선택한 YAML에서 읽습니다.

custom config 사용:

```bash
bash scripts/run_two_mid360_2d_mapping.sh \
  --robot-number 2 \
  --mapping-host \
  --mapping-config /absolute/path/to/occupancy.yaml
```

`two_live_mapping.launch.py`의 기존 `alignment_*` argument는 호환용으로 유지하지만
기본값은 비어 있습니다. 값을 명시했을 때만 YAML 설정을 override합니다. 새로운
실험 설정은 `/initial_xy_icp_alignment` YAML block에 두는 것을 권장합니다.

## 주요 토픽과 TF

```text
/r0/odom
/r1/odom
/r0/toy/corrected_odometry
/r1/toy/corrected_odometry
/r0/toy/local_occupancy
/r1/toy/local_occupancy
/r0/toy/global_occupancy
/r1/toy/global_occupancy
/toy/initial_xy_alignment
/toy_record/r0/odom
/toy_record/r1/odom
/toy_record/r0/global_occupancy
/toy_record/r1/global_occupancy
/toy_record/merged_global_occupancy
```

공통 TF 구조:

```text
map -> r0/odom -> r0/base_link -> r0/livox_frame
map -> r1/odom -> r1/base_link -> r1/livox_frame
```

`map -> r1/odom`은 initial alignment가 accepted된 뒤 publish됩니다.

## RViz

단일 로봇:

```bash
rviz2 -d "$(ros2 pkg prefix --share co_3dto2d_mapping)/rviz/single_robot_mapping.rviz"
```

두 로봇:

```bash
rviz2 -d "$(ros2 pkg prefix --share co_3dto2d_mapping)/rviz/two_robot_mapping.rviz"
```

## Bag mode

단일 bag:

```bash
ros2 launch co_3dto2d_mapping single_bag_mapping.launch.py \
  use_bag:=true \
  bag_path:=/path/to/bag \
  storage_id:=sqlite3 \
  rate:=1.0 \
  use_sim_time:=true
```

두 bag:

```bash
ros2 launch co_3dto2d_mapping two_bag_mapping.launch.py \
  bag_path_0:=/path/to/robot0_bag \
  bag_path_1:=/path/to/robot1_bag \
  rate:=0.5 \
  robot_delay_s:=20.0
```

S3E runner:

```bash
bash scripts/run_s3ev1_mapping.sh \
  --sequence S3E_Laboratory_1 \
  --robot0 Alpha \
  --robot1 Bob \
  --rviz
```

## 점검

STAGE 1 raw input:

```bash
ros2 topic hz /r0/livox/lidar
ros2 topic hz /r1/livox/lidar
```

alignment:

```bash
ros2 topic echo --once \
  --qos-reliability reliable \
  --qos-durability transient_local \
  /toy/initial_xy_alignment \
  geometry_msgs/msg/TransformStamped
```

STAGE 2:

```bash
ros2 topic hz /r0/odom
ros2 topic hz /r1/odom
ros2 topic hz /r0/toy/global_occupancy
ros2 topic hz /r1/toy/global_occupancy
ros2 topic echo --once /toy_record/merged_global_occupancy --field header
```

STAGE 1에서 RTAB-Map odometry 로그가 나타난다면 다음을 확인합니다.

1. 두 노트북 모두 같은 branch를 다시 빌드했는지
2. 새 shell에서 올바른 `install/local_setup.bash`를 source했는지
3. `--no-initial-alignment-gate`를 사용하지 않았는지
4. 예전 `two_live_mapping.launch.py`가 별도로 실행 중이지 않은지
5. `AMENT_PREFIX_PATH` 앞쪽에 오래된 install workspace가 남아 있지 않은지

자세한 내용은 [`docs/two_live_mode.md`](docs/two_live_mode.md)를 참고합니다.
