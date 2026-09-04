# 두 로봇 Live Mapping

이 문서는 `run_two_mid360_2d_mapping.sh`를 이용한 두 MID-360 로봇의 권장
실행 순서를 설명합니다. 현재 구현 범위는 `r0`, `r1` 두 대이며, 정확히 한
노트북이 `--mapping-host` 역할을 맡습니다.

## 운용 가정

- 두 로봇은 같은 평면 바닥에서 **나란히**, **같은 방향**을 바라본 상태로
  시작합니다.
- initial alignment가 완료될 때까지 두 로봇을 움직이지 않습니다.
- 두 노트북은 같은 `ROS_DOMAIN_ID`를 사용하고 `ROS_LOCALHOST_ONLY=0`이어야 합니다.
- LiDAR/IMU 토픽과 TF frame은 로봇별로 분리되어야 합니다.

기본 입력은 다음과 같습니다.

| 로봇 | LiDAR | IMU | LiDAR frame |
| --- | --- | --- | --- |
| r0 | `/r0/livox/lidar` | `/r0/livox/imu` | `r0/livox_frame` |
| r1 | `/r1/livox/lidar` | `/r1/livox/imu` | `r1/livox_frame` |

## 권장 시작 순서

스크립트의 기본값은 `TWO_LIVE_WAIT_FOR_INITIAL_ALIGNMENT=true`입니다. 따라서
두 노트북 모두 다음 두 단계로 동작합니다.

### STAGE 1: raw LiDAR initial alignment

먼저 각 노트북의 Livox driver만 시작합니다. 이 단계에서는 다음 노드가 아직
시작되지 않습니다.

- RTAB-Map `icp_odometry`
- IMU/plane 기반 mapping state
- occupancy mapper
- common-frame odometry/map republisher
- merged-map fusion

`--mapping-host`로 지정한 노트북만 `initial_alignment_live.launch.py`를 실행합니다.
이 launch는 두 raw LiDAR와 두 sensor static TF만 사용해 cropped XYZ 3D ICP를
수행합니다.

```text
/r0/livox/lidar ─┐
                  ├─ initial_xy_icp_alignment
/r1/livox/lidar ─┘
                         │
                         └─ /toy/initial_xy_alignment
```

두 노트북 모두 transient-local QoS로 `/toy/initial_xy_alignment`를 기다립니다.
대기 중에는 다음 로그가 표시됩니다.

```text
[STAGE 1/2] ODOM/MAPPING BLOCKED: waiting for /toy/initial_xy_alignment ...
```

### STAGE 2: odometry, mapping, fusion

유효한 alignment transform이 publish된 뒤에만 각 노트북의 로컬 RTAB-Map
odometry와 occupancy mapping이 시작됩니다. mapping host에서는 common-frame
republisher와 merged-map fusion도 별도 launch로 시작합니다.

```text
[STAGE 1/2] INITIAL ALIGNMENT READY.
[STAGE 2/2] Releasing RTAB-Map odometry and occupancy mapping now.
```

**두 노트북 모두에서 `STAGE 2/2` 로그를 확인한 뒤 로봇을 움직입니다.**

## 실행 명령

먼저 두 노트북에서 같은 branch를 빌드합니다.

```bash
cd ~/co_3dto2d_mapping
git fetch origin
git switch fix/initial-alignment-config-startup-gate

source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select co_3dto2d_mapping
source install/local_setup.bash
```

r0 노트북:

```bash
bash scripts/run_two_mid360_2d_mapping.sh \
  --robot-number 1
```

r1을 mapping host로 사용하는 경우:

```bash
bash scripts/run_two_mid360_2d_mapping.sh \
  --robot-number 2 \
  --mapping-host
```

`--mapping-host`는 r0 또는 r1 어느 쪽에 지정해도 되지만, 정확히 한 곳에만
지정해야 합니다. 두 곳 모두 빠뜨리면 양쪽이 alignment를 기다리며 정지하고,
두 곳 모두 지정하면 alignment/fusion publisher가 중복됩니다.

alignment가 무한정 대기하는 것을 피하고 싶다면 초 단위 timeout을 지정합니다.

```bash
bash scripts/run_two_mid360_2d_mapping.sh \
  --robot-number 2 \
  --mapping-host \
  --alignment-timeout 120
```

초기 정지 시간을 바꾸려면 다음 옵션을 사용합니다.

```bash
--alignment-startup-delay 15
```

기존처럼 odometry와 alignment를 동시에 시작하는 동작은 디버깅 목적으로만
남겨두었습니다.

```bash
--no-initial-alignment-gate
```

## Initial ICP 설정

`initial_xy_icp_alignment`는 occupancy mapper와 동일하게
`--mapping-config`로 지정한 YAML을 읽습니다. 기본 파일은
`config/occupancy.yaml`이며, aligner 전용 설정은 정확한 node namespace 아래에
있습니다.

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

중요한 점은 mapper의 과거 fixed-Z/inverse-Z 필터를 initial alignment에 다시
적용하지 않는다는 것입니다.

```yaml
use_z_filter: false
invert_z_slice: false
```

따라서 startup ICP는 body/range crop을 통과한 raw XYZ 구조 전체를 사용합니다.
`initial_alignment_live.launch.py`도 두 값을 `false`로 한 번 더 강제하므로,
오래된 custom YAML이 사용되더라도 inverse-Z가 실수로 켜지지 않습니다.

`two_live_mapping.launch.py`에 남아 있는 `alignment_*` launch argument는 이전
호출 방식과의 호환용입니다. 기본값은 빈 문자열이므로 YAML을 덮어쓰지 않으며,
사용자가 값을 명시한 경우에만 해당 parameter가 override됩니다. 새 설정은 가능하면
YAML의 `/initial_xy_icp_alignment` 블록에서 관리하는 것을 권장합니다.

custom profile 사용 예:

```bash
bash scripts/run_two_mid360_2d_mapping.sh \
  --robot-number 2 \
  --mapping-host \
  --mapping-config /absolute/path/to/my_occupancy.yaml
```

## 실제 실행되는 aligner

CMake는 `cropped_xyz_initial_icp_alignment.py`를 다음 executable 이름으로 설치합니다.

```text
initial_xy_icp_alignment.py
```

따라서 live startup의 대응점 검색과 RMSE는 XYZ 3차원에서 계산됩니다. accepted
6-DoF registration 결과 가운데 downstream 2D map fusion에 필요한 `x`, `y`, `yaw`만
`/toy/initial_xy_alignment`로 publish됩니다. heading/tilt prior와 여러 번의
consistent-result gate는 유지됩니다.

## Alignment 이후 데이터 경로

```text
raw LiDAR + IMU
       │
       ├─ RTAB-Map 3D ICP odometry
       │          │
       │          └─ /rN/odom
       │
       ├─ shared single-floor state
       │          ├─ mapping/plane_height_filtered
       │          └─ mapping/floor_odometry
       │
       └─ occupancy mapper
                  ├─ /rN/toy/corrected_odometry
                  ├─ /rN/toy/local_occupancy
                  └─ /rN/toy/global_occupancy

/toy/initial_xy_alignment + r0/r1 outputs
       └─ /toy_record/* and merged_global_occupancy
```

주요 출력:

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
/toy_record/r0/*
/toy_record/r1/*
/toy_record/merged_global_occupancy
```

## 점검 명령

STAGE 1에서 raw LiDAR가 양쪽 모두 보이는지 확인합니다.

```bash
ros2 topic hz /r0/livox/lidar
ros2 topic hz /r1/livox/lidar
ros2 topic echo --once /r0/livox/lidar --field header
ros2 topic echo --once /r1/livox/lidar --field header
```

alignment 확인:

```bash
ros2 topic echo --once \
  --qos-reliability reliable \
  --qos-durability transient_local \
  /toy/initial_xy_alignment \
  geometry_msgs/msg/TransformStamped
```

STAGE 2 이후 odometry와 map 확인:

```bash
ros2 topic hz /r0/odom
ros2 topic hz /r1/odom
ros2 topic hz /r0/toy/global_occupancy
ros2 topic hz /r1/toy/global_occupancy
ros2 topic echo --once /toy_record/merged_global_occupancy --field header
```

## 로그 해석

정상 순서는 다음과 같습니다.

```text
Starting physical robot ...
[STAGE 1/2] Initial alignment host is active.        # mapping host만
[STAGE 1/2] ODOM/MAPPING BLOCKED ...                 # 양쪽
Cached robot0 cropped XYZ ICP frame ...
Cached robot1 cropped XYZ ICP frame ...
Initial cropped-cloud 3D ICP alignment accepted ...
[STAGE 1/2] INITIAL ALIGNMENT READY.                 # 양쪽
[STAGE 2/2] Releasing RTAB-Map odometry ...          # 양쪽
RTAB-Map / occupancy mapper startup logs
[STAGE 2/2] ... merged occupancy fusion              # mapping host만
```

STAGE 1에서 RTAB-Map odometry 로그가 보인다면 다음을 확인합니다.

1. 두 노트북 모두 새 branch를 다시 빌드하고 `install/local_setup.bash`를 source했는지
2. `--no-initial-alignment-gate`를 사용하지 않았는지
3. 예전 `install/` workspace가 `AMENT_PREFIX_PATH` 앞쪽에 남아 있지 않은지
4. 별도의 `ros2 launch ... two_live_mapping.launch.py`가 동시에 실행 중이지 않은지

직접 `ros2 launch co_3dto2d_mapping two_live_mapping.launch.py`를 실행하는 경로는
호환성을 위해 alignment와 odometry를 같은 launch graph에서 시작합니다. **strict
alignment-first gate는 `run_two_mid360_2d_mapping.sh`의 기본 동작입니다.**
