# 두 로봇 Live Mapping

`two_live_mapping.launch.py`는 두 MID-360의 실시간 LiDAR/IMU 입력으로 각 로봇의
RTAB-Map ICP odometry와 2D occupancy mapper를 실행하고, 두 로봇의 좌표계를 정렬해
`/toy_record` 결과를 공통 `map` frame으로 재게시합니다. 현재 구현 범위는
**2대(r0, r1)** 입니다. 분산 실행에서는 각 로봇 노트북이 자기 mapping pipeline만
실행하고 한 노트북이 fusion도 담당합니다.

## 전제 조건

두 로봇의 LiDAR와 IMU가 동일한 ROS 2 graph에서 보이되, 반드시 서로 다른 절대
토픽을 사용해야 합니다. 기본값은 다음과 같습니다.

| 로봇 | LiDAR 입력 | IMU 입력 |
| --- | --- | --- |
| r0 | `/r0/livox/lidar` | `/r0/livox/imu` |
| r1 | `/r1/livox/lidar` | `/r1/livox/imu` |

두 Livox driver가 모두 `/livox/lidar`, `/livox/imu`를 publish하면 데이터가 섞여
어느 로봇의 데이터인지 구분할 수 없습니다. 각 driver 설정 또는 launch의
namespace/remap을 먼저 바꿔야 합니다. 여러 PC에서 실행할 때는 DDS 통신이 가능하고
`ROS_DOMAIN_ID`가 같아야 하며, chrony/NTP 등으로 시스템 시각도 동기화하는 것이
좋습니다.

Livox driver나 별도 `robot_state_publisher`가 `/tf` 또는 `/tf_static`을
publish한다면 `base_link`, `livox_frame` 같은 frame도 로봇별 prefix를 사용해야
합니다. 외부에서 이미 `r0/base_link -> r0/livox_frame`,
`r1/base_link -> r1/livox_frame`을 제공한다면
`publish_sensor_static_tf:=false`로 중복 static TF를 끕니다.

## 빌드

```bash
cd ~/co_3dto2d_mapping
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select co_3dto2d_mapping
source install/local_setup.bash
```

## 권장 실행: 로봇별 분산 mapping

두 노트북 모두 같은 저장소와 Livox workspace를 빌드한 뒤 같은 스크립트를
실행합니다. `--mapping-host`가 없어도 각 노트북에서 자기 odom과 occupancy가
생성됩니다.

물리 로봇 1 노트북:

```bash
bash scripts/run_two_mid360_2d_mapping.sh --robot-number 1
```

물리 로봇 2 노트북을 fusion host로 사용할 때:

```bash
bash scripts/run_two_mid360_2d_mapping.sh --robot-number 2 --mapping-host
```

두 로봇이 시작할 때 같은 장소에 함께 서 있다는 운영 조건을 기본으로 합니다.
startup ICP가 초기 `map <- r1/odom` 변환을 만든 뒤에는 occupancy place recognition을
실행하지 않습니다. 시작 위치가 다르거나 주행 중 재정렬이 필요한 경우에만 fusion
host에서 다음 옵션을 추가합니다.

```bash
bash scripts/run_two_mid360_2d_mapping.sh --robot-number 2 --mapping-host \
  --enable-place-recognition
```

내부적으로 로봇 1은 `enable_robot0_pipeline:=true`, 로봇 2는
`enable_robot1_pipeline:=true`만 사용합니다. fusion host는 추가로
`enable_fusion:=true`를 사용하지만 다른 로봇의 sensor pipeline을 중복 실행하지
않습니다. 따라서 DDS graph에서 양쪽 raw topic을 발견하더라도 각 odometry는 자기
로봇 입력만 처리합니다.

## 시작 순서와 기본 warm-up

두 스크립트를 실행할 때 로봇은 제자리에 고정합니다. 기본 설정에서는 Livox driver,
frame republisher, Madgwick IMU filter가 즉시 시작하지만 RTAB-Map ICP odometry와
occupancy mapper는 `mapping_startup_delay_sec:=10.0` 동안 시작하지 않습니다. 이
구간은 센서와 IMU 자세 추정이 안정되기 전에 생기는 작은 변화가 odometry 원점과
초기 지도에 누적되는 것을 막습니다.

두 로봇 사이의 정렬은 이 정지 구간을 활용합니다. fusion host에서 각
`/r*/mapping/lidar` 입력은 서로를 기다리지 않고, 자기 입력이 처음 보인 뒤
`alignment_startup_delay_sec:=3.0`을 기다려 기본 5 frame의 cropped XYZ submap을
독립적으로 만듭니다. ICP 계산만 두 submap이 모두 준비된 뒤 시작합니다. 기본값에서는 독립적인
두 정합 결과가 0.25 m / 5도 안에서 일치해야 정렬을 확정합니다. 정상적인 경우 이
과정은 10초 odometry warm-up이 끝나기 전에 완료되므로, 로봇은 적어도 다음 로그가
나올 때까지 움직이지 않는 것이 안전합니다.

```text
Initial cropped-cloud 3D ICP alignment accepted ...
```

센서 안정화가 더 오래 필요한 장비에서는 두 로봇에 같은 mapping delay를 적용하고,
fusion host의 alignment delay도 함께 늘립니다.

```bash
bash scripts/run_two_mid360_2d_mapping.sh --robot-number 1 \
  --launch-arg mapping_startup_delay_sec:=15.0

bash scripts/run_two_mid360_2d_mapping.sh --robot-number 2 --mapping-host \
  --launch-arg mapping_startup_delay_sec:=15.0 \
  --launch-arg alignment_startup_delay_sec:=5.0
```

## 중앙집중 실행

별도 mapping PC 한 대에서 두 sensor pipeline을 모두 처리할 수도 있습니다. 분산
스크립트와 동시에 사용하면 publisher가 중복되므로 둘 중 한 방식만 선택합니다.

```bash
ros2 launch co_3dto2d_mapping two_live_mapping.launch.py
```

실제 driver 토픽이 다른 경우:

```bash
ros2 launch co_3dto2d_mapping two_live_mapping.launch.py \
  robot0_lidar_topic:=/robot_alpha/livox/lidar \
  robot0_imu_topic:=/robot_alpha/livox/imu \
  robot1_lidar_topic:=/robot_beta/livox/lidar \
  robot1_imu_topic:=/robot_beta/livox/imu
```

각 입력 토픽은 `/`로 시작하는 절대 이름이어야 하며 r0/r1 입력은 서로 달라야
합니다. 동일 토픽 또는 내부 mapping 토픽과의 순환 연결을 감지하면 launch가 즉시
종료합니다.

## 데이터 경로

LiDAR 입력은 `pointcloud_frame_republisher.py`를 거쳐 다음 내부 토픽으로 들어갑니다.

- r0: `/r0/mapping/lidar`, frame `r0/livox_frame`
- r1: `/r1/mapping/lidar`, frame `r1/livox_frame`

이 두 토픽은 각 RTAB-Map `icp_odometry`의 `scan_cloud` 입력이며, 동시에 초기
cross-robot 3D ICP의 입력입니다. `enable_rear_lidar_filter:=true`이면 후방 sector가
제거된 뒤 두 소비자에게 동일한 cloud가 전달됩니다. frame republisher는 점 좌표를
변환하지 않고 `header.frame_id`만 로봇별 이름으로 바꾸므로, 원본 점은 실제
MID-360 센서 좌표계에 표현되어 있어야 하고 `sensor_tf_*_0`, `sensor_tf_*_1`은 실제
장착 외부 파라미터와 일치해야 합니다.

IMU 경로는 다음과 같습니다.

- r0 filtered IMU: `/r0/mapping/imu_filtered`, frame `r0/livox_frame`
- r1 filtered IMU: `/r1/mapping/imu_filtered`, frame `r1/livox_frame`

주요 결과:

- `/r0/odom`, `/r1/odom`
- `/r0/toy/local_occupancy`, `/r1/toy/local_occupancy`
- `/r0/toy/global_occupancy`, `/r1/toy/global_occupancy`
- `/toy/initial_xy_alignment`
- `/toy_record/r0/*`, `/toy_record/r1/*`
- `/toy_record/merged_global_occupancy`

TF 트리는 `map -> r0/odom -> r0/base_link -> r0/livox_frame`과
`map -> r1/odom -> r1/base_link -> r1/livox_frame`으로 구성됩니다.
`map -> r1/odom`은 3D ICP가 quality/stability gate를 통과한 뒤 publish됩니다.

## Cropped XYZ 3D ICP

이전 live 구성은 `/r0/toy/global_occupancy`와 `/r1/toy/global_occupancy`의 occupied
cell을 XY 점으로 바꿔 2D ICP를 수행했습니다. 현재 기본 live 구성은 occupancy map을
기다리지 않고, RTAB-Map과 같은 `/r*/mapping/lidar` PointCloud2를 사용합니다.
이 변경은 `two_live_mapping.launch.py`에만 적용되며, 기존 `two_bag_mapping.launch.py`의
occupancy 기반 2D 재생 경로는 회귀 호환성을 위해 그대로 유지됩니다.

처리 순서는 다음과 같습니다.

1. 각 cloud를 static TF로 자기 `rN/base_link` frame에 변환합니다.
2. `alignment_z_min`, `alignment_z_max`, `alignment_invert_z_slice`로 z slice를
   적용합니다. `slice_z_in_cloud_frame:=true`이면 mapper와 마찬가지로 변환 전 센서
   frame의 z를 판정에 사용하되, 정합 좌표는 계속 base frame XYZ입니다.
3. 로봇 본체 주변 사각 영역과 `alignment_range_min_m`~`alignment_range_max_m`
   밖의 점을 제거합니다.
4. 3D voxel downsample을 수행하고 최대 점 수를 제한합니다.
5. 대응점 거리와 RMSE를 **XYZ 3차원 거리**로 계산하고, SVD 기반 6-DoF rigid
   transform을 반복 추정합니다.
6. 반복적·대칭적 구조의 180도 오정합을 줄이기 위해 기존 heading prior를 유지하고,
   3D 정합이 비현실적으로 기울어지는 것을 막는 tilt prior도 적용합니다.
7. 최종 3D 결과의 z/roll/pitch는 진단 로그에 남기고, merged 2D occupancy와 기존 TF
   인터페이스에는 x/y/yaw만 투영해 publish합니다.

즉, **정합의 대응점 선택과 오차 계산은 3D**이지만 최종 지도 결합 transform은 기존
호환성을 위해 planar입니다. 두 로봇 사이에 실제 높이/roll/pitch까지 공통 TF로
반영해야 하는 경우에는 record republisher와 occupancy fusion 인터페이스도 6-DoF로
확장해야 합니다.

기본 crop/ICP 값:

| 인자 | 기본값 | 의미 |
| --- | ---: | --- |
| `alignment_frame_count` | 5 | 한 번의 정합 submap에 합칠 frame 수 |
| `alignment_z_min`, `alignment_z_max` | 0.4, 0.8 | z slice 경계 |
| `alignment_invert_z_slice` | true | slice 내부를 제거하고 외부를 사용 |
| `alignment_center_box_half_extent_m` | 0.80 | 로봇 중심 사각 영역 제거 |
| `alignment_range_min_m`, `alignment_range_max_m` | 0.80, 12.0 | base-frame XY 거리 crop |
| `alignment_voxel_size` | 0.10 | 3D voxel 크기 |
| `alignment_max_points` | 15000 | submap별 최대 점 수 |
| `alignment_max_correspondence_distance` | 0.75 | 3D 대응점 최대 거리 |
| `alignment_max_iterations` | 40 | 초기값별 최대 ICP 반복 |
| `alignment_max_tilt_deviation_rad` | 0.261799... | 허용 roll/pitch 합성 tilt, 15도 |

환경에 따라 다음처럼 더 보수적으로 설정할 수 있습니다.

```bash
bash scripts/run_two_mid360_2d_mapping.sh --robot-number 2 --mapping-host \
  --launch-arg alignment_startup_delay_sec:=5.0 \
  --launch-arg alignment_frame_count:=8 \
  --launch-arg alignment_voxel_size:=0.15 \
  --launch-arg alignment_max_points:=10000 \
  --launch-arg alignment_required_consistent_results:=3 \
  --launch-arg alignment_max_consistency_translation_m:=0.15
```

천장이나 바닥을 포함하는 편이 더 잘 맞는 환경에서는 z crop을 끌 수 있습니다.

```bash
--launch-arg alignment_use_z_filter:=false
```

초기 정렬을 고정하지 않고 주기적으로 새 cropped cloud submap으로 갱신하려면 다음을
사용합니다. 갱신 주기는 `alignment_recompute_period_sec`입니다.

```bash
--launch-arg alignment_lock_after_first:=false \
--launch-arg alignment_recompute_period_sec:=5.0
```

움직이는 동안 갱신하면 두 로봇 cloud가 시간 동기화되지 않은 영향이 커질 수 있으므로,
일반 운용에서는 기본값인 `alignment_lock_after_first:=true`를 권장합니다.

## RViz 및 점검

```bash
rviz2 -d "$(ros2 pkg prefix --share co_3dto2d_mapping)/rviz/two_robot_mapping.rviz"
```

```bash
ros2 topic hz /r0/mapping/lidar
ros2 topic hz /r1/mapping/lidar
ros2 topic echo /r0/mapping/lidar --once
ros2 topic echo /r1/mapping/lidar --once
ros2 topic echo /toy/initial_xy_alignment --once
ros2 run tf2_ros tf2_echo map r0/base_link
ros2 run tf2_ros tf2_echo map r1/base_link
```

첫 두 PointCloud2 메시지의 `header.frame_id`가 각각 `r0/livox_frame`,
`r1/livox_frame`인지 확인합니다. alignment 로그에는 각 submap의 점 수, 3D fitness,
3D RMSE, raw z/roll/pitch/yaw, planar publish 결과가 표시됩니다. 점 수가 너무 적으면
z/range/body crop을 완화하고, RMSE가 높으면 두 로봇에서 동시에 보이는 3D 구조가
충분한지 먼저 확인합니다.

후방 LiDAR sector filter를 켤 때:

```bash
ros2 launch co_3dto2d_mapping two_live_mapping.launch.py \
  enable_rear_lidar_filter:=true \
  rear_filter_angle_deg:=120.0 \
  rear_filter_axis:=-x
```

로봇별 센서 장착 자세가 다를 때:

```bash
ros2 launch co_3dto2d_mapping two_live_mapping.launch.py \
  sensor_tf_x_0:=0.0 sensor_tf_y_0:=0.0 sensor_tf_z_0:=0.0 \
  sensor_tf_roll_0:=3.141592653589793 \
  sensor_tf_x_1:=0.1 sensor_tf_y_1:=0.0 sensor_tf_z_1:=0.2 \
  sensor_tf_roll_1:=3.141592653589793
```

`enable_record_republisher:=false`로 실행하면 `/toy_record`와 공통 `map` TF를 만들지
않습니다. 다중 로봇 frame 충돌을 막기 위해 각 RTAB-Map odometry의 직접 TF publish도
꺼져 있으므로, 이 옵션에서는 `/r0/odom`, `/r1/odom` 토픽은 사용할 수 있지만 RViz의
`map` 기준 TF 트리는 연결되지 않습니다.
