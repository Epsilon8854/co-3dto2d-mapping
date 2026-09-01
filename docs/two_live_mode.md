# 두 로봇 Live Mapping

`two_live_mapping.launch.py`는 기존 `two_bag_mapping.launch.py`의 두 로봇 정렬 및 `/toy_record` 재게시 구성을 실시간 센서 입력에 연결합니다. 현재 구현 범위는 bag 모드와 동일한 **2대(r0, r1)** 입니다. 분산 실행에서는 각 로봇 노트북이 자기 mapping pipeline만 실행하고 한 노트북이 fusion도 담당합니다.

## 전제 조건

두 로봇의 LiDAR와 IMU가 동일한 ROS 2 그래프에서 보이되, 반드시 서로 다른 절대 토픽을 사용해야 합니다. 기본값은 다음과 같습니다.

| 로봇 | LiDAR 입력 | IMU 입력 |
| --- | --- | --- |
| r0 | `/r0/livox/lidar` | `/r0/livox/imu` |
| r1 | `/r1/livox/lidar` | `/r1/livox/imu` |

두 Livox driver가 모두 `/livox/lidar`, `/livox/imu`를 publish하면 메시지가 섞여 어느 로봇의 데이터인지 구분할 수 없습니다. 각 driver 설정 또는 driver launch의 namespace/remap을 먼저 바꿔야 합니다. 여러 PC에서 실행할 때는 DDS 통신이 가능하고 `ROS_DOMAIN_ID`가 같아야 하며, 센서 timestamp 불일치를 줄이도록 chrony/NTP 등으로 시스템 시각도 동기화합니다.

Livox driver나 별도 `robot_state_publisher`가 `/tf` 또는 `/tf_static`을 publish한다면 `base_link`, `livox_frame` 같은 frame도 로봇별 prefix를 사용해야 합니다. 외부에서 이미 `r0/base_link -> r0/livox_frame`, `r1/base_link -> r1/livox_frame`을 올바르게 제공한다면 `publish_sensor_static_tf:=false`로 중복 static TF를 끕니다.

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

내부적으로 로봇 1은 `enable_robot0_pipeline:=true`, 로봇 2는
`enable_robot1_pipeline:=true`만 사용합니다. fusion host는 추가로
`enable_fusion:=true`를 사용하지만 다른 로봇의 sensor pipeline을 중복 실행하지
않습니다. 따라서 DDS graph에서 양쪽 raw topic을 발견하더라도 각 odometry는
자기 로봇 입력만 처리합니다.

### 시작 순서와 기본 warm-up

두 스크립트를 실행할 때 로봇은 제자리에 고정합니다. 기본 설정에서는 Livox driver와
Madgwick IMU filter는 즉시 시작하지만, ICP odometry와 occupancy mapper는
`mapping_startup_delay_sec:=10.0` 동안 시작하지 않습니다. 이 구간은 센서와 IMU
자세 추정이 안정되기 전에 생기는 작은 변화가 odometry 원점과 초기 지도에 누적되는
것을 막기 위한 warm-up입니다.

두 로봇에서 스크립트를 실행한 뒤 최소 10초 동안 움직이지 말고, 각 노트북 로그에
`mid360_icp_odometry`와 `occupancy_mapper`가 시작된 것이 보인 다음 출발하는 방식을
권장합니다. 더 긴 안정화 시간이 필요한 장비에서는 다음처럼 값을 늘립니다.

```bash
bash scripts/run_two_mid360_2d_mapping.sh --robot-number 1 \
  --launch-arg mapping_startup_delay_sec:=15.0

bash scripts/run_two_mid360_2d_mapping.sh --robot-number 2 --mapping-host \
  --launch-arg mapping_startup_delay_sec:=15.0
```

두 노트북에 같은 값을 적용합니다. `0.0`으로 설정하면 이전처럼 mapping과 odometry가
즉시 시작됩니다.

## 중앙집중 실행

다음 방식은 별도 mapping PC 한 대에서 두 sensor pipeline을 모두 처리합니다.
위 분산 스크립트와 동시에 사용하면 publisher가 중복되므로 둘 중 한 방식만
선택합니다.

기본 토픽을 사용할 때:

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

각 입력 토픽은 `/`로 시작하는 절대 이름이어야 하며, r0/r1 입력은 서로 달라야 합니다. launch가 동일 토픽 또는 내부 mapping 토픽과의 순환 연결을 감지하면 즉시 종료합니다.

## 데이터 경로

LiDAR 입력은 새 `pointcloud_frame_republisher.py`를 거쳐 다음 공통 내부 토픽으로 들어갑니다.

- r0: `/r0/mapping/lidar`, frame `r0/livox_frame`
- r1: `/r1/mapping/lidar`, frame `r1/livox_frame`

이 노드는 점 좌표를 변환하지 않고 `header.frame_id`만 로봇별 이름으로 바꿉니다. 따라서 원본 점들이 실제로 각 MID-360 센서 좌표계에 표현되어 있어야 하며, `sensor_tf_*_0`, `sensor_tf_*_1`은 실제 장착 외부 파라미터와 일치해야 합니다.

IMU는 기존 Madgwick filter와 IMU frame republisher를 사용합니다.

- r0 filtered IMU: `/r0/mapping/imu_filtered`, frame `r0/livox_frame`
- r1 filtered IMU: `/r1/mapping/imu_filtered`, frame `r1/livox_frame`

각 mapping pipeline의 주요 결과는 기존 bag 두 개 모드와 동일합니다.

- `/r0/odom`, `/r1/odom`
- `/r0/toy/local_occupancy`, `/r1/toy/local_occupancy`
- `/r0/toy/global_occupancy`, `/r1/toy/global_occupancy`
- `/toy/initial_xy_alignment`
- `/toy_record/r0/*`, `/toy_record/r1/*`
- `/toy_record/merged_global_occupancy`

TF 트리는 `map -> r0/odom -> r0/base_link -> r0/livox_frame`과 `map -> r1/odom -> r1/base_link -> r1/livox_frame`으로 구성됩니다. `r1/odom`의 `map` 정렬은 ICP가 유효한 결과를 얻은 뒤 publish됩니다.

## ICP 정렬 시점과 재시도

두 global occupancy가 처음 보인 직후에는 지도 면적이 작고 startup noise의 영향이
클 수 있습니다. live 2대 모드에서는 두 지도가 모두 들어온 시점부터 추가로
`alignment_startup_delay_sec:=3.0`을 기다린 뒤 ICP를 시작합니다.

유효한 결과 한 번을 즉시 쓰지 않고 기본적으로 2회의 후보가 다음 범위 안에서
일치해야 정렬을 publish합니다.

- 평행 이동 차이: `alignment_max_consistency_translation_m:=0.25`
- 회전 차이: `alignment_max_consistency_rotation_rad:=0.0872664626` (5도)

대응점 부족이나 fitness/RMSE 기준 미달로 실패하면 다음 주기에 다시 시도합니다.
한 번 성공한 초기 정렬은 `alignment_lock_after_first:=true`에 의해 고정되어, 이후
각 로봇 odometry의 작은 흔들림이 공통 frame 정렬을 계속 움직이지 않게 합니다.
또한 독립 odom 원점 사이의 평행 이동이 큰 경우를 위해 identity 초기값과 두 지도
centroid를 맞춘 초기값을 모두 시험하고 더 나은 ICP 결과를 선택합니다.

환경에 따라 더 보수적으로 설정할 수 있습니다.

```bash
bash scripts/run_two_mid360_2d_mapping.sh --robot-number 2 --mapping-host \
  --launch-arg alignment_startup_delay_sec:=5.0 \
  --launch-arg alignment_required_consistent_results:=3 \
  --launch-arg alignment_max_consistency_translation_m:=0.15
```

정렬을 계속 갱신해야 하는 실험에서는
`--launch-arg alignment_lock_after_first:=false`를 사용합니다.

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

첫 두 PointCloud2 메시지의 `header.frame_id`가 각각 `r0/livox_frame`, `r1/livox_frame`인지 확인합니다.

## 주요 옵션

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

ICP 조건은 기존 두 bag 모드와 같은 `alignment_*` 인자로 조정합니다.

```bash
ros2 launch co_3dto2d_mapping two_live_mapping.launch.py \
  alignment_voxel_size:=0.05 \
  alignment_min_fitness:=0.05 \
  alignment_max_rmse:=0.40 \
  alignment_recompute_period_sec:=2.0
```

ICP 정렬을 위해 두 로봇의 global occupancy에 충분한 공통 구조가 있어야 합니다. 반복적이거나 대칭적인 환경에서는 잘못된 지역 최적점에 수렴할 수 있으므로 `/toy/initial_xy_alignment`와 병합 지도를 반드시 검증합니다.

`enable_record_republisher:=false`로 실행하면 `/toy_record`와 공통 `map` TF를 만들지 않습니다. 다중 로봇 frame 충돌을 막기 위해 각 RTAB-Map odometry의 직접 TF publish도 계속 꺼져 있으므로, 이 옵션에서는 `/r0/odom`, `/r1/odom` 토픽은 사용할 수 있지만 RViz의 `map` 기준 TF 트리는 연결되지 않습니다.
