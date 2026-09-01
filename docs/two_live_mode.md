# 두 로봇 Live Mapping

`two_live_mapping.launch.py`는 기존 `two_bag_mapping.launch.py`의 두 로봇 정렬 및 `/toy_record` 재게시 구성을 실시간 센서 입력에 연결합니다. 현재 구현 범위는 bag 모드와 동일한 **2대(r0, r1)** 입니다.

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

## 실행

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
  alignment_recompute_period_sec:=5.0
```

ICP 정렬을 위해 두 로봇의 global occupancy에 충분한 공통 구조가 있어야 합니다. 반복적이거나 대칭적인 환경에서는 잘못된 지역 최적점에 수렴할 수 있으므로 `/toy/initial_xy_alignment`와 병합 지도를 반드시 검증합니다.

`enable_record_republisher:=false`로 실행하면 `/toy_record`와 공통 `map` TF를 만들지 않습니다. 다중 로봇 frame 충돌을 막기 위해 각 RTAB-Map odometry의 직접 TF publish도 계속 꺼져 있으므로, 이 옵션에서는 `/r0/odom`, `/r1/odom` 토픽은 사용할 수 있지만 RViz의 `map` 기준 TF 트리는 연결되지 않습니다.
