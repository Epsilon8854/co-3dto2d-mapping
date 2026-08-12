# co_3dto2d_mapping

Livox MID-360 포인트 클라우드와 odometry를 받아 local 및 누적 2D occupancy grid를 만드는 독립 ROS 2 패키지입니다. C++ mapper, MID-360 bag/IMU/odometry 실행 구성, 두 로봇 정렬과 기록 재게시, 선택적 Rerun 시각화, 설정 파일, RViz 구성, 단위 테스트를 포함합니다.

점유 상태는 다음 규칙을 따릅니다. 미관측 셀은 `-1`, raycast로 빈 공간임을 확인한 셀은 `0`, 장애물 hit 셀은 `100`입니다. 관측하지 않은 영역을 빈 공간으로 바꾸지 않습니다.

## 사전 요구 사항

새 장비는 Ubuntu 22.04와 ROS 2 Humble을 기준으로 합니다. ROS 2 Humble은 [공식 설치 안내](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)에 따라 설치한 뒤, workspace에 필요한 도구를 설치합니다.

```bash
sudo apt update
sudo apt install -y python3-rosdep python3-colcon-common-extensions
sudo rosdep init  # 최초 한 번만 실행
rosdep update

mkdir -p /path/to/ros2_ws/src
cd /path/to/ros2_ws/src
git clone https://github.com/Epsilon8854/co-3dto2d-mapping.git co_3dto2d_mapping
```

패키지 manifest에는 `rtabmap_odom`, `imu_filter_madgwick`, `rviz2`를 포함한 ROS 의존성이 선언되어 있습니다. 아래의 `rosdep` 명령으로 현재 ROS 배포판에 맞는 의존성을 설치합니다. Rerun용 Python 패키지와 `rerun` viewer는 선택 사항입니다.

### 셸 환경

새 셸에서는 ROS 배포판 하나만 source합니다. 새 Humble 장비에서는 다음 순서로 시작합니다.

```bash
bash --noprofile --norc
source /opt/ros/humble/setup.bash
```

같은 셸에서 ROS 1 setup 파일을 source하지 않습니다. workspace를 처음 빌드한 뒤에만 `/path/to/ros2_ws/install/local_setup.bash`를 source합니다.

현재 개발 호스트에는 Foxy와 Livox workspace가 설치되어 있습니다. 이 호스트에서는 다음 순서를 사용합니다.

```bash
bash --noprofile --norc
source /opt/ros/foxy/setup.bash
source /home/user/ws_livox/install/local_setup.bash
source /home/user/Swarm-SLAM/install/local_setup.bash
cd /home/user/Swarm-SLAM
```

이 호스트에서는 `~/.bashrc`에 의존하지 않고, 기본 `real_cslam` Conda 셸도 사용하지 않습니다. Foxy를 source한 뒤 workspace의 `setup.bash`를 다시 source하면 ROS 1 Noetic 라이브러리가 섞일 수 있으므로 `local_setup.bash`를 사용합니다.

## 빌드와 테스트

새 workspace의 루트에서 실행합니다.

```bash
cd /path/to/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select co_3dto2d_mapping
source install/local_setup.bash
colcon test --packages-select co_3dto2d_mapping
colcon test-result --verbose
```

테스트에는 occupancy 유틸리티, record-republisher 보조 함수, 시각화 보조 함수, 패키지와 launch의 정적 격리 검사가 포함됩니다. 실제 센서나 rosbag을 사용하는 통합 검사를 대신하지는 않습니다.

## Bag 요구 사항

`bag_path`에는 `metadata.yaml`이 있는 rosbag2 디렉터리를 지정합니다. 실행 코드는 빈 경로와 해당 파일이 없는 디렉터리를 거부합니다. 기본 storage plugin은 SQLite입니다.

```bash
ros2 bag info /path/to/bag
```

일반적인 SQLite bag은 `storage_id:=sqlite3`를 사용합니다. MCAP 환경에서는 `storage_id:=mcap`을 사용할 수 있지만, bag metadata와 설치된 ROS plugin이 현재 ROS 배포판과 호환되어야 합니다.

재생 파이프라인은 bag에서 `/livox/lidar`, `/livox/imu`, 선택적으로 `/tf_static`을 읽습니다. `/livox/imu`가 없는 bag으로는 제공된 MID-360 odometry 경로를 초기화할 수 없습니다.

## 로봇 한 대 실행

단일 파이프라인은 bag 재생, 선택적인 후방 LiDAR 영역 필터링, IMU 필터링 및 재게시, RTAB-Map odometry, occupancy mapper를 순서대로 실행합니다. 다음 명령에서 경로만 실제 bag 경로로 바꿉니다.

```bash
ros2 launch co_3dto2d_mapping single_bag_mapping.launch.py \
  bag_path:=/path/to/bag \
  storage_id:=sqlite3 \
  rate:=1.0 \
  use_sim_time:=true
```

기본 robot 이름은 `r0`입니다. 실제 MID-360을 사용할 때는 `base_link -> livox_frame` extrinsic을 측정해야 합니다. launch에 들어 있는 identity translation과 고정 roll 값은 시작점일 뿐, 보정된 센서 변환값이 아닙니다.

## Bag 두 개 실행

```bash
ros2 launch co_3dto2d_mapping two_bag_mapping.launch.py \
  bag_path_0:=/path/to/bag_robot_0 \
  bag_path_1:=/path/to/bag_robot_1 \
  storage_id:=sqlite3 \
  rate:=0.5 \
  robot_delay_s:=20.0
```

`/r0`과 `/r1` 파이프라인을 독립적으로 실행하고, 두 global occupancy grid에서 초기 XY 정렬을 계산합니다. 기본 설정에서는 기록에 사용할 수 있는 `/toy_record` 토픽도 재게시합니다. 각 map은 `r0/odom`과 `r1/odom`에서 독립적으로 누적되며, record republisher와 정렬 변환이 공통 `map` 시각화 구성을 만듭니다.

## 토픽과 프레임

| 구분 | 로봇 한 대 | 로봇 두 대 |
| --- | --- | --- |
| Bag LiDAR 입력 | `/livox/lidar_raw` 후 `/livox/lidar` | `/r0/livox/lidar_raw` / `/r1/livox/lidar_raw` 후 필터링된 `/rN/livox/lidar` |
| Bag IMU 입력 | `/livox/imu` 후 `/livox/imu_filtered` | `/rN/livox/imu` 후 `/rN/livox/imu_filtered` |
| Odometry | `/r0/odom` | `/r0/odom`, `/r1/odom` |
| Mapper 출력 | `/r0/toy/local_occupancy`, `/r0/toy/global_occupancy` | 각 `rN`에 대해 같은 두 토픽 |
| Slice 디버그 출력 | `/r0/toy/slice_kept_points`, `/r0/toy/slice_rejected_points` | 각 `rN`에 대해 같은 두 토픽 |
| 정렬 및 기록 | 기본적으로 사용하지 않음 | `/toy/initial_xy_alignment`, `/toy_record/rN/{odom,local_occupancy,global_occupancy,slice_kept_points,slice_rejected_points}`, 선택적 `/toy_record/merged_global_occupancy` |

단일 파이프라인의 기본 프레임은 `base_link`, `livox_frame`, `odom`입니다. 두 bag 파이프라인에서는 `r0/base_link`, `r0/livox_frame`, `r0/odom`과 `r1/base_link`, `r1/livox_frame`, `r1/odom`으로 prefix가 붙습니다. extrinsic을 바꿀 때 point-cloud frame, static transform, `local_frame_id`가 서로 일치하는지 확인합니다.

## RViz

두 bag 파이프라인을 실행한 뒤 포함된 레이아웃을 엽니다.

```bash
rviz2 -d "$(ros2 pkg prefix --share co_3dto2d_mapping)/rviz/two_robot_mapping.rviz"
```

저장된 RViz 레이아웃은 `/toy_record/*` 토픽을 사용하고 fixed frame을 `map`으로 설정합니다. 따라서 두 bag launch의 기본값인 record republisher를 켜 두고, `map` 공통 프레임과 TF publish를 유지해야 하며, `/toy/initial_xy_alignment`가 있어야 합니다. record republisher 없이 로봇별 map을 보려면 `/rN/toy/*` display를 직접 추가하고 해당 `rN/odom` 프레임을 사용합니다.

## 선택 사항: Rerun 시각화

외부 viewer가 필요한 경우에만 호환되는 `rerun-sdk`와 `rerun` 도구를 설치한 뒤 실행합니다.

```bash
ros2 launch co_3dto2d_mapping rerun_mapping.launch.py \
  spawn_viewer:=true \
  rerun_port:=9876
```

노드는 `/toy_record/*` 토픽을 구독합니다. 호환되는 viewer를 이미 실행했다면 `spawn_viewer:=false`로 설정합니다. 기본 mapping에는 Rerun이 필요하지 않습니다.

## 설정

선언된 mapper 및 Python 노드 parameter, YAML 기본값, launch 인자 override, 단위, validation, occupied-hit range와 free-space raycast range의 차이는 [`docs/parameters.md`](docs/parameters.md)에서 확인할 수 있습니다.

## 과거 문서와의 관계

소스 monorepo의 과거 문서인 `docs/toy-occupancy-map.md`는 Foxy 시절의 `cslam_experiments` / `cslam_visualization` 경로를 설명합니다. 참고용으로만 두며, 이 standalone 저장소에는 해당 monorepo 문서가 포함되어 있지 않습니다. 이 저장소에서는 이 README와 `launch/`, `config/`, [`docs/parameters.md`](docs/parameters.md)에 적힌 패키지명과 명령을 사용합니다.

## 문제 해결

- **`bag metadata not found` 또는 `bag_path is required`:** 상위 디렉터리나 database 파일이 아니라 rosbag2 디렉터리 자체를 지정하고, `/path/to/bag/metadata.yaml`이 있는지 확인합니다.
- **Odometry 또는 IMU 초기화가 되지 않음:** `ros2 bag info /path/to/bag`로 내용을 확인합니다. 제공된 파이프라인에는 `/livox/imu`가 필요합니다. bag의 실제 IMU 토픽명이 다를 때만 `imu_raw_topic`을 바꿉니다.
- **Library 오류 또는 ROS 1 메시지가 나타남:** 새 `bash --noprofile --norc` 셸을 열고 Humble setup만 source하거나, 이 호스트에서는 위의 Foxy 순서를 그대로 사용합니다.
- **Cloud-to-base TF 경고 또는 map의 회전·위치 오차:** `sensor_tf_x`, `sensor_tf_y`, `sensor_tf_z`, `sensor_tf_yaw`, `sensor_tf_pitch`, `sensor_tf_roll`을 측정값으로 설정합니다. 기본 transform을 실제 MID-360 보정값으로 사용하지 않습니다.
