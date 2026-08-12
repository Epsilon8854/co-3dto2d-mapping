# co_3dto2d_mapping

Livox MID-360 포인트 클라우드와 odometry를 받아 2D occupancy grid를 만드는 ROS 2 패키지입니다. C++ mapper, rosbag2 재생 launch, 두 로봇 정렬과 기록 재게시, RViz 구성, 단위 테스트를 포함합니다.

이 README는 새 Ubuntu 22.04 노트북에 ROS 2 Humble만 설치되어 있고, 기존 workspace가 없는 상황을 기준으로 작성했습니다. 저장소를 clone한 뒤 저장소 루트에서 바로 의존성 설치와 빌드를 진행합니다.

## 실행 범위

이 저장소에 포함된 기본 실행 경로는 rosbag2 재생입니다.

- 저장소에 Livox 드라이버나 rosbag 파일은 포함되어 있지 않습니다.
- 저장소만 clone하면 패키지 빌드와 테스트를 실행할 수 있습니다.
- occupancy map을 만들려면 `/livox/lidar`와 `/livox/imu` 토픽이 들어 있는 rosbag2가 필요합니다.
- 실제 MID-360을 직접 연결하는 경로는 별도 Livox ROS 2 드라이버와 센서 설정이 필요합니다. 이 저장소의 launch가 드라이버를 설치하거나 실행하지는 않습니다.

## 1. ROS 2 설치

Ubuntu 22.04에 [ROS 2 Humble 공식 설치 안내](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)에 따라 ROS 2 Desktop을 설치합니다. 설치가 끝난 새 셸에서 ROS 2 환경을 확인합니다.

```bash
bash --noprofile --norc
source /opt/ros/humble/setup.bash
ros2 --help
```

한 셸에서 ROS 1과 ROS 2를 함께 source하지 않습니다. 이 README의 명령은 ROS 2 Humble을 기준으로 하므로 다른 배포판을 사용한다면 `/opt/ros/humble` 경로와 패키지 이름을 해당 배포판에 맞춰 바꿉니다.

## 2. 저장소 clone

별도 workspace를 먼저 만들 필요 없이 저장소 자체를 clone합니다.

```bash
sudo apt update
sudo apt install -y git python3-rosdep python3-colcon-common-extensions

git clone https://github.com/Epsilon8854/co-3dto2d-mapping.git ~/co_3dto2d_mapping
cd ~/co_3dto2d_mapping
```

`colcon`이 만드는 `build/`, `install/`, `log/` 디렉터리는 저장소의 `.gitignore`에 포함되어 있습니다.

## 3. 의존성 설치와 빌드

`rosdep`을 처음 사용하는 노트북에서만 `rosdep init`을 실행합니다.

```bash
sudo rosdep init
rosdep update

cd ~/co_3dto2d_mapping
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install --packages-select co_3dto2d_mapping
source install/local_setup.bash
```

`package.xml`에 선언된 ROS 의존성은 `rosdep`이 현재 설치된 ROS 배포판에 맞춰 설치합니다. 빌드가 끝난 뒤 새 셸을 열 때마다 다음 두 파일을 순서대로 source합니다.

```bash
bash --noprofile --norc
source /opt/ros/humble/setup.bash
source ~/co_3dto2d_mapping/install/local_setup.bash
```

## 4. 테스트

저장소 루트에서 실행합니다.

```bash
cd ~/co_3dto2d_mapping
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon test --packages-select co_3dto2d_mapping
colcon test-result --verbose
```

테스트에는 occupancy 유틸리티, record-republisher와 시각화 보조 함수, launch 및 패키지 격리 검사가 포함됩니다.

## 5. Bag 준비

`bag_path`에는 `metadata.yaml`이 있는 rosbag2 디렉터리를 지정합니다. 예를 들어 bag을 `~/bags/mid360_run`에 둔 경우 다음 명령으로 내용을 확인합니다.

```bash
ros2 bag info ~/bags/mid360_run
```

기본 재생 경로는 bag 안에 다음 토픽이 있어야 합니다.

- `/livox/lidar`
- `/livox/imu`
- `/tf_static` (선택 사항)

기본 storage plugin은 SQLite이므로 일반적인 bag은 `storage_id:=sqlite3`를 사용합니다. MCAP bag은 호환되는 ROS plugin이 설치되어 있을 때 `storage_id:=mcap`을 사용합니다.

## 6. 로봇 한 대 실행

새 셸에서 ROS 2와 이 저장소의 install 환경을 source한 뒤 실행합니다.

```bash
cd ~/co_3dto2d_mapping
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 launch co_3dto2d_mapping single_bag_mapping.launch.py \
  bag_path:=/path/to/mid360_run \
  storage_id:=sqlite3 \
  rate:=1.0 \
  use_sim_time:=true
```

기본 robot 이름은 `r0`입니다. 단일 pipeline은 bag 재생, 후방 LiDAR 영역 필터링, IMU 필터링 및 재게시, RTAB-Map odometry, occupancy mapper를 실행합니다.

실제 MID-360의 `base_link -> livox_frame` extrinsic은 측정값으로 바꿔야 합니다. launch의 기본 translation과 roll은 실행을 위한 시작값이며 센서 보정값이 아닙니다.

## 7. Bag 두 개 실행

두 rosbag을 동시에 사용하려면 다음과 같이 실행합니다.

```bash
cd ~/co_3dto2d_mapping
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 launch co_3dto2d_mapping two_bag_mapping.launch.py \
  bag_path_0:=/path/to/mid360_robot_0 \
  bag_path_1:=/path/to/mid360_robot_1 \
  storage_id:=sqlite3 \
  rate:=0.5 \
  robot_delay_s:=20.0
```

이 launch는 `/r0`과 `/r1` pipeline을 실행하고, 두 global occupancy grid에서 초기 XY 정렬을 계산합니다. 기본 설정에서는 RViz와 기록에 사용할 `/toy_record/*` 토픽도 재게시합니다.

## 토픽과 프레임

| 구분 | 로봇 한 대 | 로봇 두 대 |
| --- | --- | --- |
| Bag LiDAR 입력 | `/livox/lidar_raw` 후 `/livox/lidar` | `/r0/livox/lidar_raw`, `/r1/livox/lidar_raw` 후 `/rN/livox/lidar` |
| Bag IMU 입력 | `/livox/imu` 후 `/livox/imu_filtered` | `/rN/livox/imu` 후 `/rN/livox/imu_filtered` |
| Odometry | `/r0/odom` | `/r0/odom`, `/r1/odom` |
| Mapper 출력 | `/r0/toy/local_occupancy`, `/r0/toy/global_occupancy` | 각 `rN`에 대해 같은 두 토픽 |
| Slice 디버그 출력 | `/r0/toy/slice_kept_points`, `/r0/toy/slice_rejected_points` | 각 `rN`에 대해 같은 두 토픽 |
| 정렬 및 기록 | 기본적으로 사용하지 않음 | `/toy/initial_xy_alignment`, `/toy_record/rN/{odom,local_occupancy,global_occupancy,slice_kept_points,slice_rejected_points}`, 선택적 `/toy_record/merged_global_occupancy` |

단일 pipeline의 기본 frame은 `base_link`, `livox_frame`, `odom`입니다. 두 bag pipeline은 `r0/base_link`, `r0/livox_frame`, `r0/odom`과 `r1/base_link`, `r1/livox_frame`, `r1/odom`을 사용합니다. extrinsic을 바꿀 때 point-cloud frame, static transform, `local_frame_id`가 서로 일치하는지 확인합니다.

## RViz

두 bag pipeline을 실행한 뒤 저장소에 포함된 RViz layout을 엽니다.

```bash
rviz2 -d "$(ros2 pkg prefix --share co_3dto2d_mapping)/rviz/two_robot_mapping.rviz"
```

이 layout은 `/toy_record/*` 토픽과 fixed frame `map`을 사용합니다. 따라서 두 bag launch의 기본값인 record republisher를 켜고 `/toy/initial_xy_alignment`가 publish되는지 확인합니다. record republisher 없이 로봇별 map을 보려면 `/rN/toy/*` display를 직접 추가하고 해당 `rN/odom` frame을 사용합니다.

## 선택 사항: Rerun 시각화

Rerun을 사용할 때만 호환되는 `rerun-sdk`와 `rerun` CLI를 설치합니다. 기본 mapping과 RViz 실행에는 Rerun이 필요하지 않습니다.

```bash
ros2 launch co_3dto2d_mapping rerun_mapping.launch.py \
  spawn_viewer:=true \
  rerun_port:=9876
```

호환되는 viewer를 이미 실행한 경우에는 `spawn_viewer:=false`로 설정합니다.

## 주요 Parameter

mapper와 Python node의 선언값, YAML 기본값, launch override, 단위, validation은 [`docs/parameters.md`](docs/parameters.md)에 정리되어 있습니다.

특히 다음 두 범위는 서로 다릅니다.

- `range_min_m` / `range_max_m`: 장애물 hit endpoint에 적용되는 범위
- `raycast_min_range_m` / `raycast_max_range_m`: sensor origin에서 빈 공간을 추적하는 raycast 범위

occupancy 값은 미관측 `-1`, free `0`, occupied `100`입니다. 미관측 영역을 free로 바꾸지 않습니다.

## 문제 해결

- **`ros2: command not found`:** 새 `bash --noprofile --norc` 셸에서 `source /opt/ros/humble/setup.bash`를 실행합니다.
- **`Package 'co_3dto2d_mapping' not found`:** `source ~/co_3dto2d_mapping/install/local_setup.bash`를 실행하고, 필요하면 저장소 루트에서 다시 빌드합니다.
- **`rosdep init` 오류:** 이미 초기화된 노트북에서는 이 명령을 다시 실행하지 말고 `rosdep update`부터 실행합니다.
- **`bag metadata not found` 또는 `bag_path is required`:** database 파일이 아니라 `metadata.yaml`이 들어 있는 rosbag2 디렉터리 자체를 지정합니다.
- **Odometry 또는 IMU가 초기화되지 않음:** `ros2 bag info`로 `/livox/lidar`와 `/livox/imu`가 실제로 있는지 확인합니다. launch의 remap 인자는 bag 안의 source topic 이름을 바꾸지 않으므로, 다른 이름으로 기록된 bag은 먼저 변환하거나 launch를 수정해야 합니다.
- **Cloud-to-base TF 경고 또는 map의 회전·위치 오차:** `sensor_tf_x`, `sensor_tf_y`, `sensor_tf_z`, `sensor_tf_yaw`, `sensor_tf_pitch`, `sensor_tf_roll`에 측정한 extrinsic을 지정합니다.
- **ROS 1 library가 섞인 것처럼 보임:** 새 `bash --noprofile --norc` 셸을 열고 `/opt/ros/humble/setup.bash`와 이 저장소의 `install/local_setup.bash`만 순서대로 source합니다.
