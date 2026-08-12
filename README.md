# co_3dto2d_mapping

이미 설정된 Livox MID-360의 point cloud와 IMU를 받아 odometry와 2D occupancy grid를 만드는 ROS 2 패키지입니다. C++ mapper, live mapping launch, rosbag2 재생 launch, 두 로봇 정렬과 기록 재게시, 단위 테스트를 포함합니다.

이 README는 ROS 2와 Livox 입력 토픽이 이미 준비된 환경에서 mapping 패키지를 실행하는 흐름을 설명합니다. rosbag2는 센서가 없을 때 사용하는 선택 사항입니다.

## 0. 사전 세팅

이 문서는 ROS 2 Humble과 Livox MID-360이 이미 설치 및 설정되어 있다고 가정합니다. 설치가 되어 있지 않다면 아래 링크를 참고하여 먼저 설치해 주세요.

- [ROS 2 Humble 설치 안내](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [Livox-SDK2 설치 안내](https://github.com/Livox-SDK/Livox-SDK2)
- [Livox ROS Driver 2 설치 안내](https://github.com/Livox-SDK/livox_ros_driver2)

설치와 네트워크 설정이 끝난 뒤, live mode에서는 Livox driver가 `/livox/lidar`와 `/livox/imu`를 publish하는 상태여야 합니다.

## 실행 구조

| 구성 요소 | 역할 |
| --- | --- |
| `co_3dto2d_mapping` | 센서 토픽을 받아 IMU filtering, RTAB-Map ICP odometry, occupancy mapping을 실행합니다. |
| `live_mapping.launch.py` | bag 없이 위 mapping pipeline을 실행합니다. 기본 `use_sim_time`은 `false`입니다. |
| `single_bag_mapping.launch.py` | rosbag2를 재생하며 mapping pipeline을 실행합니다. bag mode는 선택 사항입니다. |

Livox 입력은 `/livox/lidar`와 `/livox/imu`를 사용한다고 가정합니다. mapping 패키지와 rosbag 파일만 이 저장소에서 관리합니다.

## 주요 토픽과 frame

| 구분 | Live mode | Bag 한 개 |
| --- | --- | --- |
| LiDAR 입력 | `/livox/lidar` | bag의 `/livox/lidar`를 재생 후 `/livox/lidar` 또는 `/livox/lidar_raw` |
| IMU 입력 | `/livox/imu` | bag의 `/livox/imu`를 재생 후 `/livox/imu` |
| Filtered IMU | `/livox/imu_filtered` | `/livox/imu_filtered` |
| Odometry | `/r0/odom` | `/r0/odom` |
| Occupancy | `/r0/toy/local_occupancy`, `/r0/toy/global_occupancy` | 같은 토픽 |
| Frame | `base_link`, `livox_frame`, `odom` | `base_link`, `livox_frame`, `odom` |

## 1. mapping 패키지 clone과 빌드

이 저장소를 clone하고 저장소 루트에서 직접 빌드합니다.

```bash
git clone https://github.com/Epsilon8854/co-3dto2d-mapping.git ~/co_3dto2d_mapping
cd ~/co_3dto2d_mapping

# rosdep을 아직 초기화하지 않은 노트북에서만 sudo rosdep init을 실행합니다.
# 이미 초기화되어 있으면 이 줄을 건너뜁니다.
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install --packages-select co_3dto2d_mapping
source install/local_setup.bash
ros2 pkg prefix co_3dto2d_mapping
```

`colcon`이 만드는 `build/`, `install/`, `log/`는 저장소의 `.gitignore`에 포함되어 있습니다. 새 셸에서는 다음 순서로 ROS 2와 mapping workspace를 source합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/co_3dto2d_mapping/install/local_setup.bash
```

## 주의사항

현재 기본 설정에는 다음 두 가지 가정이 들어 있습니다.

- 센서를 중심으로 `160 cm × 160 cm` 정사각형 영역을 제거합니다. 이는 `center_box_filter_half_extent_m=0.80`에 해당하며, 필터 크기는 이 값을 조정해 바꿀 수 있습니다. `0`으로 설정하면 중앙 필터를 끕니다.
- MID-360가 위아래가 뒤집힌 상태로 장착되었다고 가정해 `config/occupancy.yaml`에서 `invert_z_slice: true`를 사용합니다. 일반 방향으로 장착했다면 해당 값을 `false`로 바꾸세요. `invert_z_slice`는 좌표계 자체를 뒤집는 값이 아니라 Z slice의 안쪽/바깥쪽 선택을 반대로 합니다.

## 2. Live mode 실행

### 터미널 A: Livox MID-360 실행

Livox workspace가 `~/ws_livox`에 설치되어 있다고 가정합니다. 경로가 다르면 `local_setup.bash` 경로를 실제 Livox workspace에 맞게 바꿉니다.

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/local_setup.bash

ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

이 launch는 MID-360 driver를 시작하고 mapping에 필요한 `sensor_msgs/msg/PointCloud2` 형식의 `/livox/lidar`를 publish합니다. Livox driver에 포함된 센서 확인용 RViz도 함께 열리므로, 아래의 mapping RViz를 사용할 때는 해당 창을 닫아도 됩니다.

```bash
ros2 topic list | grep -E '/livox/(lidar|imu)'
ros2 topic type /livox/lidar
ros2 topic type /livox/imu
ros2 topic hz /livox/lidar
ros2 topic echo /livox/imu --once
```

기본 입력 토픽은 LiDAR `/livox/lidar`, IMU `/livox/imu`입니다. 실제 토픽 이름이 다르면 mapping launch에 실제 이름을 넘깁니다. 예를 들면:

```bash
ros2 launch co_3dto2d_mapping live_mapping.launch.py \
  scan_cloud_topic:=/actual/lidar_topic \
  imu_raw_topic:=/actual/imu_topic
```

### 터미널 B: mapping

```bash
source /opt/ros/humble/setup.bash
source ~/co_3dto2d_mapping/install/local_setup.bash

ros2 launch co_3dto2d_mapping live_mapping.launch.py
```

## 3. RViz

### 단일 로봇 live mode

```bash
rviz2 -d "$(ros2 pkg prefix --share co_3dto2d_mapping)/rviz/single_robot_mapping.rviz"
```

### 다중 로봇 bag mode

```bash
rviz2 -d "$(ros2 pkg prefix --share co_3dto2d_mapping)/rviz/two_robot_mapping.rviz"
```

## 4. Bag mode (선택 사항)

저장된 rosbag2로 mapping을 검증할 때 bag mode를 사용합니다. Bag mode에는 Livox driver가 필요하지 않습니다.

Bag 디렉터리에는 `metadata.yaml`이 있어야 합니다. 기본 입력 토픽은 LiDAR `/livox/lidar`, IMU `/livox/imu`이며, `/tf_static`은 선택 사항입니다.

```bash
source /opt/ros/humble/setup.bash
source ~/co_3dto2d_mapping/install/local_setup.bash
```

```bash
ros2 bag info /path/to/mid360_run
```

### Bag 한 개

```bash
ros2 launch co_3dto2d_mapping single_bag_mapping.launch.py \
  use_bag:=true \
  bag_path:=/path/to/mid360_run \
  storage_id:=sqlite3 \
  rate:=1.0 \
  use_sim_time:=true
```

Bag 실행에서는 `bag_path`, `rate`, `storage_id`, `use_sim_time`을 주로 조정합니다. 별도 occupancy 설정 파일을 사용할 때는 다음처럼 `occupancy_config_file`을 추가합니다.

```bash
ros2 launch co_3dto2d_mapping single_bag_mapping.launch.py \
  use_bag:=true \
  bag_path:=/path/to/mid360_run \
  occupancy_config_file:=/absolute/path/to/occupancy.yaml \
  rate:=1.0 \
  storage_id:=sqlite3 \
  use_sim_time:=true
```

### Bag 두 개

```bash
ros2 launch co_3dto2d_mapping two_bag_mapping.launch.py \
  bag_path_0:=/path/to/mid360_robot_0 \
  bag_path_1:=/path/to/mid360_robot_1 \
  storage_id:=sqlite3 \
  rate:=0.5 \
  robot_delay_s:=20.0
```

두 bag launch는 `/r0`과 `/r1` pipeline을 실행하고 초기 XY 정렬을 계산합니다. 기본 설정에서는 `/toy_record/*` 토픽을 publish하며, 결과는 [다중 로봇 RViz](#다중-로봇-bag-mode)에서 확인할 수 있습니다.

두 bag 실행에서는 `robot_delay_s`로 두 번째 bag의 시작 지연을 조정하고, `alignment_*`로 초기 XY 정렬 조건을 조정합니다. merged map을 저장하거나 output prefix를 바꾸려면 다음 옵션을 사용합니다.

```bash
ros2 launch co_3dto2d_mapping two_bag_mapping.launch.py \
  bag_path_0:=/path/to/mid360_robot_0 \
  bag_path_1:=/path/to/mid360_robot_1 \
  robot_delay_s:=20.0 \
  alignment_voxel_size:=0.05 \
  alignment_min_fitness:=0.05 \
  alignment_max_rmse:=0.40 \
  record_output_prefix:=/toy_record \
  record_publish_merged_global:=true
```

두 bag launch는 두 재생을 wall clock으로 지연시키므로 `use_sim_time:=true`를 추가하지 않습니다. bag 하나를 먼저 단독으로 검증한 다음 두 bag 정렬을 실행합니다.

Bag 안의 source topic 이름이 `/livox/lidar` 또는 `/livox/imu`와 다르면 launch remap 인자만으로는 바꿀 수 없습니다. 먼저 bag을 변환하거나 `launch/bag_mid360.launch.py`의 source topic을 수정해야 합니다.

## 5. Hyperparameters

기본 occupancy 설정은 [`config/occupancy.yaml`](config/occupancy.yaml)에 있습니다. 자주 조정하는 핵심 값과 나머지 파라미터의 기본값은 [`docs/parameters.md`](docs/parameters.md)에서 확인하세요. 별도 설정 파일을 사용하려면 YAML 파일을 복사해 수정한 뒤 `occupancy_config_file:=/absolute/path/to/occupancy.yaml`로 지정합니다.

```yaml
/**:
  ros__parameters:
    grid_resolution: 0.05
    local_map_size_m: 20.0
    z_min: 0.4
    z_max: 0.8
    invert_z_slice: true
    center_box_filter_half_extent_m: 0.80
    range_min_m: 0.80
    range_max_m: 12.0
    enable_raycast_free_space: true
    raycast_min_range_m: 0.80
    raycast_max_range_m: 12.0
    raycast_clear_occupied: false
    occupied_threshold_points: 1
```

특히 다음 두 범위는 서로 다른 역할을 합니다.

- `range_min_m` / `range_max_m`: 장애물 hit endpoint에 적용되는 범위
- `raycast_min_range_m` / `raycast_max_range_m`: sensor origin에서 빈 공간을 추적하는 raycast 범위

occupancy 값은 미관측 `-1`, free `0`, occupied `100`입니다. 미관측 영역은 free로 바꾸지 않습니다.
