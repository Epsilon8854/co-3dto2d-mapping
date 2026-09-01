# co_3dto2d_mapping

이미 설정된 Livox MID-360의 point cloud와 IMU를 받아 odometry와 2D occupancy grid를 만드는 ROS 2 패키지입니다. C++ mapper, 단일·두 로봇 live mapping, rosbag2 재생, 두 로봇 정렬과 기록 재게시, 단위 테스트를 포함합니다.

이 README는 ROS 2와 Livox 입력 토픽이 이미 준비된 환경에서 단일 로봇 또는 두 로봇 mapping을 실행하는 흐름을 설명합니다. rosbag2는 센서가 없을 때 사용하는 선택 사항입니다. 현재 다중 로봇 구현 범위는 `r0`, `r1`의 **2대**이며, 3대 이상의 임의 N대 구성은 지원하지 않습니다.

## 0. 사전 세팅

이 문서는 ROS 2 Humble과 Livox MID-360이 이미 설치 및 설정되어 있다고 가정합니다. 설치가 되어 있지 않다면 아래 링크를 참고하여 먼저 설치해 주세요.

- [ROS 2 Humble 설치 안내](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [Livox-SDK2 설치 안내](https://github.com/Livox-SDK/Livox-SDK2)
- [Livox ROS Driver 2 설치 안내](https://github.com/Livox-SDK/livox_ros_driver2)

설치와 네트워크 설정이 끝난 뒤, 단일 로봇 live mode에서는 Livox driver가 `/livox/lidar`와 `/livox/imu`를 publish하는 상태여야 합니다. 두 로봇 live mode에서는 두 driver가 `/r0/livox/*`, `/r1/livox/*`처럼 서로 구분되는 LiDAR·IMU 토픽을 publish해야 합니다.

## 실행 구조

| 구성 요소 | 역할 |
| --- | --- |
| `co_3dto2d_mapping` | 센서 토픽을 받아 IMU filtering, RTAB-Map ICP odometry, occupancy mapping을 실행합니다. |
| `live_mapping.launch.py` | 한 대의 live 센서로 mapping pipeline을 실행합니다. 기본 `use_sim_time`은 `false`입니다. |
| `two_live_mapping.launch.py` | 두 대의 live 센서를 `/r0`, `/r1` pipeline에 연결하고 초기 XY ICP 정렬, 공통 `map` TF, `/toy_record` 재게시와 merged occupancy를 실행합니다. |
| `single_bag_mapping.launch.py` | rosbag2 한 개를 재생하며 mapping pipeline을 실행합니다. bag mode는 선택 사항입니다. |
| `two_bag_mapping.launch.py` | rosbag2 두 개를 `/r0`, `/r1`로 재생하고 두 로봇의 초기 XY 정렬과 결과 병합을 실행합니다. |

단일 로봇 live 입력은 기본적으로 `/livox/lidar`, `/livox/imu`를 사용합니다. 두 로봇 live 입력은 로봇별로 분리된 네 개의 절대 토픽을 사용하며, 기본값은 `/r0/livox/lidar`, `/r0/livox/imu`, `/r1/livox/lidar`, `/r1/livox/imu`입니다.

## 주요 토픽과 frame

| 구분 | 단일 Live | 두 로봇 Live | Bag 한 개 |
| --- | --- | --- | --- |
| LiDAR 입력 | `/livox/lidar` | `/r0/livox/lidar`, `/r1/livox/lidar` | bag의 `/livox/lidar`를 재생 후 `/livox/lidar` 또는 `/livox/lidar_raw` |
| IMU 입력 | `/livox/imu` | `/r0/livox/imu`, `/r1/livox/imu` | bag의 `/livox/imu`를 재생 후 `/livox/imu` |
| Mapping LiDAR | 입력 토픽 사용 | `/r0/mapping/lidar`, `/r1/mapping/lidar` | 재생·필터 설정에 따름 |
| Filtered IMU | `/livox/imu_filtered` | `/r0/mapping/imu_filtered`, `/r1/mapping/imu_filtered` | `/livox/imu_filtered` |
| Odometry | `/r0/odom` | `/r0/odom`, `/r1/odom` | `/r0/odom` |
| Occupancy | `/r0/toy/local_occupancy`, `/r0/toy/global_occupancy` | `/r0/toy/*`, `/r1/toy/*` | 단일 Live와 같은 토픽 |
| 정렬·병합 | - | `/toy/initial_xy_alignment`, `/toy_record/merged_global_occupancy` | - |
| Frame | `base_link`, `livox_frame`, `odom` | `map -> rN/odom -> rN/base_link -> rN/livox_frame` | `base_link`, `livox_frame`, `odom` |

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

두 로봇 live mode에는 다음 조건이 추가됩니다.

- 네 입력 토픽은 `/`로 시작하는 절대 이름이어야 하며 서로 중복되면 안 됩니다. launch가 중복 토픽, 내부 `/rN/mapping/*` 토픽 또는 재게시 순환 연결을 감지하면 종료합니다.
- 두 driver가 이미 동일한 `/livox/lidar`, `/livox/imu`에 데이터를 섞어 publish한 뒤에는 launch 인자만으로 로봇별 메시지를 다시 분리할 수 없습니다. driver의 namespace 또는 remap을 먼저 설정해야 합니다.
- 외부 `robot_state_publisher`나 driver가 TF를 publish한다면 `r0/base_link`, `r0/livox_frame`, `r1/base_link`, `r1/livox_frame`처럼 frame을 로봇별로 분리합니다. 동일 static TF를 외부에서 제공할 때는 `publish_sensor_static_tf:=false`를 사용합니다.

## 2. Live mode 실행

### 2.1 단일 로봇 live mode

#### 터미널 A: Livox MID-360 실행

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

기본 입력 토픽은 LiDAR `/livox/lidar`, IMU `/livox/imu`입니다. 실제 토픽 이름이 다르면 mapping launch에 실제 이름을 넘깁니다.

```bash
ros2 launch co_3dto2d_mapping live_mapping.launch.py \
  scan_cloud_topic:=/actual/lidar_topic \
  imu_raw_topic:=/actual/imu_topic
```

#### 터미널 B: mapping

```bash
source /opt/ros/humble/setup.bash
source ~/co_3dto2d_mapping/install/local_setup.bash

ros2 launch co_3dto2d_mapping live_mapping.launch.py
```

### 2.2 두 로봇 live mode

`two_live_mapping.launch.py`는 기존 두 bag mode와 같은 `/r0`, `/r1` mapping pipeline, 초기 XY ICP 정렬, `/toy_record` 재게시와 merged occupancy 구성을 실시간 LiDAR·IMU 입력에 연결합니다.

#### 센서 토픽 준비

두 Livox driver를 다음 기본 토픽 또는 이에 대응하는 서로 다른 절대 토픽으로 설정합니다.

| 로봇 | LiDAR | IMU |
| --- | --- | --- |
| r0 | `/r0/livox/lidar` | `/r0/livox/imu` |
| r1 | `/r1/livox/lidar` | `/r1/livox/imu` |

#### 로봇별 실제 실행 명령

아래에서는 **물리 로봇 1을 `r0`**, **물리 로봇 2를 `r1`**로 사용합니다. 두 로봇 PC와 mapping PC는 같은 `ROS_DOMAIN_ID`를 사용해야 하며, 다른 PC의 토픽도 검색할 수 있도록 `ROS_LOCALHOST_ONLY=0`으로 둡니다. ROS 배포판이나 Livox workspace 경로가 다르면 각 PC의 실제 경로로 바꿉니다.

각 로봇의 `MID360_config.json`에는 그 로봇에 연결된 MID-360과 로봇 PC의 실제 IP를 설정해야 합니다. `run_livox_robot.sh`는 로봇 번호 하나만 받아 Livox driver를 실행합니다. 물리 로봇 번호 `1`은 `r0`, 번호 `2`는 `r1`으로 변환되며, `xfer_format:=0`과 로봇별 node·frame·LiDAR·IMU remap은 스크립트가 자동으로 지정합니다. 기본 `ROS_DOMAIN_ID`는 `72`, `ROS_LOCALHOST_ONLY`는 `0`입니다.

물리 로봇 1 PC (`r0`)에서 실행:

```bash
cd ~/co_3dto2d_mapping
bash scripts/run_livox_robot.sh 1
```

물리 로봇 2 PC (`r1`)에서 실행:

```bash
cd ~/co_3dto2d_mapping
bash scripts/run_livox_robot.sh 2
```

패키지를 빌드하고 `install/local_setup.bash`를 source한 환경에서는 같은 스크립트를 다음처럼 실행할 수도 있습니다.

```bash
ros2 run co_3dto2d_mapping run_livox_robot.sh 1  # 물리 로봇 1
ros2 run co_3dto2d_mapping run_livox_robot.sh 2  # 물리 로봇 2
```

기본 경로와 다른 환경만 `ROS_SETUP`, `LIVOX_SETUP`, `LIVOX_CONFIG` 환경 변수로 덮어씁니다. 위치 인자로는 로봇 번호 외의 값을 받지 않습니다.

두 driver가 올라온 뒤 mapping PC에서 실행:

```bash
source /opt/ros/humble/setup.bash
source ~/co_3dto2d_mapping/install/local_setup.bash

export ROS_DOMAIN_ID=72
export ROS_LOCALHOST_ONLY=0

ros2 launch co_3dto2d_mapping two_live_mapping.launch.py
```

두 로봇이 서로 다른 노트북에 연결된 경우 두 노트북에서 같은 스크립트를
실행합니다. 각 스크립트는 해당 노트북의 Livox driver만 시작하며, 두 노트북
중 정확히 한 곳에만 `--mapping-host`를 지정해 공통 mapping과 RViz도 함께
실행합니다.

```bash
# 물리 로봇 1 노트북: r0 driver
cd ~/co_3dto2d_mapping
bash scripts/run_two_mid360_2d_mapping.sh --robot-number 1

# 물리 로봇 2 노트북: r1 driver + 두 로봇 mapping + RViz
cd ~/co_3dto2d_mapping
bash scripts/run_two_mid360_2d_mapping.sh --robot-number 2 --mapping-host
```

`--mapping-host`는 어느 노트북에 지정해도 되지만 두 곳에서 동시에 지정하면
같은 mapping node와 출력 토픽이 중복되므로 한 곳에서만 사용합니다.
기존 배포 구조의 `aibot/bash/mid360.env`가 있으면 `ROBOT_ID`를 자동으로
읽으므로 `--robot-number`를 생략할 수 있으며, 다른 환경 파일은
`MID360_ENV_FILE=/path/to/mid360.env`로 지정할 수 있습니다.
Livox workspace는 저장소 옆의 `ws_livox`, `~/aibot/livox_mid360/ws_livox`,
`~/ws_livox`, `~/livox_ws` 순서로 탐색하며, 다른 위치라면
`--livox-workspace /path/to/ws_livox`를 지정합니다.

```bash
export ROS_DOMAIN_ID=72
export ROS_LOCALHOST_ONLY=0
source /opt/ros/humble/setup.bash
source ~/co_3dto2d_mapping/install/local_setup.bash

rviz2 -d "$(ros2 pkg prefix --share co_3dto2d_mapping)/rviz/two_robot_mapping.rviz"
```

```bash
ros2 topic list | grep -E '^/r[01]/livox/(lidar|imu)$'
ros2 topic hz /r0/livox/lidar
ros2 topic hz /r1/livox/lidar
ros2 topic echo /r0/livox/imu --once
ros2 topic echo /r1/livox/imu --once
```

여러 PC에서 driver와 mapping을 나누어 실행할 때는 DDS 통신이 가능하고 `ROS_DOMAIN_ID`가 같아야 합니다. ICP 입력의 시간 차이를 줄이도록 chrony 또는 NTP로 시스템 시각도 동기화합니다.

#### 두 로봇 mapping 실행

기본 토픽을 사용할 때:

```bash
source /opt/ros/humble/setup.bash
source ~/co_3dto2d_mapping/install/local_setup.bash

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

주요 출력은 다음과 같습니다.

```text
/r0/odom
/r1/odom
/r0/toy/local_occupancy
/r1/toy/local_occupancy
/r0/toy/global_occupancy
/r1/toy/global_occupancy
/toy/initial_xy_alignment
/toy_record/r0/*
/toy_record/r1/*
/toy_record/merged_global_occupancy
```

기본 TF 트리는 `map -> r0/odom -> r0/base_link -> r0/livox_frame`과 `map -> r1/odom -> r1/base_link -> r1/livox_frame`입니다. r1의 `map` 정렬은 두 global occupancy에 충분한 공통 구조가 있고 ICP가 유효한 결과를 얻은 뒤 publish됩니다. 반복적이거나 대칭적인 환경에서는 `/toy/initial_xy_alignment`와 병합 지도를 반드시 확인합니다.

외부 `robot_state_publisher` 또는 sensor driver가 이미 로봇별 static TF를 제공하면 중복 발행을 끕니다.

```bash
ros2 launch co_3dto2d_mapping two_live_mapping.launch.py \
  publish_sensor_static_tf:=false
```

센서 장착 자세, 후방 LiDAR filter, ICP 조건, 외부 static TF 및 상세 점검 방법은 [`docs/two_live_mode.md`](docs/two_live_mode.md)를 참고합니다.

## 3. RViz

### 단일 로봇 live mode

```bash
rviz2 -d "$(ros2 pkg prefix --share co_3dto2d_mapping)/rviz/single_robot_mapping.rviz"
```

### 두 로봇 live mode 및 bag mode

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

두 bag launch는 `/r0`과 `/r1` pipeline을 실행하고 초기 XY 정렬을 계산합니다. 기본 설정에서는 `/toy_record/*` 토픽을 publish하며, 결과는 [두 로봇 RViz](#두-로봇-live-mode-및-bag-mode)에서 확인할 수 있습니다.

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
