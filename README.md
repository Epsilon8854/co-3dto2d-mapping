# co_3dto2d_mapping

이미 설정된 Livox MID-360의 point cloud와 IMU를 받아 odometry와 2D occupancy grid를 만드는 ROS 2 패키지입니다. C++ mapper, live mapping launch, rosbag2 재생 launch, 두 로봇 정렬과 기록 재게시, 단위 테스트를 포함합니다.

이 README는 ROS 2와 Livox 입력 토픽이 이미 준비된 환경에서 mapping 패키지를 실행하는 흐름을 설명합니다. rosbag2는 센서가 없을 때 사용하는 선택 사항입니다.

## 실행 구조

| 구성 요소 | 역할 |
| --- | --- |
| `co_3dto2d_mapping` | 센서 토픽을 받아 IMU filtering, RTAB-Map ICP odometry, occupancy mapping을 실행합니다. |
| `live_mapping.launch.py` | bag 없이 위 mapping pipeline을 실행합니다. 기본 `use_sim_time`은 `false`입니다. |
| `single_bag_mapping.launch.py` | rosbag2를 재생하며 mapping pipeline을 실행합니다. bag mode는 선택 사항입니다. |

Livox 입력은 `/livox/lidar`와 `/livox/imu`를 사용한다고 가정합니다. mapping 패키지와 rosbag 파일만 이 저장소에서 관리합니다.

## 1. ROS 2 Humble 설치

Ubuntu 22.04에 [ROS 2 Humble 공식 설치 안내](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)에 따라 ROS 2 Desktop을 설치합니다. 설치 후 새 셸에서 ROS 2 환경을 확인합니다.

```bash
bash --noprofile --norc
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
ros2 --help
```

같은 셸에서 ROS 1과 ROS 2를 함께 source하지 않습니다. 이 README의 명령은 ROS 2 Humble을 기준으로 합니다.

`PYTHONNOUSERSITE=1`은 `~/.local`에 `pip install --user`로 설치한 Python 패키지가 ROS Humble의 system Python 패키지를 덮어쓰지 않게 합니다. 이 저장소는 서로 맞지 않는 사용자 `setuptools`/`packaging` 조합이 있으면 빌드가 실패할 수 있으므로, 아래의 모든 ROS 셸에 이 값을 설정합니다.

기본 도구도 설치합니다.

```bash
sudo apt update
sudo apt install -y git build-essential cmake python3-rosdep python3-colcon-common-extensions
```

## 2. mapping 패키지 clone과 빌드

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

export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install --packages-select co_3dto2d_mapping
source install/local_setup.bash
ros2 pkg prefix co_3dto2d_mapping
```

`colcon`이 만드는 `build/`, `install/`, `log/`는 저장소의 `.gitignore`에 포함되어 있습니다. 새 셸을 열 때는 다음 순서로 두 workspace를 source합니다.

```bash
bash --noprofile --norc
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source ~/co_3dto2d_mapping/install/local_setup.bash
```

## 3. Live mode 실행

Livox driver는 이미 실행 중이며 `/livox/lidar`와 `/livox/imu`를 publish한다고 가정합니다. 먼저 입력 토픽과 message type을 확인합니다.

```bash
ros2 topic list | grep -E '/livox/(lidar|imu)'
ros2 topic type /livox/lidar
ros2 topic type /livox/imu
ros2 topic hz /livox/lidar
ros2 topic echo /livox/imu --once
```

기대 type은 각각 `sensor_msgs/msg/PointCloud2`, `sensor_msgs/msg/Imu`입니다. 실제 topic 이름이 다르면 mapping launch에 실제 이름을 넘깁니다. 예를 들면:

```bash
ros2 launch co_3dto2d_mapping live_mapping.launch.py \
  scan_cloud_topic:=/actual/lidar_topic \
  imu_raw_topic:=/actual/imu_topic
```

### 터미널 B: mapping

```bash
bash --noprofile --norc
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source ~/co_3dto2d_mapping/install/local_setup.bash

ros2 launch co_3dto2d_mapping live_mapping.launch.py
```

`live_mapping.launch.py`의 기본 동작은 다음과 같습니다.

- bag 재생을 하지 않습니다 (`use_bag=false`).
- `/livox/lidar`와 `/livox/imu`를 입력으로 사용합니다.
- `use_sim_time=false`로 실행합니다.
- rear-sector filter는 기본적으로 끕니다.
- `/r0/odom`, `/r0/toy/local_occupancy`, `/r0/toy/global_occupancy`를 publish합니다.

센서 장착 위치를 알고 있다면 측정한 `base_link -> livox_frame` extrinsic을 지정합니다.

```bash
ros2 launch co_3dto2d_mapping live_mapping.launch.py \
  sensor_tf_x:=0.0 \
  sensor_tf_y:=0.0 \
  sensor_tf_z:=0.0 \
  sensor_tf_yaw:=0.0 \
  sensor_tf_pitch:=0.0 \
  sensor_tf_roll:=3.141592653589793
```

위 값은 예시입니다. 실제 장착값을 사용해야 하며, 기본값을 보정값으로 간주하지 않습니다.

## 4. Live mode 확인

mapping 노드와 topic을 확인합니다.

```bash
ros2 node list | grep -E 'occupancy|odometry|imu'
ros2 topic list | grep -E 'livox|odom|occupancy'
ros2 topic hz /r0/odom
```

RViz를 별도로 실행하려면:

```bash
rviz2
```

RViz에서 Fixed Frame을 `odom`으로 설정하고 다음 display를 추가합니다.

- Map: `/r0/toy/global_occupancy`
- Map: `/r0/toy/local_occupancy`
- Odometry: `/r0/odom`
- PointCloud2: `/livox/lidar` 또는 `/r0/toy/slice_kept_points`

저장된 `rviz/two_robot_mapping.rviz`는 `/toy_record/*`와 `map` frame을 사용하는 두 로봇용 layout입니다. 단일 로봇 live mode에서는 위처럼 Fixed Frame과 topic을 설정합니다.

## 5. Bag mode (선택 사항)

센서 없이 저장된 rosbag2로 확인할 때만 bag mode를 사용합니다. Bag mode에는 센서 드라이버가 필요하지 않고, mapping 패키지와 ROS 2만 source하면 됩니다.

Bag 디렉터리에는 `metadata.yaml`이 있어야 합니다. 기본 source topic은 다음과 같습니다.

- `/livox/lidar`
- `/livox/imu`
- `/tf_static` (선택 사항)

```bash
ros2 bag info /path/to/mid360_run
```

### Bag 한 개

```bash
bash --noprofile --norc
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source ~/co_3dto2d_mapping/install/local_setup.bash

ros2 launch co_3dto2d_mapping single_bag_mapping.launch.py \
  use_bag:=true \
  bag_path:=/path/to/mid360_run \
  storage_id:=sqlite3 \
  rate:=1.0 \
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

두 bag launch는 `/r0`과 `/r1` pipeline을 실행하고 초기 XY 정렬을 계산합니다. 기본 설정에서는 `/toy_record/*` 토픽을 publish하므로 저장된 RViz layout을 사용할 수 있습니다.

두 bag launch는 두 재생을 wall clock으로 지연시키므로 `use_sim_time:=true`를 추가하지 않습니다. bag 하나를 먼저 단독으로 검증한 다음 두 bag 정렬을 실행합니다.

```bash
rviz2 -d "$(ros2 pkg prefix --share co_3dto2d_mapping)/rviz/two_robot_mapping.rviz"
```

Bag 안의 source topic 이름이 `/livox/lidar` 또는 `/livox/imu`와 다르면 launch remap 인자만으로는 바꿀 수 없습니다. 먼저 bag을 변환하거나 `launch/bag_mid360.launch.py`의 source topic을 수정해야 합니다.

## 주요 Parameter

mapper와 Python node의 선언값, YAML 기본값, launch override, 단위, validation은 [`docs/parameters.md`](docs/parameters.md)에 정리되어 있습니다.

특히 다음 두 범위는 서로 다릅니다.

- `range_min_m` / `range_max_m`: 장애물 hit endpoint에 적용되는 범위
- `raycast_min_range_m` / `raycast_max_range_m`: sensor origin에서 빈 공간을 추적하는 raycast 범위

occupancy 값은 미관측 `-1`, free `0`, occupied `100`입니다. 미관측 영역을 free로 바꾸지 않습니다.

## 주요 토픽과 frame

| 구분 | Live mode | Bag 한 개 |
| --- | --- | --- |
| LiDAR 입력 | `/livox/lidar` | bag의 `/livox/lidar`를 재생 후 `/livox/lidar` 또는 `/livox/lidar_raw` |
| IMU 입력 | `/livox/imu` | bag의 `/livox/imu`를 재생 후 `/livox/imu` |
| Filtered IMU | `/livox/imu_filtered` | `/livox/imu_filtered` |
| Odometry | `/r0/odom` | `/r0/odom` |
| Occupancy | `/r0/toy/local_occupancy`, `/r0/toy/global_occupancy` | 같은 토픽 |
| Frame | `base_link`, `livox_frame`, `odom` | `base_link`, `livox_frame`, `odom` |

## 문제 해결

- **`ros2: command not found`:** 새 `bash --noprofile --norc` 셸에서 `/opt/ros/humble/setup.bash`를 source합니다.
- **`Package 'co_3dto2d_mapping' not found`:** `source ~/co_3dto2d_mapping/install/local_setup.bash`를 실행하고 필요하면 저장소 루트에서 다시 빌드합니다.
- **`canonicalize_version()` 또는 `setuptools` 오류로 build 실패:** 새 셸에서 `export PYTHONNOUSERSITE=1`을 설정한 뒤 다시 `colcon build`합니다. ROS 2 Humble에는 `pip install --user --upgrade setuptools packaging`으로 system package를 덮어쓰지 않는 것이 안전합니다.
- **Odometry가 시작되지 않음:** `/livox/imu`가 실제로 publish되는지 확인합니다. 기본 설정은 IMU가 들어올 때까지 odometry 초기화를 기다립니다.
- **Cloud-to-base TF 경고 또는 map의 회전·위치 오차:** `sensor_tf_x`, `sensor_tf_y`, `sensor_tf_z`, `sensor_tf_yaw`, `sensor_tf_pitch`, `sensor_tf_roll`에 측정한 extrinsic을 지정합니다.
- **ROS 1 library가 섞인 것처럼 보임:** 새 `bash --noprofile --norc` 셸을 열고 `PYTHONNOUSERSITE=1`, ROS 2와 mapping workspace의 `local_setup.bash`만 순서대로 source합니다.
