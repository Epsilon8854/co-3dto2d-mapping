# co_3dto2d_mapping

Livox MID-360의 실시간 point cloud와 IMU를 받아 odometry와 2D occupancy grid를 만드는 ROS 2 패키지입니다. C++ mapper, live mapping launch, rosbag2 재생 launch, 두 로봇 정렬과 기록 재게시, 단위 테스트를 포함합니다.

이 README는 Ubuntu 22.04와 ROS 2 Humble이 설치된 새 노트북에서 **실제 MID-360을 먼저 연결하는 흐름**을 기준으로 합니다. rosbag2는 센서가 없을 때 사용하는 선택 사항입니다. 명령은 각 코드 블록 순서대로 실행하고, `~/livox_ws`와 `~/co_3dto2d_mapping`은 서로 다른 workspace로 유지합니다.

## 실행 구조

| 구성 요소 | 역할 |
| --- | --- |
| `livox_ros_driver2` | MID-360에서 `/livox/lidar`와 `/livox/imu`를 publish합니다. 별도 ROS 2 workspace에 설치합니다. |
| `co_3dto2d_mapping` | 센서 토픽을 받아 IMU filtering, RTAB-Map ICP odometry, occupancy mapping을 실행합니다. |
| `live_mapping.launch.py` | bag 없이 위 mapping pipeline을 실행합니다. 기본 `use_sim_time`은 `false`입니다. |
| `single_bag_mapping.launch.py` | rosbag2를 재생하며 mapping pipeline을 실행합니다. bag mode는 선택 사항입니다. |

이 저장소에는 Livox 드라이버와 rosbag 파일이 들어 있지 않습니다. live mode에는 Livox 드라이버를 한 번 설치해야 하며, mapping 패키지는 이 저장소에서 별도로 빌드합니다.

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

## 2. Livox MID-360 드라이버 설치

Livox ROS Driver 2는 이 저장소와 별도로 설치합니다. 현재 driver는 Livox-SDK2를 자동으로 포함하지 않으므로, **SDK2를 먼저 system-wide로 설치**해야 합니다. SDK library는 기본적으로 `/usr/local/lib`에 설치됩니다.

### 2.1 Livox-SDK2 설치

```bash
mkdir -p ~/livox_ws/src
git clone https://github.com/Livox-SDK/Livox-SDK2.git ~/livox_ws/src/Livox-SDK2

cmake -S ~/livox_ws/src/Livox-SDK2 -B ~/livox_ws/src/Livox-SDK2/build
cmake --build ~/livox_ws/src/Livox-SDK2/build --parallel
sudo cmake --install ~/livox_ws/src/Livox-SDK2/build
sudo ldconfig
```

`sudo ldconfig` 뒤에 다음 명령이 `liblivox_lidar_sdk_shared.so`를 출력해야 합니다.

```bash
ldconfig -p | grep liblivox_lidar_sdk_shared
```

### 2.2 Driver clone, 네트워크 설정, 빌드

```bash
git clone https://github.com/Livox-SDK/livox_ros_driver2.git \
  ~/livox_ws/src/livox_ros_driver2

# 아래 JSON을 먼저 실제 IP로 수정합니다.
# host의 cmd/push/point/imu_data_ip는 모두 노트북의 LiDAR 연결 NIC IP여야 합니다.
# lidar_configs[0].ip는 MID-360 IP여야 합니다.
nano ~/livox_ws/src/livox_ros_driver2/config/MID360_config.json

# NIC와 IP가 맞는지 확인합니다. NIC_NAME을 LiDAR에 연결한 interface 이름으로 바꿉니다.
ip -br addr
ip addr show NIC_NAME

cd ~/livox_ws/src/livox_ros_driver2
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
./build.sh humble
source ~/livox_ws/install/setup.bash
ros2 pkg prefix livox_ros_driver2
```

### MID-360 네트워크와 출력 형식

MID-360과 노트북을 Ethernet으로 연결하고, driver의 `config/MID360_config.json`에서 노트북의 host IP와 LiDAR IP를 실제 네트워크에 맞춰 설정합니다. 샘플의 `192.168.1.5` (host)와 `192.168.1.12` (LiDAR)를 그대로 사용하지 말고, 노트북 NIC와 LiDAR가 같은 IPv4 subnet에 있도록 설정합니다. `host_net_info`의 `cmd_data_ip`, `push_msg_ip`, `point_data_ip`, `imu_data_ip`는 모두 host NIC IP를 사용합니다. `lidar_configs[0].ip`는 장비 IP입니다.

driver launch는 설치된 package 아래의 JSON을 사용하므로 source JSON을 바꾼 뒤에는 반드시 `./build.sh humble`을 다시 실행합니다. `MID360_config.json`의 `extrinsic_parameter`와 이 패키지의 `sensor_tf_*`를 동시에 보정하지 마십시오. 이 README의 mapping pipeline은 `sensor_tf_*`를 TF의 단일 기준으로 사용합니다.

mapping pipeline은 `sensor_msgs/msg/PointCloud2`를 사용하므로 driver launch의 `xfer_format`은 `0`이어야 합니다. `rviz_MID360_launch.py`의 현재 기본값은 `0`이며, `msg_MID360_launch.py`는 customized point cloud 형식(`1`)이므로 이 pipeline과 함께 사용하지 않습니다. `multi_topic=0`일 때 이 README는 `/livox/lidar`, `/livox/imu`를 기대합니다.

## 3. mapping 패키지 clone과 빌드

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
source ~/livox_ws/install/setup.bash
source ~/co_3dto2d_mapping/install/local_setup.bash
```

## 4. Live mode 실행

### 터미널 A: Livox driver

```bash
bash --noprofile --norc
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash

ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

이 launch는 PointCloud2 출력과 Livox용 RViz 창을 함께 시작합니다. mapping을 시작하기 전에 아래 검증을 통과해야 합니다. `type`의 기대값이 다르면 mapping을 실행하지 말고 driver의 `xfer_format`, `multi_topic`, 그리고 JSON IP를 먼저 수정합니다.

```bash
ros2 topic list | grep -E '/livox/(lidar|imu)'
ros2 topic type /livox/lidar
ros2 topic type /livox/imu
ros2 topic hz /livox/lidar
ros2 topic echo /livox/imu --once
```

기대 type은 각각 `sensor_msgs/msg/PointCloud2`, `sensor_msgs/msg/Imu`입니다. `multi_topic=1` 또는 별도 driver 설정으로 실제 topic 이름이 다르면, mapping launch에 실제 이름을 넘깁니다. 예를 들면:

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
source ~/livox_ws/install/setup.bash
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

## 5. Live mode 확인

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

## 6. Bag mode (선택 사항)

센서 없이 저장된 rosbag2로 확인할 때만 bag mode를 사용합니다. Bag mode에는 Livox driver workspace가 필요하지 않고, mapping 패키지와 ROS 2만 source하면 됩니다.

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
- **`livox_ros_driver2`를 찾지 못함:** `~/livox_ws/install/setup.bash`를 ROS 2 setup 뒤에 source하고 `ros2 pkg prefix livox_ros_driver2`로 확인합니다.
- **`liblivox_lidar_sdk_shared.so`를 열 수 없음:** SDK2 설치 뒤 `sudo ldconfig`을 실행합니다. 그래도 해결되지 않으면 현재 셸에서 `export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}`를 설정하고 driver를 다시 실행합니다.
- **Livox topic이 나오지 않음:** MID-360과 노트북의 Ethernet 연결, host IP, LiDAR IP, `MID360_config.json`을 확인합니다.
- **PointCloud2가 나오지 않음:** `msg_MID360_launch.py`가 아니라 `rviz_MID360_launch.py`를 사용하고 `xfer_format=0`인지 확인합니다. `ros2 topic type /livox/lidar`가 `sensor_msgs/msg/PointCloud2`인지도 확인합니다.
- **`Package 'co_3dto2d_mapping' not found`:** `source ~/co_3dto2d_mapping/install/local_setup.bash`를 실행하고 필요하면 저장소 루트에서 다시 빌드합니다.
- **`canonicalize_version()` 또는 `setuptools` 오류로 build 실패:** 새 셸에서 `export PYTHONNOUSERSITE=1`을 설정한 뒤 다시 `colcon build`합니다. ROS 2 Humble에는 `pip install --user --upgrade setuptools packaging`으로 system package를 덮어쓰지 않는 것이 안전합니다.
- **Odometry가 시작되지 않음:** `/livox/imu`가 실제로 publish되는지 확인합니다. 기본 설정은 IMU가 들어올 때까지 odometry 초기화를 기다립니다.
- **Cloud-to-base TF 경고 또는 map의 회전·위치 오차:** `sensor_tf_x`, `sensor_tf_y`, `sensor_tf_z`, `sensor_tf_yaw`, `sensor_tf_pitch`, `sensor_tf_roll`에 측정한 extrinsic을 지정합니다.
- **ROS 1 library가 섞인 것처럼 보임:** 새 `bash --noprofile --norc` 셸을 열고 `PYTHONNOUSERSITE=1`, ROS 2, Livox driver의 `setup.bash`, mapping workspace의 `local_setup.bash`만 순서대로 source합니다.
