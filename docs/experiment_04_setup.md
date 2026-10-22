# 실험 4: 두 노트북 환경 설정·실행

## 1. 공유 범위와 준비

- 브랜치: `features/jeongeon/optimization`. 기존 `features/jeongoen/optimization` 원격 브랜치는 보존합니다.
- Ubuntu 22.04, ROS 2 Humble, Python 3.10을 기준으로 합니다. [ROS 공식 설치 안내](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
- source/config/launch/RViz/실행·녹화·재생 스크립트를 공유합니다. `outputs/`, rosbag, 결과 PNG/HTML, build/install/log, 외부 SDK/driver 빌드는 포함하지 않습니다.
- 설치 스크립트가 `package.xml`의 의존성을 rosdep으로 설치합니다. C++/Python OpenCV, NumPy/SciPy/YAML, RTAB-Map odometry, Madgwick, RViz, rosbag2 Python 및 SQLite storage를 포함합니다.
- apt 패키지는 설치 시점의 Humble 저장소 버전을 사용합니다. `config/environment/experiment_04_reference.json`은 현재 PC의 비교 기준입니다. 컨테이너처럼 모든 바이너리 버전을 강제로 고정하는 구성은 아닙니다.

```bash
git clone --branch features/jeongeon/optimization --single-branch \
  https://github.com/Epsilon8854/co-3dto2d-mapping.git
cd co-3dto2d-mapping
bash scripts/setup_experiment_04_environment.sh
```

- 다른 mapping/Livox workspace를 source하지 않은 새 터미널에서 설치합니다.
- 이미 의존성이 설치되어 있으면 `bash scripts/setup_experiment_04_environment.sh --skip-install`로 빌드만 수행할 수 있습니다.
- 양쪽 노트북은 `git rev-parse HEAD`가 같아야 합니다. `config/experiments/experiment_04_multi_agent_mapping/robot0.yaml`, `robot1.yaml`, `alignment.yaml`이 실험 설정입니다.

## 2. Livox SDK와 driver

- [Livox-SDK2 공식 설치 안내](https://github.com/Livox-SDK/Livox-SDK2#readme)에 따라 SDK를 `/usr/local`에 설치합니다.
- 현재 PC에 설치된 SDK의 원본 Git revision은 확인할 수 없어 고정 revision으로 표기하지 않습니다. 비교용 SDK library/header SHA-256은 reference JSON에 기록했습니다. 양쪽 노트북에 새로 설치할 때는 같은 SDK commit을 사용하고 해당 SHA를 별도 기록합니다.
- Driver는 현재 PC에서 사용한 commit을 지정합니다. SDK와 driver를 이미 설치했다면 재설치하지 않고 `--livox-workspace`로 기존 경로를 지정할 수 있습니다.

```bash
mkdir -p "$HOME/ws_livox/src"
git clone https://github.com/Livox-SDK/livox_ros_driver2.git \
  "$HOME/ws_livox/src/livox_ros_driver2"
git -C "$HOME/ws_livox/src/livox_ros_driver2" checkout \
  4a1def929e5b59c7a8122d19fce6efba581ce9f7
source /opt/ros/humble/setup.bash
# Driver upstream의 ROS 2 package manifest를 선택한 후 의존성 설치.
cp "$HOME/ws_livox/src/livox_ros_driver2/package_ROS2.xml" \
  "$HOME/ws_livox/src/livox_ros_driver2/package.xml"
rosdep install --from-paths "$HOME/ws_livox/src" --ignore-src --rosdistro humble -y
cd "$HOME/ws_livox/src/livox_ros_driver2"
bash build.sh humble
```

- `build.sh`는 해당 Livox workspace의 기존 build/install을 다시 만듭니다. 위 전용 workspace에서 사용합니다.
- Driver `config/MID360_config.json`을 각 PC에서 복사해 센서 IP와 host IP를 설정합니다. 현재 PC의 실센서 IP 설정은 Git에 올리지 않습니다.
- 실행 시 `--driver-config /absolute/path/MID360_config.json`을 지정합니다. 실행기가 LiDAR/IMU를 `/rN/livox/*`, frame을 `rN/livox_frame`으로 구분하고 PointCloud2를 사용합니다.
- 같은 물리 센서의 driver를 중복 실행하지 않습니다. 이미 실행 중이면 `--start-driver`를 빼십시오.

## 3. 두 PC 네트워크·시간

- 두 PC가 DDS 통신 가능한 같은 네트워크에 있어야 합니다. 동일한 `ROS_DOMAIN_ID=72`, `ROS_LOCALHOST_ONLY=0`을 사용합니다.
- 센서 Ethernet IP/라우팅, 방화벽·multicast 허용은 각 PC 네트워크에 맞게 설정합니다. 스크립트는 OS 네트워크 설정을 변경하지 않습니다.
- 취득 전 NTP/chrony 등으로 PC clock을 맞추고, 실제 LiDAR/IMU header stamp가 같은 시간 기준인지 확인합니다. clock 오프셋이 있으면 원 bag의 기록 시각을 임의 이동해 해결하지 않습니다.
- 수동 확인 터미널에서는 다음을 실행합니다. 실행 wrapper는 ROS와 mapping workspace를 자동 source합니다.

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=72
export ROS_LOCALHOST_ONLY=0
```

## 4. 실시간 실행

```bash
# r0 PC: r0 driver + single pipeline + fusion + RViz
cd ~/co-3dto2d-mapping
bash scripts/run_experiment_04_multi_agent_mapping.sh \
  --role all --robot-id 0 --start-driver --domain-id 72 \
  --livox-workspace "$HOME/ws_livox" \
  --driver-config /absolute/path/to/r0_MID360_config.json

# r1 PC: r1 driver + single pipeline
cd ~/co-3dto2d-mapping
bash scripts/run_experiment_04_multi_agent_mapping.sh \
  --role robot --robot-id 1 --start-driver --domain-id 72 \
  --livox-workspace "$HOME/ws_livox" \
  --driver-config /absolute/path/to/r1_MID360_config.json
```

- 초기에는 두 센서를 고정하고 공통 벽·코너를 관측합니다. 같은 방향일 필요는 없습니다.
- 3초 관측 창 3회와 후보 검증으로 정합을 확정합니다. 실제 대기시간은 map 생성·검색 시간에 따라 늘어납니다. 대칭 공간·overlap 부족은 거절될 수 있습니다.
- RViz fixed frame은 `r0/odom`입니다. `alignment_ready=true`와 벽·코너 정합을 확인한 뒤 주행을 시작합니다.
- r0/r1 실행 터미널에 session ID와 `WAIT_INPUT`(입력 대기), `COLLECTING`(정지 관측 수집), `SEARCHING`(정합 검색), `VERIFYING`(반복 검증), `LOCKED`(완료), `STALE`(입력 지연)를 표시합니다. 상태 변경 시 즉시, 유지 중에는 5초마다 출력합니다. 중앙 상태가 5초 이상 끊기면 별도 수신 공백 경고를 출력합니다.
- 자세한 driver/ICP 로그는 기존 로그 파일에 보관합니다. 터미널은 진단 topic과 session만 구독하며 raw cloud/IMU/map을 추가 구독하지 않습니다.
- ICP는 raw cloud, deskew off, queue 1을 유지합니다. rear/height filter는 mapping 입력에 적용합니다. 고정 높이·IMU gravity 및 PHT full_se2/history 5가 각 single pipeline에 적용됩니다.
- 센서 장착 높이·base 높이·extrinsic은 robot YAML의 `fixed_base_link_height_above_floor_m`, `lidar_mount_height_above_floor_m`, `experiment_sensor_tf_*`에서 변경합니다. 초기값은 높이 각각 0.30 m, extrinsic 0입니다.

```bash
ros2 topic echo /multi_agent/alignment_ready --qos-durability transient_local
ros2 topic echo /multi_agent/alignment_status --qos-durability transient_local
ros2 run tf2_ros tf2_echo r0/odom r1/odom
ros2 service call /multi_agent/save_map std_srvs/srv/Trigger '{}'
# 두 센서를 멈춘 후 초기 정합 재시도
ros2 service call /multi_agent/reset_alignment std_srvs/srv/Trigger '{}'
```

- 기존 mapping을 중단하지 않고 별도 터미널에서 상태만 확인할 수도 있습니다. 이 명령의 Ctrl+C는 표시기만 종료합니다. r1 PC에서는 `--robot-id 1`을 사용합니다.

```bash
bash scripts/run_experiment_04_multi_agent_mapping.sh \
  --role robot --robot-id 0 --domain-id 72 --status-only
```

- 정합 확정 후 중앙의 local map 구독을 해제합니다. session 변경·clock reset·정합 reset 시 자동 복구합니다. 각 single mapper의 local map 발행·로컬 녹화는 유지됩니다. 다른 원격 RViz/녹화기가 local map을 구독하면 해당 전송은 계속됩니다.
- Global map과 pose/session은 계속 수신합니다. 지도는 현재 전체 grid를 주기적으로 전송하며 증분 전송·별도 저주기 bridge는 아직 적용하지 않았습니다. raw odometry는 비교 경로용입니다.
- `alignment_status`의 `input_receipt_ages_sec`는 중앙 콜백의 수신 공백(PC clock 차이 제외), `central_tick_runtime_ms`는 지도 합성·경로 발행·checkpoint를 포함한 중앙 주기 처리시간입니다. 이 지표만으로 실제 무선 지연이나 원격 센서 처리 지연을 단정하지 않습니다.

| Topic | 내용 |
| --- | --- |
| `/multi_agent/r1_to_r0_transform` | 확정 후 고정된 r1 → r0 변환 |
| `/multi_agent/alignment_ready`, `/multi_agent/alignment_status` | 정합·session·입력 지연 상태와 후보 평가 |
| `/multi_agent/r1/aligned_global_occupancy` | r0 frame으로 변환한 r1 map |
| `/multi_agent/merged_global_occupancy` | `r0/odom` 기준 합성 map |
| `/multi_agent/rN/aligned_odometry`, `/multi_agent/rN/aligned_path` | r0 frame의 최종 pose·경로 |
| `/multi_agent/rN/raw_path`, `/multi_agent/rN/session` | raw 비교 경로·session 정보 |

## 5. 녹화·재생·저장

```bash
# 각 PC의 별도 터미널: robot-id와 run-id를 해당 PC에 맞게 지정
bash scripts/record_experiment_04_multi_agent_mapping.sh \
  --scope robot --robot-id 0 --run-id airoom_r0 --domain-id 72
bash scripts/record_experiment_04_multi_agent_mapping.sh \
  --scope robot --robot-id 1 --run-id airoom_r1 --domain-id 72
# 중앙 PC의 별도 터미널: map/pose/정합 결과
bash scripts/record_experiment_04_multi_agent_mapping.sh \
  --scope central --run-id airoom_central --domain-id 72
```

- 두 실제 robot bag을 한 PC로 복사합니다. 서로 다른 센서의 raw topic/frame과 겹치는 실제 취득 시간, sensor static TF가 필요합니다.
- bag 병합은 원 기록 시각·header stamp를 유지합니다. 하나의 bag을 복제해 두 센서 실험으로 사용하지 않습니다.

```bash
python3 scripts/prepare_experiment_04_combined_bag.py \
  --bag0 /absolute/path/to/r0/bag --bag1 /absolute/path/to/r1/bag \
  --output outputs/experiment_04_multi_agent_mapping/captures/airoom_combined
bash scripts/replay_experiment_04_multi_agent_mapping.sh \
  --bag outputs/experiment_04_multi_agent_mapping/captures/airoom_combined \
  --domain-id 73 --loop
```

- replay는 하나의 player가 raw LiDAR/IMU와 static TF, `/clock`을 공급합니다. 매 loop에서 두 single pipeline과 fusion을 재시작합니다.
- 실행 snapshot/log/map은 `outputs/experiment_04_multi_agent_mapping/runs/RUN_ID/`에 생성됩니다. 녹화는 `captures/RUN_ID/`에 저장합니다. 이 디렉터리들은 Git에서 제외됩니다.
- 정상 Ctrl+C·저장 서비스·10초 checkpoint로 지도 저장. 최종 `cycle_NNN/maps/`에 PGM/YAML, 정확한 occupancy NPZ, PNG와 raw/보정 경로 overlay가 저장됩니다.
- 두 PC 종료 시 robot-only 측을 먼저, fusion 측을 마지막에 종료합니다. 강제 전원 차단은 종료 저장을 보장하지 않습니다.

## 6. 검증 범위

- 단위 테스트와 합성 ROS 입력에서 yaw/XY 정합·map 저장·session reset을 확인했습니다.
- 실제 두 MID-360의 방향 차이, clock/네트워크 조건, 초기 정합 정확도·실시간 처리율은 다음 실증에서 확인해야 합니다.
- 센서 없이 재검사하려면 별도 domain에서 `scripts/validate_experiment_04_startup.py`를 실행할 수 있습니다. 생성 결과는 `outputs/`에만 저장합니다.
