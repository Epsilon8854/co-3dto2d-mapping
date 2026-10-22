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
- 각 agent의 global map이 최소 3초 범위로 갱신된 뒤, 기본 `confirmation_windows: 1`로 정합합니다. 전체 yaw 검색과 모든 품질 기준을 통과한 첫 후보를 바로 확정합니다. 거절된 검색은 다시 시도하므로 검색을 딱 한 번만 수행한다는 의미는 아닙니다. 시간에 따른 변환 일관성 재검사는 생략합니다. 실제 대기시간은 map 생성·검색 시간에 따라 늘어납니다. 대칭 공간·overlap 부족은 거절될 수 있습니다.
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

- 초기에는 중앙이 각 agent의 **누적 global map**을 구독합니다. map별 pose timestamp를 0.4초 이내로 짝짓는 조건과 local map 15개 조건을 제거했습니다. 고정된 초기 anchor를 사용해 지도 좌표를 변환합니다. pose/session은 정지 여부·재시작 감지를 위해 계속 확인합니다.
- 최초 360° 검색 → 중간 검증에서 이전 후보군 우선 탐색 → 최종 360° 재검사 순서입니다. 후보군이 불충분하거나 품질 검사를 통과하지 못하면 전체 검색으로 돌아갑니다. 지도 경계 점 수가 크게 변해도 전체 검색합니다. default free-space conflict는 18%, 후보 간 점수 차이·기하·XY/yaw 일관성 검사는 유지합니다.
- 변환 확정 후 초기 global snapshot으로 두 지도 layer를 채우고 **global 구독 OFF / local 구독 ON**으로 전환합니다. 이후 `rN/odom` 기준 local grid에 고정 agent 변환만 적용합니다. pose를 다시 곱하지 않습니다.
- Unknown은 기존 증거를 지우지 않고 occupied가 free보다 우선합니다. 이번 통신 변경에는 동적 clear를 추가하지 않습니다. local map이 누락되면 해당 관측이 없는 영역을 복구하지 못할 수 있습니다. 내부 local queue 초과 시에는 해당 agent의 global snapshot을 한 번 재수신하여 보완하고 다시 local-only로 돌아갑니다. DDS 이전 단계에서의 모든 누락을 자동 검출하는 기능은 아닙니다.
- 각 single mapper의 발행·로컬 녹화는 유지됩니다. 다른 원격 RViz/녹화기가 global map을 구독하면 해당 전송은 계속됩니다. 전체 지도 송신 주기를 낮추는 producer bridge는 아직 추가하지 않았습니다.
- **별도 `multi_agent_pose_relay.py` 프로세스**가 pose·TF를 전담합니다. source timestamp를 보존하고 입력 Best Effort / depth 1, 출력 Reliable / depth 1로 최신 pose를 중계합니다. 출력은 기존 Reliable RViz/녹화 구독자와 호환됩니다. RViz의 aligned odometry도 Best Effort / depth 1로 설정했습니다. 포즈 발행 주파수를 인위적으로 늘리거나 과거 pose의 시각을 현재로 바꾸지 않습니다.
- 지도 통합, 경로 생성, checkpoint/PNG/HTML 저장은 pose relay에서 수행하지 않습니다. 지도 노드는 각각의 worker에서 처리하며 시각화 path는 기본 1초마다, 최대 5000 pose를 표시합니다. raw ICP TF와 mapping_base_link TF의 소유자를 분리했습니다.
- `/multi_agent/pose_relay_status`에는 수신·발행 수, callback p95/max, 입력 수신 공백을 표시합니다. `alignment_status`에는 단계별 구독·local queue·합성 연산시간을 표시합니다. callback 시간과 실제 무선 지연은 서로 다른 지표입니다.
- session 변경·clock reset·명시적 reset 시 초기 global 구독으로 복구하고 누적 지도/경로를 초기화합니다. pose relay도 session/epoch를 확인해 이전 변환을 재사용하지 않습니다.

### Free-space 충돌 기준 비교

- `registration.max_free_conflict_ratio`는 정합 후 비교 가능한 샘플에서 occupied ↔ free가 모순되는 비율의 상한입니다. 지도 전체 cell의 불일치율이나 정합 실패 확률이 아닙니다. unknown 영역은 분모에서 제외됩니다.
- 기본 `alignment.yaml`은 **0.18**입니다. YAML을 생략한 Experiment 4 중앙 노드·startup helper의 기본도 0.18입니다. 공통 single/기존 정합 모듈의 기본값은 변경하지 않습니다. 기존 실행 snapshot과 실행 중인 설정은 바뀌지 않습니다.
- 이전 0.14 조건은 `alignment_free_conflict_014.yaml`에 보존합니다. `alignment_free_conflict_018.yaml`은 기존 명령 호환을 위해 유지하며 현재 기본과 같은 설정입니다.
- 0.18은 최근 후보 15.2%·16.5%의 해당 gate를 통과시킬 수 있지만, overlap/RMSE/후보 gap/기하 조건을 모두 통과해야 확정됩니다. 기본은 1회 확인이며 올바른 정합임을 보장하지 않습니다. `confirmation_windows`를 3으로 지정하면 이전처럼 누적 snapshot 간 XY·yaw 일관성을 재검사합니다.
- 단계별 통신 분리를 적용한 동일 코드에서 0.14와 0.18을 비교합니다. 실측 XY/yaw 또는 벽·코너 overlay로 오정합 증가 여부를 함께 확인합니다.

```bash
# 현재 기본 18%: 추가 옵션 없이 기존 실행 명령 사용.
# 이전 14%와 비교할 때만 아래 config를 지정합니다.
bash scripts/run_experiment_04_multi_agent_mapping.sh \
  --role all --robot-id 0 --start-driver --domain-id 72 \
  --alignment-config config/experiments/experiment_04_multi_agent_mapping/alignment_free_conflict_014.yaml
```

### 참고 브랜치와 초기 정합·시각화 비교

- 읽기 전용 비교 대상: `fix/direct-occupancy-icp-no-gates`의 `36ad9ba`. 참고 브랜치를 checkout/수정/push하지 않았습니다.
- 실제 실행 파일은 CMake 설치 규칙까지 따라 확인했습니다. public `two_live_mapping.launch.py`는 compatibility wrapper, public `record_republisher.py`는 `record_republisher_world.py`입니다.

| 항목 | 참고 브랜치의 기본 direct 모드 | 실험 4 |
| --- | --- | --- |
| 입력 | 최신 global occupancy pair | 정지 상태에서 누적한 global snapshot |
| 정합 연산 | raster search + trimmed ICP | 같은 raster/ICP 기반, 전체 yaw 검색·다중 후보 검증 |
| yaw coarse 검색 | 초기 가설 주변 ±12°, 2° 간격(13개) | −180°~180°, 5° 간격(72개) |
| 확정 조건 | 유한한 결과이면 발행; overlap/RMSE/free conflict/consensus gate 생략 | 품질·대안 후보 차이·기하 조건 통과 시 확정; 기본 1회, 반복 검증 선택 가능 |
| 정합 반복 | 새 pair마다 direct 검색(처리 timer 1초); world republisher는 기본적으로 첫 변환을 고정 | 초기 확정 후 검색 중단; session/reset 시 재시도 |
| pose 시각화 | 별도 record republisher에서 최신 pose를 200ms마다 재발행, source stamp를 현재 시각으로 변경; TF Axes 사용 | 수신 pose callback에서 변환·발행, 원 stamp 유지; aligned odometry Axes 사용 |
| 주행 중 부하 | 정합 노드와 record republisher가 별도 프로세스; republisher 자체의 지도 부하는 여전히 존재 | pose/TF 전용 프로세스와 지도/검색/경로/저장 worker 분리 |

- 참고 브랜치가 빠르게 결과를 내도록 구성된 이유는 좁은 탐색 범위와 판정 생략입니다. 완전히 다른 고속 정합 기법을 사용한다고 볼 수 없습니다. 같은 입력·장비에서 계산시간을 재측정한 비교는 아닙니다.
- 참고 브랜치가 갱신한 timestamp는 최신 센서 관측을 뜻하지 않습니다. 기존 방식의 동작과 현재 방식의 지연을 비교할 때 pose 값 변화, 원 관측 시각, 수신 시각을 함께 확인해야 합니다.
- 초기 정합 속도와 주행 중 axes 지연은 별개입니다. 실험 4는 확정 후 검색하지 않으므로 초기 검색을 빠르게 바꾸는 것만으로 주행 중 pose 지연이 해결되지는 않습니다.
- 단계별 global→local 구독, 별도 pose/TF 중계, 지도/경로/저장 worker 분리, 중간 검증 후보 재사용은 구현했습니다. 입력 local grid의 원격 전송량 자체가 큰 환경에서는 producer 측 전송 주기·압축을 추가로 검토해야 합니다.
- 입력이 약 5~6Hz라면 중계 발행률만 높여도 새 motion 관측이 늘어나지 않습니다. 우선 불필요한 대기·누적 backlog를 줄이는 것이 목적입니다.

| Topic | 내용 |
| --- | --- |
| `/multi_agent/r1_to_r0_transform` | 확정 후 고정된 r1 → r0 변환 |
| `/multi_agent/alignment_ready`, `/multi_agent/alignment_status` | 정합·session·입력 지연 상태와 후보 평가 |
| `/multi_agent/pose_relay_status` | 별도 pose 중계 처리시간·수신 공백 |
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

- 2026-10-22 후속 변경: 기본 검증 횟수를 3→1로 줄였습니다. 이 설정으로 합성 ROS 입력에서 1회 full-circle 검색 후 확정, local-only 확장, 저장, session 초기화와 Reliable 구독 호환을 확인했습니다. 아래 실제 두 PC의 31초 결과는 변경 전 3회 조건입니다. 이미 실행 중인 노드와 저장된 config snapshot은 자동 변경되지 않습니다.
- 단위 테스트와 합성 ROS 입력에서 yaw/XY 정합·local-only 지도 확장·map 저장·session reset을 확인했습니다.
- production config 합성 검증의 검색 시간은 최초 2093 ms / 중간 후보 재사용 33 ms / 최종 2118 ms였습니다. source publish→aligned pose 수신 지연 중앙값 2.87 ms, p95 6.74 ms였습니다. 실제 두 센서의 정합 오차나 무선 지연을 의미하지 않습니다.
- 2026-10-22 실제 두 MID-360 확인: r1 재시작으로 이전 누적 지도를 초기화한 뒤 약 31초 후 3회 검증을 통과했습니다. 고정 변환은 x=0.156 m, y=-0.329 m, yaw=2.03°였으며, global 구독 OFF / local 구독 ON과 병합 지도 checkpoint 저장을 확인했습니다. 최종 후보의 free conflict는 17.43%로 기본 18% 이내였습니다. 실측 정답에 대한 오차는 아닙니다.
- 같은 25초 관측에서 r0 aligned pose는 약 6.33 Hz, r1 aligned pose는 약 1.02 Hz였습니다. 동일 stamp를 두 topic에서 받은 관측자의 수신시각 차이 p95는 r0 2.28 ms / r1 1.16 ms였습니다. 원격 raw odometry는 약 6.37 Hz로 보였지만 corrected pose는 드물게 수신됐습니다. 각 구독자의 Best Effort 수신 손실과 스케줄링이 달라 관측 수가 같지는 않습니다. r1 producer 부하와 무선 전달 손실은 r1 PC의 로컬 계측을 통해 구분해야 합니다. 중계 분리만으로 axes 갱신 지연이 완전히 해결됐다고 판단하지 않습니다.
- 실제 두 센서의 다양한 방향 차이, clock/네트워크 조건, 초기 정합 정확도·주행 중 처리율은 추가 실증에서 확인해야 합니다.
- 센서 없이 재검사하려면 별도 domain에서 `scripts/validate_experiment_04_startup.py`를 실행할 수 있습니다. 생성 결과는 `outputs/`에만 저장합니다.

## 7. r1 내부 처리율과 r0 수신율 구분

- `monitor_experiment_04_pipeline.py`는 raw odometry → fixed/gravity 보정 odometry → mapper/PHT 보정 odometry의 수신율·공백·header 시각을 기록합니다. raw point cloud는 구독하지 않습니다.
- `local` 역할은 반드시 해당 센서가 연결된 PC에서 실행합니다. local/global grid는 Python 역직렬화 없이 byte 수·수신율만 관측하고, 작은 `/multi_agent/rN/pipeline_metrics` 진단 메시지를 1 Hz로 발행합니다. 지도 구독 자체의 계측 부하는 존재합니다.
- `receiver` 역할은 r0에서 실행하며 원격 지도 구독을 추가하지 않습니다. r1 pose·aligned pose와 작은 진단 메시지를 JSON으로 저장합니다. 계측 프로세스는 정해진 시간 후 종료됩니다.
- 두 PC의 겹치는 관측 구간을 비교합니다. r1 로컬도 느리면 내부 파이프라인을, 로컬은 빠르고 r0 수신만 느리면 전달/QoS/수신 부하를 먼저 점검합니다. 모두 구독자 관측치이므로 실제 publisher 카운터나 순수 무선 지연과 동일하지 않습니다. header-age 비교에는 PC clock sync가 필요합니다.
- bash 실행 파일이 ROS 환경을 자동 설정합니다. 기본은 r1 로컬 계측 40초, domain 72이며 빌드나 mapper 재시작 없이 실행됩니다. 결과는 `outputs/experiment_04_multi_agent_mapping/diagnostics/실행시각/` 아래에 저장해 실행별로 구분합니다. `--output`으로 경로를 지정할 수도 있습니다.

```bash
# r0 PC: 먼저 실행
bash scripts/monitor_experiment_04_pipeline.sh --role receiver

# r1 PC: 최신 코드 git pull 후 별도 터미널에서 실행. mapper 재시작은 불필요.
bash scripts/monitor_experiment_04_pipeline.sh
```
