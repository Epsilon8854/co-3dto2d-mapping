# 설치·실행 안내

## 1. 준비

- **환경:** Ubuntu 22.04 · ROS 2 Humble · Python 3.10
- **ROS 설치:** [공식 안내](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- **두 PC:** 동일 Git revision · 같은 네트워크 · 시계 동기화
- **터미널:** 다른 mapping workspace를 source하지 않은 새 터미널

## 2. 저장소 설치

```bash
git clone --branch features/jeongeon/optimization --single-branch \
  https://github.com/Epsilon8854/co-3dto2d-mapping.git
cd co-3dto2d-mapping
bash scripts/tools/setup_environment.sh
```

- **설치 범위:** ROS 의존성·RTAB-Map·Madgwick·RViz·rosbag2·mapping 빌드
- **업데이트:** `git pull --ff-only` → `bash scripts/tools/setup_environment.sh --skip-install`

## 3. Livox SDK·driver

- **SDK:** [Livox-SDK2 공식 안내](https://github.com/Livox-SDK/Livox-SDK2#readme)에 따라 `/usr/local`에 설치
- **두 PC:** 동일 SDK·driver commit 사용
- **Driver 설치:** 아래 전용 workspace에서 실행

```bash
mkdir -p "$HOME/ws_livox/src"
git clone https://github.com/Livox-SDK/livox_ros_driver2.git \
  "$HOME/ws_livox/src/livox_ros_driver2"
git -C "$HOME/ws_livox/src/livox_ros_driver2" checkout \
  4a1def929e5b59c7a8122d19fce6efba581ce9f7
source /opt/ros/humble/setup.bash
cp "$HOME/ws_livox/src/livox_ros_driver2/package_ROS2.xml" \
  "$HOME/ws_livox/src/livox_ros_driver2/package.xml"
rosdep install --from-paths "$HOME/ws_livox/src" --ignore-src --rosdistro humble -y
cd "$HOME/ws_livox/src/livox_ros_driver2"
bash build.sh humble
```

- **센서 설정:** driver의 `MID360_config.json`을 복사해 해당 PC의 host IP·센서 IP 지정
- **Mapping 설정:** 저장소의 `config/sensors/r0.yaml` 또는 `r1.yaml` 수정
  - `livox.workspace`: 위 driver workspace의 절대 경로
  - `livox.config_file`: 수정한 JSON의 절대 경로
  - `sensor_tf`: 로봇 몸체에 대한 센서 장착 위치·회전
  - `rear_filter`: 몸체에 가려지는 후방 영역
- **Driver 관리:** 실행기가 자동 시작·종료 · 별도 driver 사용 시 `--no-start-driver`

## 4. 네트워크

- **ROS domain:** 기본 72 · 두 PC 동일 설정
- **DDS:** PC 간 통신 NIC 자동 선택 · [수동 지정](experiment_04_dds_network.md)
- **센서망:** 각 PC의 Ethernet IP·라우팅을 해당 LiDAR 설정에 맞게 구성
- **방화벽:** PC 간 DDS multicast·UDP 허용
- **시간:** NTP/chrony 등으로 PC 시계 동기화 · 센서 header stamp 기준 확인

## 5. 실행

- **실행 위치:** 저장소 루트

| 대상 | 명령 |
|---|---|
| Single | `bash scripts/run_single_agent.sh --robot-id 0` |
| Multi r0 | `bash scripts/run_multi_agent.sh --robot-id 0` |
| Multi r1 | `bash scripts/run_multi_agent.sh --robot-id 1` |

- **초기 정합:** 공통 벽·코너 관측 · 가급적 같은 전방 방향 · `LOCKED`까지 정지
- **시각화:** r0에서 RViz·정합 모니터 자동 실행
- **RViz Fixed Frame:** Single·Multi 모두 `map`
- **Single 지도 저장:** 정상 종료 시 한 번 저장 · 실행 중 주기 저장 없음
- **종료:** Ctrl+C · 소유 driver 종료 및 지도 저장
- **결과:** `outputs/single_agent/runs/` 또는 `outputs/multi_agent/runs/`
- **녹화·재생:** [최종 결과 저장·재생](../README.md#5-최종-결과-녹화재생)
- **토픽·파라미터:** [README](../README.md)
