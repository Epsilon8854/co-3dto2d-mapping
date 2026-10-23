# 정리된 워크스페이스와 실행 경로

- README 개편은 논의 후 진행합니다. 현재 운영 명령은 이 문서를 기준으로 합니다.
- `co_3dto2d_mapping/`: 설치되는 ROS Python 패키지. `common/` 공통 수학·저장, `single_agent/` 중력·높이 필터, `multi_agent/` 정합·지도 병합·pose 중계.
- `include/`, `src/`: 현재 occupancy mapper와 PHT 보정 C++ 구현.
- `launch/`: single/multi 공용 pipeline. `internal/`은 raw RTAB-Map ICP 설정 연결.
- `scripts/`: 공용 실시간 실행기 **두 개**. `--robot-id`는 물리 PC와 무관한 논리 ID입니다. 중앙 기준 agent는 r0입니다.
- `scripts/lib/`: 실행 수명·driver 중복 방지·DDS 환경 내부 구현.
- `scripts/tools/`: 환경 설치, 녹화, raw bag 재생, 진단, 정합 모니터.
- `scripts/experiment/`: 실험 1~4 분석·재생. `support/`의 과거 floor/대안 정합 구현은 운영 빌드에서 제외됩니다.
- `config/experiment/`, `rviz/experiment/`, `docs/experiment/`: 과거 실험 재현 설정·화면·기록. 운영 기본값으로 읽지 않습니다.
- `outputs/`: 취득 bag·실행 snapshot·지도·보고서. Git에 포함하지 않습니다. 기존 실험 결과는 이동/삭제하지 않았습니다.

## 운영 설정

| 파일 | 수정할 내용 |
| --- | --- |
| `config/single_agent.yaml` | 공통 occupancy/PHT, 고정 높이·IMU 자세, 높이 필터 |
| `config/multi_agent.yaml` | `agent_overrides`: single 설정의 multi 전용 차이; `central`: 초기/주기 정합·중앙 free 갱신·pose 중계 |
| `config/sensors/r0.yaml`, `r1.yaml` | 실제 센서 TF, rear sector, 선택 Livox workspace/config 경로. 상대 경로는 해당 YAML 기준 |
| `config/lidar_odometry.yaml` | RTAB-Map ICP. raw cloud·deskew false·queue 1 유지 |
| `config/dds.yaml` | PC 간 DDS 통신 정책. 기존 NIC 제한·SHM·비동기 설정 유지 |
| `config/recording.yaml` | 녹화 topic과 QoS |

- `fixed_base_link_height_above_floor_m`: 보정 odometry의 고정 z. `lidar_mount_height_above_floor_m`: 높이 필터의 바닥 기준 환산값. 기본 각각 0.30m.
- 바닥 plane 추정은 운영 경로에서 제거했습니다. 기존 고정 높이+중력 방식의 계산은 유지합니다.
- 기존 topic/TF 인터페이스는 유지합니다. 이름에 남아 있는 `floor_based_odometry`, `plane_height_filtered`도 호환성을 위해 유지합니다.
- 초기 정합 이후 주기 정합은 기본 OFF입니다. 선택 사용 시 `central.periodic.enabled`와 연관 gate·재투영 예산을 설정합니다.
- live 실행은 각 run에 공통/센서/ICP 설정 원본, robot별 최종 YAML, 소스 hash, DDS 설정, 로그를 저장합니다.

## 설치·실시간 실행

```bash
# ROS 2 Humble/Ubuntu 22.04 및 별도 Livox driver 준비: experiment_04_setup.md
bash scripts/tools/setup_environment.sh

# single-agent: 로컬 driver + mapping + RViz
bash scripts/run_single_agent.sh --robot-id 0

# multi-agent, 중앙 PC: r0 driver + mapping + 병합 + RViz + 정합 모니터
bash scripts/run_multi_agent.sh --role coordinator --robot-id 0

# 다른 PC: r1 driver + mapping
bash scripts/run_multi_agent.sh --role agent --robot-id 1
```

- Livox 경로가 자동 탐지되지 않으면 `--livox-workspace /absolute/workspace`와 `--driver-config /absolute/MID360_config.json`을 지정합니다.
- driver는 기본 자동 시작·종료하며 같은 센서 중복 실행을 거절합니다. 이미 별도 실행 중이면 `--no-start-driver`를 지정합니다.
- 기본 domain은 72입니다. `--domain-id`, `--robot-config`, `--sensor-config`, `--odometry-config`, `--alignment-config`, `--output-root`로 실행 조건을 구분합니다.
- `--dry-run`은 센서/ROS 프로세스를 시작하지 않고 선택한 pipeline·역할·설정 경로를 출력합니다.
- single 결과: `outputs/single_agent/runs/ID/`; multi 결과: `outputs/multi_agent/runs/ID/`. Ctrl+C로 소유한 자식 프로세스만 종료합니다.
- 별도 RViz/rqt/CLI는 같은 DDS 환경을 사용합니다: `bash scripts/tools/with_dds.sh rqt`.

## 녹화·재생

```bash
# 중앙 PC 한 곳에서 두 agent의 filtered cloud/pose/TF/정합 상태 저장
bash scripts/tools/record_bag.sh --name airoom_dynamic
# local/global 지도까지 기록: 중앙 정합의 정확한 입력 재현용
bash scripts/tools/record_bag.sh --name airoom_dynamic --with-agent-maps
# ICP·IMU·높이 필터까지 재실행하려면 raw도 포함 (원격 통신 증가)
bash scripts/tools/record_bag.sh --name airoom_raw --include-raw --with-agent-maps
# single-agent 녹화
bash scripts/tools/record_bag.sh --pipeline single --robot-id 0 --name hallway --include-raw

# raw LiDAR/IMU가 포함된 bag만 전체 pipeline 재생 가능
bash scripts/tools/replay_bag.sh --pipeline multi --bag /absolute/capture/bag --loop
bash scripts/tools/replay_bag.sh --pipeline single --robot-id 0 --bag /absolute/capture/bag --loop
```

- 녹화는 driver/mapping을 시작하거나 종료하지 않습니다. `--output-root outputs/experiment_NAME`으로 실험별 관리할 수 있습니다.
- capture는 `captures/NAME_TIMESTAMP/bag/metadata.yaml`에 저장됩니다. 재생 URI는 `metadata.yaml`의 상위 **bag 디렉터리**입니다.
- 기본 filtered-only bag으로 ICP를 재실행할 수 없습니다. filtered cloud+기록 pose는 mapper 후처리용이며, local grid까지 기록했다면 `scripts/experiment/04_multi_agent_mapping/replay_experiment_04_mapping_inputs.py`로 중앙 병합을 재현할 수 있습니다.
- raw replay는 기록된 센서 TF와 하나의 clock을 사용하고 입력 raw topic만 재생합니다. 반복할 때 전체 노드를 새로 시작해 상태를 초기화합니다. 실제 라이다를 끄거나 별도 ROS domain을 사용하십시오.
- 런타임 파라미터는 녹화 중 한 번 best-effort로 snapshot하며, parameter events는 bag에 기록합니다. 노드 서비스에 접근할 수 없으면 metadata에 실패 사유를 남깁니다.

## 과거 실험 재현

```bash
bash scripts/experiment/support/build_legacy_mapper.sh
bash scripts/experiment/02_occupancy_mapping/replay_experiment_02_occupancy_mapping.sh --help
```

- 과거 launch는 source 경로에서 실행하며 명시적으로 빌드한 `outputs/experiment_support/legacy_mapper/occupancy_mapper`를 사용합니다. 운영 mapper에 폐기된 실험 토글을 다시 넣지 않습니다.
- 기존 bag·결과 파일명은 유지했습니다. 과거 snapshot에 적힌 이전 스크립트 경로는 `scripts/experiment/` 아래 대응 파일을 사용하십시오.
