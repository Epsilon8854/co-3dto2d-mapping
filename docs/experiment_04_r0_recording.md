# 실험 4 — r0 한 곳에서 두 agent 녹화

## 목적과 저장 범위

- 주요 목적: **동적 객체 필터를 적용한 multi-agent system이 잔상을 제거하면서 정적 구조를 보존하고 최종 merged map을 적절하게 만드는지** 확인
  - 개별 agent 지도와 최종 병합 지도 양쪽을 평가함
- 같은 bag의 **동일한 관측·경로·agent 변환**으로 off/on을 비교해 pose/정합 차이와 동적 필터 효과를 분리
  - 이후 live 시스템의 갱신율·지연도 확인
- 현재 높이·후방 필터를 유지한다면 **mapper 입력 filtered cloud + odometry + TF**로 충분
  - raw LiDAR/IMU는 기본에서 제외
- **기본 토픽:** 24개
  - `config/recording.yaml`에서 이름·필수 여부·기록 QoS를 관리함
  - 고정 world 도입 이후 `map_anchor`, `map_status`를 추가
- 녹화기도 실험4 공통 DDS NIC/SHM 정책을 적용하고 `configs/dds_profile.xml`과 `capture.yaml`의 `dds_transport`에 실제 설정을 저장
  - [네트워크 정책](experiment_04_dds_network.md)을 참고
- r1에서 보완한 코드가 이 저장소와 다를 경우 실제 토픽명을 확인
  - 이 문서의 이름은 현재 checkout의 launch·config에서 확인한 공개 토픽 기준

| 기본 저장 토픽 | 용도 |
| --- | --- |
| `/r0/mapping/plane_height_filtered`, `/r1/mapping/plane_height_filtered` | rear·height 전처리를 통과한 **mapper 입력** PointCloud2<br>동적 객체 필터 이전 관측 |
| `/rN/mapping/floor_based_odometry` | 고정 z·gravity 보정 단계의 pose<br>PHT 이전 비교 기준 |
| `/rN/mapping/occupancy_mapper_odometry` | PHT까지 적용한 pose<br>동적 필터 비교에서 경로를 고정하는 기준 |
| `/rN/odom` | raw ICP odometry 비교 기준 |
| `/tf`, `/tf_static` | 원래 TF와 양쪽 센서 장착 변환 |
| `/multi_agent/rN/aligned_odometry` | 당시 `map` 좌표계의 pose 결과<br>새 global_pose와 같은 pose이므로 중복 기록하지 않음 |
| `/multi_agent/r1_to_r0_transform` | 당시 고정된 agent mapping 좌표계 간 변환 |
| `/multi_agent/map_anchor`, `/multi_agent/map_status` | 고정 world anchor·coordinate epoch·지도 revision |
| `/multi_agent/alignment_ready`, `/multi_agent/alignment_status` | 정합 완료·입력 지연·session 상태 |
| `/multi_agent/rN/session`, `/rN/mapping/pipeline_diagnostics`, `/multi_agent/pose_relay_status` | 재시작·처리/발행/중계 지연 진단<br>미발행이면 누락 목록에 표시 |
| `/multi_agent/merged_global_occupancy` | 당시 최종 지도 기준 결과<br>r0 내부 구독 |
| `/parameter_events` | 실행 중 parameter 변경 이력<br>재생만으로 설정을 자동 복원하지는 않음 |

- `rN`은 r0·r1 각각을 뜻함
- `slice_kept_points`는 mapper의 occupied-hit 선택/좌표 변환 이후 디버그 cloud
  - free-ray에 사용할 관측이 빠질 수 있으므로 입력 cloud 대용으로 쓰지 않음
- path·axes는 odometry에서 재구성 가능
  - 중복 filtered/debug cloud와 agent별 local/global grid는 기본 녹화에서 제외
- r0 한 곳에 저장할 수 있지만 **r1 filtered cloud의 원격 전송량은 추가**됨
  - 필수 토픽이 존재한다는 사실만으로 메시지 무손실을 보장하지 않음

## 현재 병합 코드에서 먼저 구분할 점

- 현재 checkout의 개별 agent temporal 필터는 `occupancy_mapper/global_grid.inc`의 `updateGlobalGrid()`에서 global cell의 occupied→free 갱신을 처리
- r0는 정합 후 agent별 **local grid**를 수신
  - local grid 자체는 현재 scan의 occupied/free 관측이며 개별 global map의 temporal 필터 결과가 아님
- 중앙은 이제 `merged_dynamic_filter_enabled: true`일 때 `TemporalOccupancyAccumulator`로 새 관측을 직접 누적함
  - free 30회/관측 공백 5초/동일 0.1초 구간 중복 집계 방지를 기본으로 사용
  - [설정과 동작](experiment_04_temporal_fusion.md)을 참고
- agent의 `dynamic_filter_enabled`는 개별 global map용이고 중앙 설정과 독립적
  - 중앙 OFF는 기존 occupied 우선 누적이며 agent 필터만 ON으로 해서는 중앙 잔상이 지워지지 않음
- r1에서 별도 보완한 코드가 있다면 그 동작부터 재확인
  - 필터 OFF 상태로 취득해도 저장하는 cloud가 temporal 필터 이전 관측이므로 같은 입력으로 재평가 가능
  - cloud+pose 후처리와 local-grid 중앙 재생의 입력 조건은 아래에서 구분

## 실행 순서

1. 가능하면 **r0 녹화를 먼저 시작**하고 두 agent의 기존 매핑 명령을 실행
    - 초기 정지 관측·정합 과정을 포함하기 위함
2. r0의 `LOCKED`와 recorder의 `Required publishers visible`을 확인하고 잠시 정지 상태를 기록한 뒤 이동함
3. 이동과 동적 객체가 나타났다가 사라지는 관측을 저장
    - 물체가 이동한 뒤 같은 영역을 다시 관측하면 잔상 제거 평가에 유용
4. 종료 지점에서 잠시 정지한 뒤 **녹화 터미널 Ctrl+C**로 저장을 완료
    - 이 명령은 매핑·driver를 종료하지 않음
5. `capture_audit.yaml`을 확인하고, 매핑/두 PC 종료는 저장 완료 후 진행함

```bash
# r0의 별도 터미널. ROS 환경은 자동 source한다. 빌드는 필요 없다.
cd ~/co-3dto2d-mapping
bash scripts/tools/record_bag.sh --name hallway_dynamic
```

- 기본 ROS domain: 72
  - 다른 domain은 `--domain-id`로 지정
- 두 agent가 아직 꺼져 있어도 녹화를 시작 가능
  - recorder가 토픽을 동적으로 발견하고, 필수 발행자가 없으면 WAIT 목록을 출력
- 실제 메시지 수신 수는 종료 후 검사
  - publisher 표시만으로 데이터 품질 확인을 대신하지 않음
- 이미 매핑 중인 상태에서도 녹화 가능
  - 다만 시작 관측이 빠지고 raw 재처리 시 초기 odometry 원점이 달라질 수 있음
- 두 PC clock과 장치 header stamp의 시간 기준을 조정
  - 수집기가 timestamp를 임의로 보정하지 않음

### 포함하면 좋은 관측 상황

- 정적 벽·가구를 양쪽 agent가 보면서 이동: 잘못된 free 갱신으로 구조가 얇아지거나 사라지는지 평가
- 사람이 한곳에 머문 뒤 이동하고, 빈자리가 다시 보이는 상황: 잔상 제거와 clear 지연 평가
- 같은 동적 객체를 두 agent가 서로 다른 시점/방향에서 관측: 한쪽의 오래된 occupied 증거가 다른 쪽의 새 free 갱신을 막는지 평가
- 한동안 보이지 않던 영역을 다시 관측: unknown/관측 공백만으로 지우지 않고 새 free 관측으로 갱신하는지 평가
- 경로·치수 등의 수기 metadata 입력은 불필요
  - 관측 범위와 event 시점은 취득 후 bag에서 확인

## raw와 개별 지도는 필요한 경우에만 추가

```bash
# ICP부터 다시 계산하거나 height/rear 필터를 바꿀 계획이 있을 때
bash scripts/tools/record_bag.sh --name hallway_raw --include-raw

# 각 agent의 당시 local/global grid까지 별도로 비교할 때
bash scripts/tools/record_bag.sh --name hallway_maps --with-agent-maps

# 토픽 목록만 확인. 녹화·ROS discovery를 시작하지 않음
bash scripts/tools/record_bag.sh --dry-run
```

- `--include-raw`: `/rN/livox/lidar`, `/rN/livox/imu` 4개 추가
  - 기본 24개 → 28개
- `--with-agent-maps`: `/rN/mapping/local_occupancy`, `/rN/mapping/global_occupancy` 4개 추가
  - 원격 지도 전송과 저장량이 증가
- 현재 목적의 기본 권장은 **추가 옵션 없이 filtered 관측만 저장**하는 것
- filtered cloud에서 이미 제거한 높이·후방 관측은 나중에 복원 불가
  - raw를 제외한 bag으로 ICP 재계산이나 전처리 범위 확대를 수행할 수는 없음

## 저장 구조와 자동 metadata

```text
outputs/multi_agent/captures/hallway_dynamic_YYYYMMDD_HHMMSS/
  bag/metadata.yaml, *.db3          # ros2 bag 명령에 전달할 실제 URI는 bag/
  capture.yaml                    # branch/commit/domain/RMW/시각/선택 토픽/종료 상태
  capture_audit.yaml              # 필수 메시지·타입·수신 시간 범위·센서 TF 점검
  runtime_parameters.yaml         # 실제 노드 parameter service의 best-effort snapshot
  configs/recording_profile.yaml  # 이번 녹화 토픽 manifest
  configs/record_qos.yaml         # recorder가 실제 사용한 QoS
  configs/repository/             # 저장소 config 경로별 사본; 원격 실설정과 일치 여부는 별도 확인
  logs/record.log                 # rosbag recorder 로그
  record_command.json, working_tree.patch
```

- 별도 수기 경로·속도·바닥 높이·장애물 치수 입력은 요구하지 않음
- `capture.yaml`은 시작/종료 시 갱신
  - 실제 node parameter는 필수 발행자가 보인 뒤 한 번 조회하며 실패/timeout을 명시
  - 서비스 응답이 없으면 실제 r1 config를 확보했다고 간주하지 않음
- r1 Git revision이나 r1의 미커밋 소스를 r0 저장소에서 추정하지 않음
  - 두 노트북 코드 동기화는 별도로 유지
- **저장 기본값:** SQLite3 · 5분 단위 분할 · 압축 없음 · recorder cache 128MiB
  - rosbag cache는 이중 buffer이므로 최대 약 2배 cache 메모리가 필요할 수 있음
- 데이터 토픽은 Best Effort로 기록해 기존 publisher의 신뢰성 요구를 높이지 않음
  - `/tf_static`은 Reliable/Transient Local로 받아 늦게 시작해도 장착 TF를 수집함
  - 다른 latched 결과는 Best Effort/Transient Local로 수신
- 종료 후 SQLite 메시지 인덱스와 `/tf_static`을 읽어 점검함
  - cloud 본문 전체를 재분석하지 않음

## 종료 점검 결과 해석

- `filtered_mapping_minimum_complete: true`
  - 두 agent 입력 cloud·필수 pose가 기록됐고 타입·수신 시간 범위가 맞으며, 두 base↔sensor static TF 경로가 있음
- `fixed_pose_reference_minimum_complete: true`
  - floor/fixed·mapper 보정 pose가 양쪽 모두 있음
- `alignment_reference_available: true`
  - 고정 agent 변환이 기록됨
    - true만으로 정합 정확도 보장 불가
- `world_anchor_reference_available: true`
  - `map ← r0/mapping_odom` anchor가 기록됨
    - 현재 capture의 epoch와 일치하는지는 `map_status`/`alignment_status`와 함께 확인
- `alignment_was_ready: true`, `multi_agent_minimum_complete: true`
  - 실제 `alignment_ready=true` 메시지와 필수 관측·pose·변환을 저장
    - 최종 merged map 정확도나 전 구간 무지연을 보장하지 않음
- `merged_map_reference_available: true`
  - 당시 merged map이 하나 이상 저장돼 결과 비교 기준이 있음
- `raw_replay_minimum_complete`
  - `--include-raw`를 사용했을 때만 true가 될 수 있음
    - 기본 false는 선택적으로 raw를 저장하지 않았다는 의미
- `missing_topics`, `type_mismatches`, `missing_sensor_static_paths`
  - 필수 누락이면 종료 시 오류도 표시
    - 생성된 bag은 지우지 않음
- 이 점검은 최소 적합성 검사
  - 장시간 공백·header timestamp·clock 정합·송신 대비 누락률은 취득 후 추가 분석함

## 후처리 시 비교 조건

- 동적 객체 필터 비교는 **같은 filtered cloud·같은 기록 pose·같은 agent 변환**을 두 조건에 공급함
- agent 재시작/session 변경이 있으면 epoch를 나눠 같은 변환 이력을 사용
  - 모든 시각에 첫 변환 하나만 강제하지 않음
- 기록된 `occupancy_mapper_odometry`를 입력으로 쓸 때 PHT/다른 pose 보정을 다시 적용하지 않음
  - 입력 reference 토픽과 새 출력 토픽도 구분해 자기 구독 루프를 피함
- 기존 실험 4 `replay_experiment_04_multi_agent_mapping.sh`는 raw LiDAR/IMU부터 재계산하는 스크립트
  - **기본 filtered-only bag에는 그대로 사용 불가**
  - cloud+pose로 mapper를 다시 실행하려면 해당 입력 전용 파이프라인 필요
- `--with-agent-maps`로 local grid까지 저장한 경우: `scripts/experiment/04_multi_agent_mapping/replay_experiment_04_mapping_inputs.py`로 중앙 병합만 재생 가능
  - 기록된 초기 변환·pose·session과 원본 local-grid timestamp 사용
  - 상세 조건: [주기 정합·필터 비교](experiment_04_periodic_alignment.md)
- raw 옵션으로 기록한 bag은 기존 실험 4 raw replay의 입력 조건을 갖출 수 있음
  - 이 경우 ICP/PHT 재계산 변동을 동적 필터 개선 효과와 혼동하지 않음
- 일반 `ros2 bag play`로 저장 결과를 보는 것과, 필터 알고리즘을 다시 실행하는 후처리는 별개
  - 기존 동적 `/tf`와 재계산된 TF를 동시에 발행하지 않음
  - 재생 clock은 player 한 곳에서 생성
- 녹화된 센서가 있으면 두 노트북·실센서가 꺼져 있어도 후처리가 가능
  - r0에 도착하지 않아 저장되지 않은 관측은 복원 불가

## 평가 항목

- **동적 잔상**: 떠난 자리의 occupied cell 잔존량과 free 재관측 후 clear까지의 시간
- **정적 보존**: 벽·얇은 가구의 occupied 유지, 두께 감소·구멍·잘못된 제거
- **병합 품질**: 두 agent 증거가 겹치는 곳의 잔상·이중 벽·불연속
  - pose/정합 오차에 따른 차이는 별도로 구분
- **실시간 동작**: 실제 처리·발행·병합 갱신율, 수신 공백/age, 처리시간과 큐 누적
- 개별 global map의 개선만으로 최종 merged map 개선을 판정하지 않음
  - 정답 label 없이 계산한 잔존 cell 수 등은 해당 ROI와 관측 조건을 함께 보고함
