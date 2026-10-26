# MID-360 Single / Multi-agent 2D Mapping

- **목적:** LiDAR·IMU로 로봇 pose와 2D occupancy map을 생성하고, 두 agent의 지도를 공통 `map` 좌표계로 병합합니다.
- **현재 브랜치:** `features/jeongeon/optimization` · Ubuntu 22.04 / ROS 2 Humble 기준.
- **기본 동작:** raw ICP → 고정 z·IMU gravity 보정 → PHT 기반 XY/yaw 보정 → occupancy 생성.
- **높이 기준:** 바닥 plane을 탐지하지 않고 설정한 센서 장착 높이를 사용합니다. 평탄한 동일 층·고정 장착을 전제로 합니다.

**바로가기:** [실행](#1-설치실행) · [Single](#2-single-agent-mapping) · [설정](#3-주요-파라미터) · [동적 필터](#4-동적-객체-흔적-제거) · [Multi](#5-multi-agent-mapping) · [Frame](#6-frame과-회사-시스템-연동) · [저장](#7-지도-저장과-origin) · [녹화·재생](#8-rosbag-녹화재생)

## 1. 설치·실행

### 설치

```bash
git clone --branch features/jeongeon/optimization --single-branch \
  https://github.com/Epsilon8854/co-3dto2d-mapping.git
cd co-3dto2d-mapping
bash scripts/tools/setup_environment.sh
```

- ROS 2 Humble은 먼저 설치합니다. Livox SDK/driver 빌드와 센서 IP·host IP 설정은 [설치 안내](docs/experiment_04_setup.md)를 참고합니다.
- 이미 설치된 PC는 `git pull --ff-only` 후 `bash scripts/tools/setup_environment.sh --skip-install`로 재빌드합니다.
- 설치·빌드는 다른 mapping workspace가 source되지 않은 새 터미널에서 수행합니다. 아래 실행 명령은 저장소 루트 기준입니다.

### ▶ Single-agent: 로컬 driver + mapping + RViz

```bash
bash scripts/run_single_agent.sh --robot-id 0
```

### ▶ Multi-agent: r0 PC — 로컬 driver + mapping + 중앙 병합 + RViz

```bash
bash scripts/run_multi_agent.sh --role coordinator --robot-id 0
```

### ▶ Multi-agent: r1 PC — 로컬 driver + mapping

```bash
bash scripts/run_multi_agent.sh --role agent --robot-id 1
```

- 두 PC는 같은 코드 revision, ROS domain **72**, DDS 통신 가능한 네트워크, 동기화된 clock을 사용합니다. 논리 ID r0/r1은 물리 노트북에 고정되지 않습니다.
- **Multi 시작 시:** 가능한 두 센서의 전방을 같은 방향으로 맞추고 초기 yaw 차이를 작게 유지하면 정합 안정성에 유리합니다. 공통 벽·코너를 관측하며 `LOCKED`까지 정지합니다. 다른 방향의 초기 정합도 지원합니다.
- r0에서 초기 정합 브라우저 모니터가 자동으로 열립니다. RViz와 별도 화면입니다. 터미널에도 입력 대기·수집·검색·정합 완료 상태가 표시됩니다.
- driver는 기본 자동 시작·종료합니다. 같은 물리 센서의 중복 driver를 거절하며, Ctrl+C는 해당 실행기가 소유한 프로세스를 정리합니다.

| 실행 옵션 | 용도 |
|---|---|
| `--livox-workspace /absolute/ws_livox` | 자동 탐지되지 않는 Livox workspace 지정 |
| `--driver-config /absolute/MID360_config.json` | 해당 PC·센서의 IP 설정 파일 지정 |
| `--no-start-driver` | 이미 별도로 관리하는 driver 사용 |
| `--domain-id 72` | ROS domain 선택 |
| `--robot-config FILE`, `--sensor-config FILE` | 공통 mapping / 해당 센서 설정 선택 |
| `--alignment-config FILE` | Multi 중앙 정합·병합 설정 선택 |
| `--no-rviz`, `--no-alignment-viewer` | 자동 시각화 창 생략 |
| `--dry-run` | 센서·노드 실행 없이 역할·설정 경로 확인 |

## 2. Single-agent mapping

### Workflow

```mermaid
flowchart LR
    R[Raw LiDAR] --> I[RTAB-Map ICP]
    R --> B[Rear filter]
    B --> H[Height filter]
    M[IMU + Madgwick] --> U[Gravity up 방향]
    U --> H
    I --> C[고정 z + gravity roll/pitch]
    U --> C
    H --> S[몸체 제외 + 거리 제한<br/>2D 점유 관측]
    C --> P[PHT 선분 매칭<br/>XY/yaw pose 보정]
    S --> P
    P --> L[Local occupancy]
    P --> G[누적 Global occupancy]
    P --> O[Mapping odometry]
```

- **ICP:** raw cloud로 이동을 추정합니다. 후방·높이 필터는 ICP 입력에 적용하지 않습니다. 기본 deskew OFF, 구독 queue 1입니다.
- **Gravity 보정:** IMU에서 up 방향을 구해 roll/pitch를 정렬하고 z를 설정값에 고정합니다. XY/yaw는 ICP 결과를 전달합니다.
- **Height filter:** gravity 방향의 높이에 센서 장착 높이를 더해 바닥 기준으로 환산합니다. 기본 `0.30~1.20m`를 보존합니다. pose 보정 ON/OFF와 별도로 동작합니다.
- **PHT:** Probabilistic Hough Transform으로 2D 점유 관측에서 선분을 추출합니다. 보정된 과거 keyframe을 기본 5개 누적해 현재 선분과 매칭하고, raw odometry prior와 함께 local pose graph로 XY/yaw를 보정합니다. 기본 모드는 `full_se2`입니다.
- **Occupancy:** 보정 pose로 local/global 격자를 생성합니다. 장애물 끝점은 occupied, 유효 ray가 통과한 셀은 free로 관측합니다. 특징·overlap이 부족하면 보정 효과가 제한될 수 있습니다.

### 주요 토픽

- `N`은 `0` 또는 `1`입니다. 타입은 `패키지/msg/타입` 형식입니다.
- 아래 `mapping 기준`은 **single에서 `rN/odom`**, **multi에서 `rN/mapping_odom`**입니다.

| 토픽 | 메시지 타입 | 내용 / header frame |
|---|---|---|
| `/rN/livox/lidar` | `sensor_msgs/msg/PointCloud2` | Raw cloud · `rN/livox_frame` |
| `/rN/livox/imu` | `sensor_msgs/msg/Imu` | Raw IMU · driver 설정 frame |
| `/rN/mapping/imu_filtered` | `sensor_msgs/msg/Imu` | Madgwick orientation·IMU · `rN/livox_frame` |
| `/rN/mapping/lidar_rear_filtered` | `sensor_msgs/msg/PointCloud2` | 후방 필터 결과 · `rN/livox_frame` |
| **`/rN/mapping/plane_height_filtered`** | **`sensor_msgs/msg/PointCloud2`** | **후방 + 높이 필터 결과** · `rN/base_link` |
| `/rN/odom` | `nav_msgs/msg/Odometry` | Raw ICP pose · `rN/odom` |
| `/rN/mapping/floor_based_odometry` | `nav_msgs/msg/Odometry` | 고정 z·gravity 자세 보정 pose · mapping 기준 |
| `/rN/mapping/occupancy_mapper_odometry` | `nav_msgs/msg/Odometry` | PHT 보정까지 반영한 최종 mapping pose · mapping 기준 |
| `/rN/mapping/local_occupancy` | `nav_msgs/msg/OccupancyGrid` | 현재 관측의 local map · mapping 기준 |
| `/rN/mapping/global_occupancy` | `nav_msgs/msg/OccupancyGrid` | 해당 agent의 누적 map · mapping 기준 |
| `/rN/mapping/slice_kept_points` | `sensor_msgs/msg/PointCloud2` | 몸체·거리 제한까지 적용하고 mapping pose로 투영한 점 · mapping 기준 |
| `/rN/mapping/debug/raw_odometry_path` | `nav_msgs/msg/Path` | ICP 비교 궤적 · raw odometry 기준 |
| `/rN/mapping/debug/floor_based_odometry_path` | `nav_msgs/msg/Path` | 고정 높이·gravity 보정 궤적 · mapping 기준 |
| `/rN/mapping/debug/local_submap_corrected_odometry_path` | `nav_msgs/msg/Path` | PHT 보정 궤적 · mapping 기준 |
| `/rN/mapping/pipeline_diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | mapper 수신·처리·지연 계측 |

- `floor_based_odometry`, `plane_height_filtered`는 기존 인터페이스 이름을 유지한 것입니다. 현재 운영 경로는 **LiDAR 바닥 plane을 추정하지 않습니다.**
- `/plane_height_filtered`에는 몸체 제외·거리 제한이 아직 적용되지 않습니다. 지도와 정렬된 최종 점군을 비교하려면 `/slice_kept_points`를 확인합니다.

## 3. 주요 파라미터

### 수정할 파일

| 파일 | 수정 내용 |
|---|---|
| [`config/single_agent.yaml`](config/single_agent.yaml) | 공통 높이·자세, 몸체 제외, occupancy, PHT, single 동적 필터 |
| [`config/sensors/r0.yaml`](config/sensors/r0.yaml), [`r1.yaml`](config/sensors/r1.yaml) | 센서 TF·rear filter·선택 Livox 경로 |
| [`config/multi_agent.yaml`](config/multi_agent.yaml) | `central`: 중앙 정합·병합·동적 필터, `agent_overrides`: multi에서 덮어쓸 공통 설정 |
| [`config/lidar_odometry.yaml`](config/lidar_odometry.yaml) | RTAB-Map ICP·deskew |
| [`config/dds.yaml`](config/dds.yaml) | DDS NIC·버퍼·전송 정책 |
| [`config/recording.yaml`](config/recording.yaml) | 녹화 토픽·QoS |

- 아래 기본값은 현재 운영 YAML 기준입니다. 공통 mapping 설정은 `/**/ros__parameters` 아래에 둡니다.
- 센서 TF·rear 설정은 sensor YAML에 둡니다. 변경한 설정은 노드를 재시작해 적용하고, multi 연동 변경은 양쪽 PC에서 업데이트합니다.

### 높이·자세·후방·몸체 제외

| 파라미터 | 기본값 | 의미 / 변경 효과 |
|---|---|---|
| `lidar_mount_height_above_floor_m` | `0.30m` | 높이 필터용 장착 높이. 바닥 기준 환산값이 달라짐 |
| `fixed_base_link_height_above_floor_m` | `0.30m` | 보정 pose의 고정 z. 센서 높이와 별도로 설정 |
| `height_filter_enabled` | `true` | 높이 필터 ON/OFF |
| `height_filter_min_m`<br>`height_filter_max_m` | `0.30` / `1.20m` | 바닥 기준 보존 구간. min 증가 시 낮은 장애물도 제외 |
| `pose_correction_enabled` | `true` | 고정 z·gravity 자세 보정 ON/OFF. 높이 필터는 별도 |
| `gravity_filter_gain` | `0.25` | 증가하면 자세 변화에 빠르게 반응, 작으면 더 평활화 |
| `imu_timeout_sec`<br>`gravity_hold_timeout_sec` | `0.30` / `0.75s` | IMU 유효 시각차 / 마지막 gravity 유지 시간 |
| `rear_filter.enabled` | `true` | 후방 필터 ON/OFF · sensor YAML |
| `rear_filter.angle_deg`<br>`rear_filter.axis` | `120°` / `-x` | 센서 좌표계에서 제거할 전체 부채꼴 각도 / 중심 방향 |
| `sensor_tf.x/y/z` / `roll/pitch/yaw` | 모두 `0` | `base_link`에 대한 센서 장착 위치(m)·회전(rad) |
| **`robot_body_exclusion_box_size_m`** | **`1.6m`** | `base_link` 중심 XY 정사각형의 **전체 한 변**. ±0.8m 내부 점 제외. `0`이면 OFF |

- 몸체 제외 크기는 로봇 치수에서 자동 계산하지 않습니다. 실제 몸체와 여유에 맞춰 설정하며, 과도하게 키우면 가까운 장애물도 지도에서 빠집니다.
- 센서 높이 0.30m·보존 높이 0.30~1.20m는 gravity 방향으로 센서 기준 약 **0~0.90m**에 해당합니다. 장착 TF가 0이 아닌 경우 좌표 변환도 함께 고려합니다.

### 지도·선분 매칭

| 파라미터 | 기본값 | 의미 / 변경 효과 |
|---|---|---|
| `grid_resolution` | `0.05m` | 작을수록 세밀하며 연산·메모리 증가 |
| `local_map_size_m` | `20m` | local 정사각 격자의 한 변 |
| `range_min_m`<br>`range_max_m` | `0.8` / `12m` | occupied 끝점에 사용할 XY 거리 구간 |
| `raycast_min_range_m`<br>`raycast_max_range_m` | `0.8` / `12m` | free-space ray 처리 거리 제한 |
| `publish_period_ms` | `200ms` | local/global 지도 발행 주기. 처리·입력 속도와는 별개 |
| `local_pht_submap_pose_graph_enabled` | `true` | PHT 기반 pose 보정 활성화 |
| `local_pht_submap_pose_graph_mode` | `full_se2` | `visualization_only`: 보정 OFF / `single_line`: yaw·법선 방향 이동 / `full_se2`: 비평행 선분으로 XY/yaw 보정 |
| `local_pht_submap_history_keyframes` | `5` | target에 누적할 과거 keyframe 수. 프레임마다 무조건 추가되는 값은 아님 |
| `local_pht_submap_keyframe_translation_m`<br>`local_pht_submap_keyframe_yaw_deg` | `0.5m` / `5°` | 새 keyframe을 추가할 이동·회전 기준 |
| `local_line_pht_min_length_m` | `0.70m` | 최소 선분 길이. 증가하면 짧은 선분 제외 |
| `local_line_pht_max_gap_m` | `0.10m` | Hough 후보 추출에서 연결을 허용할 gap |
| `local_line_pht_max_contiguous_gap_m` | `0.15m` | 추출 후 선분 내부에 허용할 연속 관측 공백 |
| `local_line_pht_min_occupied_support_ratio` | `0.70` | 선분을 뒷받침하는 occupied 비율. 증가하면 더 엄격 |
| `local_pht_submap_max_angle_deg`<br>`local_pht_submap_max_normal_distance_m` | `10°` / `0.35m` | 매칭 후보의 각도·법선 거리 허용 범위 |
| `local_pht_submap_min_overlap_m` | `0.50m` | 매칭 선분의 최소 중첩 길이 |
| `local_pht_submap_weight` | `0.20` | raw odometry prior에 대한 선분 제약 가중치 |

- Hough vote threshold는 현재 C++ 추출 설정값이며 **운영 YAML에 노출한 파라미터가 아닙니다.**
- `config/lidar_odometry.yaml`의 `deskewing: false`를 기본으로 사용합니다. ICP의 queue 1은 공용 launch에서 지정합니다.

## 4. 동적 객체 흔적 제거

- **방식:** 객체를 분류·추적하는 대신, 이전 occupied 셀에서 반복 확인한 free 관측으로 흔적을 지웁니다.
- **현재 기본:** single 내부 필터 **OFF**, multi 중앙 필터 **ON**. 중앙은 양쪽 agent의 local 관측을 함께 사용합니다.

```mermaid
flowchart LR
    O[기존 occupied 셀] --> F[Free 관측 누적]
    F -->|기준 충족| C[Free로 갱신]
    F -->|Occupied 재관측| Z[횟수 0 · occupied 유지]
    F -->|관측 공백 초과| T[횟수 0 · 기존 셀 유지]
```

| Single: `single_agent.yaml` | Multi: `multi_agent.yaml`의 `central` | 기본값 / 효과 |
|---|---|---|
| `dynamic_filter_enabled` | `merged_dynamic_filter_enabled` | `false` / `true` |
| `dynamic_free_clear_count` | `merged_dynamic_free_clear_count` | `30` · 증가하면 삭제가 보수적이며 흔적이 오래 유지 |
| `dynamic_evidence_timeout_sec` | `merged_dynamic_evidence_timeout_sec` | `5s` · 관측 공백 후 연속 free 횟수 초기화 |
| `dynamic_clear_max_range_m` | `merged_dynamic_clear_max_range_m` | `6m` · 기존 occupied 삭제를 허용할 관측 거리 |
| 해당 없음 | `merged_dynamic_observation_bin_sec` | `0.1s` · 같은 셀·시간 bin에서 두 agent를 합쳐 free 최대 1회 인정 |

- Unknown·미관측은 free 증거가 아닙니다. Timeout으로 occupied 셀이 자동 삭제되지 않습니다.
- 중앙은 원 관측 timestamp를 사용하며 중복·과거 관측을 검사합니다. 같은 bin에서 occupied와 free가 섞이면 occupied가 우선합니다.
- `30회`가 고정된 삭제 시간을 뜻하지 않습니다. 실제 유효 관측률·가림·occupied 재관측에 따라 달라집니다.
- 정적 장애물이 얇아지면 clear count 증가·clear 거리 축소를 검토합니다. 단순히 더 먼 hit가 보인다고 모든 중간 셀이 유효 free로 누적된 것은 아닙니다.

## 5. Multi-agent mapping

### Workflow와 교환 정보

```mermaid
flowchart TD
    A[r0 Single pipeline] --> G0[초기 누적 Global map]
    B[r1 Single pipeline] --> G1[초기 누적 Global map]
    G0 --> R[전체 yaw 후보 검색 + XY 정합<br/>Overlap · RMSE · Free 충돌 검증]
    G1 --> R
    R --> T[LOCKED: r1 → r0 변환 확정]
    A --> L0[주행 중 Local map + Mapping pose]
    B --> L1[주행 중 Local map + Mapping pose]
    T --> W[고정 map 기준으로 좌표 변환]
    L0 --> W
    L1 --> W
    W --> D[셀별 시간 증거 갱신<br/>Occupied / Free 병합]
    D --> G[Merged global map]
    W --> P[별도 pose·TF 중계<br/>각 agent Global pose]
```

- **초기:** 정지 상태의 누적 global map으로 r1→r0 변환을 구합니다. 전체 yaw를 검색하되 overlap·오차·occupied/free 충돌과 후보 구분 조건을 통과해야 확정합니다.
- **주행:** 초기 확정 뒤 global map 구독을 해제하고 local map·mapping pose·session을 받아 병합합니다. r1의 전송된 점유 좌표는 r1 기준이며 중앙에서 공통 좌표로 변환합니다.
- **통신:** 중앙 병합·pose 중계는 원격 raw cloud/IMU를 입력으로 쓰지 않습니다. 같은 ROS domain에서 rqt·녹화기로 해당 토픽을 추가 구독하면 별도 전송 비용이 생깁니다.
- **분리:** pose/TF 중계는 지도 정합·누적·경로·파일 저장과 별도 프로세스입니다.
- **주기 정합:** 기본 OFF입니다. `central.periodic.enabled: true`이면 r1의 최근 누적 submap을 r0의 과거 누적 지도 영역과 비교하고, 채택 시 r1→r0 변환과 기존 관측 재투영을 갱신합니다. r0 pose나 고정 world anchor를 이 inter-agent 정합으로 보정하지 않습니다. [상세 동작](docs/experiment_04_periodic_alignment.md)

### 주요 공개 토픽

| 토픽 | 메시지 타입 | 내용 / 기준 |
|---|---|---|
| **`/multi_agent/merged_global_occupancy`** | `nav_msgs/msg/OccupancyGrid` | **최종 병합 지도 · `map`** |
| `/multi_agent/r1/aligned_global_occupancy` | `nav_msgs/msg/OccupancyGrid` | 공통 좌표로 변환·누적한 r1 layer · `map` |
| **`/multi_agent/rN/global_pose`** | `geometry_msgs/msg/PoseWithCovarianceStamped` | **각 agent의 전역 pose · `map`** |
| `/multi_agent/rN/aligned_odometry` | `nav_msgs/msg/Odometry` | 전역 pose와 기존 body 속도 추정 · `map`, child=`rN/base_link` |
| `/multi_agent/rN/aligned_path`, `/multi_agent/rN/raw_path` | `nav_msgs/msg/Path` | 최종 보정 / raw 비교 궤적 · `map` |
| `/multi_agent/r1_to_r0_transform` | `geometry_msgs/msg/TransformStamped` | `r0/mapping_odom ← r1/mapping_odom` 변환 |
| `/multi_agent/map_anchor` | `geometry_msgs/msg/TransformStamped` | 고정 `map ← r0/mapping_odom` 변환 |
| `/multi_agent/alignment_ready` | `std_msgs/msg/Bool` | 초기 정합 완료 여부 |
| `/multi_agent/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 정합 단계·후보·입력 지연·변환 정보 |
| `/multi_agent/map_status` | `diagnostic_msgs/msg/DiagnosticArray` | 지도 ready·revision·coordinate epoch |
| `/multi_agent/pose_relay_status` | `diagnostic_msgs/msg/DiagnosticArray` | 전역 pose 발행·TF 대기·지연 상태 |
| `/multi_agent/rN/session` | `diagnostic_msgs/msg/DiagnosticArray` | agent 재시작·입력 reset 식별 |


### 초기 정합·병합 설정

- 파일: `config/multi_agent.yaml`의 **`central` 아래**.

| 파라미터 | 기본값 | 의미 / 변경 효과 |
|---|---|---|
| `startup_accumulation_sec` | `3s` | 초기 누적 지도 수집 기준. 전체 정합 소요시간을 보장하지 않음 |
| `confirmation_windows` | `1` | 채택 후보 확인 횟수. 증가하면 시간적 일관성 검증 강화 |
| `search.coarse_yaw_step_deg` | `5°` | 전체 yaw 후보 간격. 작을수록 탐색량 증가 |
| `registration.coarse_translation_range_m` | `2.5m` | 초기 XY 검색 범위 |
| `registration.min_symmetric_overlap` | `0.30` | 최소 양방향 중첩 비율. 증가하면 엄격 |
| `registration.max_symmetric_rmse_m` | `0.25m` | 정합 RMSE 상한. 감소하면 엄격 |
| `registration.max_free_conflict_ratio` | `0.18` | 비교 가능한 occupied/free 모순 비율 상한. 성공 확률을 뜻하지 않음 |
| `submap_radius_m` | `12m` | 초기 정합용 입력 patch 반경 |
| `output_resolution_m`<br>`merged_period_sec` | `0.05m` / `0.5s` | 병합 격자 해상도 / 처리 스케줄 기준 간격 |
| `local_queue_size` | `100` | 중앙 local map 대기열. 연산 자체를 빠르게 하지는 않음 |
| `periodic.enabled` | `false` | 초기 정합 이후 주기적 inter-agent 정합·재투영 활성화 |

## 6. Frame과 회사 시스템 연동

| Frame | 역할 |
|---|---|
| **`map`** | Multi 공통 고정 world. 중앙이 처음 수신한 r0 mapping pose의 **XY·yaw**를 기준으로 원점·방향 설정 |
| `rN/odom` | RTAB-Map raw ICP odometry의 로컬 기준 |
| `rN/mapping_odom` | Multi에서 고정 높이·gravity·PHT mapping pose와 지도의 내부 기준 |
| `rN/base_link` | 로봇 몸체 좌표계 |
| `rN/livox_frame` | 몸체에 고정된 LiDAR 좌표계 |

```mermaid
flowchart TD
    M[map: 공통 world] -->|보정 TF| O0[r0/odom]
    M -->|보정 TF| O1[r1/odom]
    O0 -->|Raw ICP TF| B0[r0/base_link]
    O1 -->|Raw ICP TF| B1[r1/base_link]
    B0 -->|장착 TF| L0[r0/livox_frame]
    B1 -->|장착 TF| L1[r1/livox_frame]
    M -->|고정 anchor A| S0[r0/mapping_odom]
    M -->|anchor A + 정합 C| S1[r1/mapping_odom]
```

- $A = {}^{map}T_{r0/mapping\_odom}$: 최초 r0 XY/yaw 기준의 고정 anchor. $C = {}^{r0/mapping\_odom}T_{r1/mapping\_odom}$: agent 간 정합 변환.
- 전역 pose는 $P_{0,global}=A P_0$, $P_{1,global}=A C P_1$입니다. 기본 base z는 바닥 기준 0.30m이며 초기 XY/yaw를 0으로 잡는 것과 구분합니다.
- Raw `odom→base_link`는 ICP가 발행합니다. 중앙은 같은 source timestamp에서 아래 TF를 계산합니다.

$$
T_{map\leftarrow odom}(t)=T_{map\leftarrow base,\ corrected}(t)\,T_{odom\leftarrow base,\ raw}(t)^{-1}
$$

- **회사 측 연동:** goal·global path·현재 global pose를 모두 `map` 기준으로 사용합니다. 보정 시 `map` 원점은 유지되고 로봇 pose·지도 관측 위치가 갱신됩니다.
- 지도 변경 후 goal의 도달 가능성·경로·frontier 재선택은 주행/exploration 시스템에서 판단합니다. 이 저장소는 goal을 자동 변경하지 않습니다.
- `map_status`의 `coordinate_epoch`가 바뀌면 기존 goal/path의 좌표 유효성을 다시 확인합니다. 중앙 재시작·r0 session reset 후에도 같은 절대 좌표가 유지된다고 가정하지 않습니다.
- RViz Fixed Frame: **single은 `rN/odom`, multi는 `map`**. Multi의 전역 TF·pose는 초기 정합과 동일 시각 raw TF가 준비되어야 발행됩니다.

## 7. 지도 저장과 Origin

### 저장 위치·형태

- **Merged map은 중앙 r0 PC에 저장**합니다. 초기 정합이 완료되지 않으면 merged map 대신 agent별 미정합 지도가 저장됩니다.

```text
outputs/
├── single_agent/runs/20261026_143025/
│   ├── configs/                         # 원본·최종 적용 YAML, DDS 설정
│   ├── run.yaml                         # 실행 시각·인자·Git revision
│   └── cycle_001/maps/global_map.*
└── multi_agent/runs/20261026_143025/
    ├── configs/
    ├── run.yaml
    └── cycle_001/
        ├── maps/
        │   ├── merged_global_map.pgm
        │   ├── merged_global_map.yaml
        │   ├── merged_global_map.png
        │   ├── merged_global_map_with_paths.png
        │   ├── merged_global_map_data.npz
        │   ├── r0_global_map.*
        │   └── r1_aligned_global_map.*
        ├── checkpoint/                  # 기본 10초 간격의 중간 저장
        ├── alignment/                   # r1→r0 변환
        └── save_status.json             # world anchor·epoch·revision·저장 상태
```

- 폴더명은 한국 시각 **`YYYYMMDD_HHMMSS`**입니다. `cycle_001`은 첫 처리 회차이며 주행한 바퀴 수가 아닙니다. 반복 재생은 `cycle_002` 이후로 구분합니다.
- `--run-id 20261026_143025_airoom`으로 식별명, `--output-root outputs/experiment_NAME`으로 실험별 저장 위치를 지정할 수 있습니다.
- PNG는 검토용, PGM+YAML은 map server용, NPZ는 원 격자·좌표·timestamp 분석용입니다. `outputs/`·bag·빌드 결과는 Git에 포함하지 않습니다.
- **정상 Ctrl+C 종료 시 최종 지도를 저장**합니다. 중앙 지도는 기본 10초 간격으로 checkpoint도 저장하며, 탐사 완료 등 원하는 시점에 다음 서비스를 호출할 수 있습니다.

```bash
bash scripts/tools/with_dds.sh ros2 service call \
  /multi_agent/save_map std_srvs/srv/Trigger '{}'
```

### map 원점과 YAML origin의 차이

- **`map` 원점:** 세션 공통의 고정 좌표 기준.
- **YAML `origin: [x, y, yaw]`:** 저장 격자의 좌하단이 `map`의 어디에 있는지 나타내는 값. `map` 원점 자체와 다르며, 음수여도 정상입니다.
- `resolution`은 한 셀의 폭(m)입니다. 열 `c`·아래에서부터 행 `r`인 셀 중심은 다음과 같습니다. 이미지 파일의 위에서부터 행 번호는 반대로 변환해야 합니다.

$$
p_{map}=\begin{bmatrix}x_0\\y_0\end{bmatrix}+R(\theta_0)\,resolution\begin{bmatrix}c+0.5\\r+0.5\end{bmatrix}
$$

- Occupancy 값은 unknown=`-1`, free=`0`, occupied=`100`입니다. 회사 측 map server의 frame도 `map`으로 설정합니다.
- 지도 확장으로 YAML의 `origin`이 달라져도 세션의 고정 `map` 좌표 기준이 이동한 것은 아닙니다.

## 8. Rosbag 녹화·재생

### 녹화: 중앙 r0 PC 한 곳에서 두 agent 저장

```bash
# 기본: 후방·높이 필터 cloud, pose, TF, 정합 상태
bash scripts/tools/record_bag.sh --name airoom_dynamic

# 중앙 정합·동적 필터를 동일 local 입력으로 재현하려면 지도도 포함
bash scripts/tools/record_bag.sh --name airoom_dynamic --with-agent-maps

# ICP·IMU·높이 필터부터 다시 계산하려면 raw 포함 (원격 통신 증가)
bash scripts/tools/record_bag.sh --name airoom_raw --include-raw --with-agent-maps

# Single 녹화
bash scripts/tools/record_bag.sh --pipeline single --robot-id 0 --name hallway --include-raw
```

- 녹화는 mapping/driver를 시작·종료하지 않습니다. 각 agent의 mapping은 별도로 실행합니다.
- 저장 위치는 `outputs/multi_agent/captures/NAME_YYYYMMDD_HHMMSS/bag/`입니다. Single은 `single_agent` 아래에 저장합니다. 설정·취득 metadata는 `bag/`의 상위 폴더에 저장됩니다.
- **기본 bag에는 raw LiDAR/IMU가 없습니다.** Filtered cloud+기록 pose로 mapper를, local map까지 기록했다면 중앙 병합을 후처리할 수 있습니다. ICP부터 재계산하려면 `--include-raw`가 필요합니다.

### 재생: raw LiDAR/IMU를 포함한 bag

```bash
# URI는 metadata.yaml을 포함하는 bag 디렉터리
bash scripts/tools/replay_bag.sh --pipeline single --robot-id 0 \
  --bag /absolute/capture/bag --loop

bash scripts/tools/replay_bag.sh --pipeline multi \
  --bag /absolute/combined_capture/bag --loop
```

- Multi raw 재생에는 두 agent의 raw 입력과 센서 TF가 필요합니다. `--rate 0.5`로 감속하고, `--loop`를 생략하면 한 번만 재생합니다.
- 같은 domain의 실센서·다른 player를 종료하거나 `--domain-id`로 분리합니다. 재생은 단일 clock을 사용하며 회차마다 노드를 새로 시작해 상태를 초기화합니다.
- 별도 RViz/rqt/CLI는 같은 DDS 환경으로 실행합니다: `bash scripts/tools/with_dds.sh rqt`.

## 9. 저장소 구조·상세 문서

| 경로 | 용도 |
|---|---|
| `co_3dto2d_mapping/common/` | 공통 좌표 변환·격자·저장 |
| `co_3dto2d_mapping/single_agent/` | 높이·gravity·rear 필터, single 저장 |
| `co_3dto2d_mapping/multi_agent/` | 정합·중앙 병합·pose/TF 중계 |
| `include/`, `src/` | 현재 occupancy/PHT C++ mapper |
| `scripts/run_single_agent.sh`, `scripts/run_multi_agent.sh` | 공용 실시간 실행기 |
| `scripts/tools/`, `scripts/lib/` | 설치·녹화·재생·진단 / 실행·DDS·driver 내부 처리 |
| `rviz/single_agent.rviz`, `rviz/multi_agent.rviz` | 운영 RViz 설정 |
| `scripts/experiment/`, `config/experiment/` | 과거 실험 재현 코드·설정. 운영 기본값과 구분 |

- [운영 명령·구조](docs/workspace_layout.md) · [Livox·두 PC 설치](docs/experiment_04_setup.md) · [DDS 설정](docs/experiment_04_dds_network.md) · [주기 정합·재투영](docs/experiment_04_periodic_alignment.md)
