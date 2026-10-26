# 최종 지도·pose 녹화·재생

## 녹화

```bash
bash scripts/tools/record_bag.sh --name mapping
```

- **실행 PC:** 중앙 r0 · 매핑과 별도 터미널
- **권장 시점:** 매핑 시작 전 녹화 → 정합 완료 → 이동 → 녹화 종료
- **Single:** `--pipeline single` 추가 · r1 센서 사용 시 `--robot-id 1`
- **종료:** Ctrl+C · 매핑·driver는 계속 실행
- **설정:** `config/recording.yaml`

| 구성 | 저장 토픽 |
|---|---|
| Multi 지도 | `/multi_agent/merged_global_occupancy` |
| Multi pose | `/multi_agent/rN/global_pose`, `/multi_agent/rN/aligned_odometry` |
| Multi 경로 | `/multi_agent/rN/aligned_path` |
| Single 지도 | `/rN/mapping/global_occupancy` |
| Single pose | `/rN/mapping/occupancy_mapper_odometry` |
| Single 경로 | `/rN/mapping/debug/local_submap_corrected_odometry_path` |
| 공통 | `/tf`, `/tf_static` |
| Multi 지도 상태 | `/multi_agent/map_status` |

- **통신:** 지도·pose·경로·TF만 구독 · raw cloud/IMU·중간 cloud 미구독
- **보관:** 녹화 당시 결과 · 알고리즘 재계산용 센서 입력은 미포함

## 저장 구조

```text
outputs/multi_agent/captures/mapping_YYYYMMDD_HHMMSS/
├── bag/                 # metadata.yaml · *.db3
├── capture.yaml         # 실행 환경·토픽·Git revision
├── capture_audit.yaml   # 최종 지도·pose 저장 여부
├── runtime_parameters.yaml
├── configs/             # 녹화 설정·저장소 설정 사본
└── logs/
```

- **Single:** `outputs/single_agent/captures/`에 동일 구조로 저장
- **확인 항목:** `results_replay_minimum_complete: true`
  - 최종 지도·pose의 메시지 존재·타입 확인 · 전 구간 무손실 보장은 아님

## 재생

```bash
bash scripts/tools/replay_bag.sh --bag /absolute/capture/bag --loop
```

- **입력:** `bag/` 또는 그 상위 capture 폴더
- **동작:** single/multi 자동 판별 → RViz 실행 → 저장된 지도·pose·경로 재생
- **좌표계:** Multi = `map` · Single = 해당 `rN/odom`
- **실행 범위:** bag player·RViz · driver/ICP/mapper/정합 노드 미실행
- **통신:** 로컬 전용 domain 73 · 실센서 연결 불필요
- **옵션:** `--rate 0.5` = 절반 속도 · `--no-rviz` = player만 실행
- **반복:** 회차마다 RViz·player 재시작 · TF·시각화 상태 초기화
- **한 번 재생:** `--loop` 생략 · 마지막 화면 유지
- **종료:** Ctrl+C
- **재생 로그:** `outputs/{single,multi}_agent/replays/실행시각/`
- **이전 bag:** 최종 지도·pose가 저장된 경우 사용 가능 · 미기록 경로는 표시하지 않음
