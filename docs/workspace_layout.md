# 워크스페이스 구조

| 경로 | 용도 |
|---|---|
| `co_3dto2d_mapping/common/` | 좌표 변환·격자·저장 |
| `co_3dto2d_mapping/single_agent/` | 높이·gravity·후방 필터 |
| `co_3dto2d_mapping/multi_agent/` | 정합·병합·pose/TF 중계 |
| `include/`, `src/` | Occupancy·PHT C++ mapper |
| `launch/` | 공통 pipeline 구성 |
| `scripts/run_single_agent.sh` | Single 실행기 |
| `scripts/run_multi_agent.sh` | Multi 실행기 |
| `scripts/lib/` | 프로세스 수명·driver·DDS 내부 처리 |
| `scripts/tools/` | 설치·결과 녹화/재생·진단 |
| `config/`, `rviz/` | 운영 설정·화면 |
| `scripts/experiment/`, `config/experiment/`, `docs/experiment/` | 과거 실험 자료 |
| `outputs/` | Bag·실행 결과·지도·로그 · Git 제외 |

## 운영 설정

| 파일 | 내용 |
|---|---|
| `config/single_agent.yaml` | 높이·gravity·occupancy·PHT·동적 필터 |
| `config/multi_agent.yaml` | `agent_overrides`: agent 설정 차이 · `central`: 정합·병합 |
| `config/sensors/rN.yaml` | 센서 TF·rear filter·Livox 경로 |
| `config/lidar_odometry.yaml` | Raw ICP·deskew |
| `config/dds.yaml` | PC 간 DDS 통신 |
| `config/recording.yaml` | 최종 지도·pose·경로·TF 녹화 |

- **실행·토픽·파라미터:** [README](../README.md)
- **설치:** [환경 설정](experiment_04_setup.md)
- **최종 결과 녹화·재생:** [사용 안내](experiment_04_r0_recording.md)
