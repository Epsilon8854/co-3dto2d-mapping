# 2026-10-23 워크스페이스 정리

## 변경 범위

- 운영 실시간 entrypoint를 `scripts/run_single_agent.sh`, `scripts/run_multi_agent.sh`로 통합했습니다. robot/중앙 역할은 인자로 선택합니다.
- Python 패키지를 `common/`, `single_agent/`, `multi_agent/`로 분리했습니다. 실험 실행·분석은 `scripts/experiment/`로 이동했습니다.
- 운영 config는 single 공통 설정, multi override/central 설정, sensor별 extrinsic/rear/driver 설정으로 분리했습니다. r0/r1 전체 robot YAML 중복을 제거했습니다.
- 사용하지 않는 inherited two-live/rerun/record-republisher 및 관련 launch·설정·데이터 목록을 제거했습니다. 실제 bag·결과는 보존했습니다.
- 과거에 비교했던 floor fitting·대안 ICP/pose graph 구현은 실험 재현용 source로 분리하고 기본 colcon 빌드에서 제외했습니다.
- 운영 floor estimator와 폐기된 보정 토글을 제거했습니다. 현재 사용하는 고정 z+IMU gravity, rear/height filtering, raw ICP, PHT history5/full_se2 계산은 유지했습니다.
- ROS topic/TF 계약, DDS 통신 최적화, driver 중복 방지·소유 프로세스 종료, 중앙 temporal clearing 정책은 유지했습니다.
- `ros2 launch` 종료는 launch leader에 먼저 SIGINT를 보내 자식에게 중복 SIGINT가 전달되는 것을 줄였습니다. 잔류 process group은 기존 TERM/KILL 단계로 정리합니다.
- 사용자 RViz 수정은 `rviz/multi_agent.rviz`에 그대로 보존했습니다. **README는 변경하지 않았습니다.**

## 오프라인 확인

- 새 build/install 경로 및 운영 기본 경로에서 Release 빌드 통과.
- CTest 10개 target 통과. 현재 핵심 Python 26개, 이동한 실험 Python 45개 통과.
- 기록된 cloud/IMU·TF 150프레임에 동일 입력을 적용했을 때 고정 높이·중력 보정 odometry와 filtered cloud의 ROS 필드가 이전 구현과 동일했습니다. odometry 입력은 통제한 값이며 전체 ICP 성능 평가가 아닙니다.
- 기록된 filtered cloud+floor odometry 300쌍을 이전/새 C++ mapper에 순차 입력: 지도 셀·geometry·frame 동일. pose 성분 최대 차이 `1.11e-16`.
- 실험4 기존 capture의 중앙 입력 972건 재처리: 최종 병합/agent 지도, free streak·관측 시각·bin evidence가 기존 기준 결과와 동일. 별도 domain의 ROS 1x 재생도 최종 지도와 정합 변환이 동일했습니다.
- raw hallway bag의 8초 발췌본으로 새 single 실행기·지도 저장·두 번째 반복 회차의 지도 생성을 확인했습니다. 정상 종료와 잔류 프로세스 정리를 확인했으며 센서 driver는 시작하지 않았습니다.
- 이 확인은 코드 정리로 인한 회귀 점검이며, 새 두 PC 실증이나 odometry 정확도 개선을 의미하지 않습니다.

## 사용

- 현재 명령·설정·기록 구조: [workspace_layout.md](workspace_layout.md).
- 다른 PC에서도 pull 후 다시 build해야 합니다. 오래된 install 실행 파일이 남지 않도록 최초 정리 버전 적용 시 기존 build/install/log를 별도 보관한 후 빌드하십시오. `third_party/` Livox workspace는 별개입니다.

## 2026-10-26 중심 제외 영역 정정

- 참고 브랜치 `fix/direct-occupancy-icp-no-gates`의 YAML은 half extent 2.5m였지만 실제 live/single bag launch 기본값 0.8m가 이를 덮어썼습니다.
- 10/23 정리에서 single 설정에 YAML의 2.5m가 남아 기존 launch의 1.6×1.6m보다 큰 5×5m 제외 영역이 적용됐습니다. 당시 mapper 비교는 multi-agent 설정으로 수행되어 이 single 실행 설정 차이를 검출하지 못했습니다.
- 운영 설정을 기존 크기로 복구하고 이름을 `robot_body_exclusion_box_size_m`로 변경했습니다. 이제 전체 한 변 길이를 입력하며 single/multi 기본 1.6m입니다.
- 과거 실험 구현·config·결과 snapshot은 이전 파라미터 의미를 유지합니다. 해당 이름 변경은 운영 mapper와 공용 설정에 적용됩니다.
