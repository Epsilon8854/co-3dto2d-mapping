# 실험4 — 실행 중 초기 정합 모니터

## 실행

- r0의 `run_experiment_04_r0.sh` 또는 기존 `--role all`/`--role fusion` 실행 시 브라우저 모니터가 자동으로 열린다. r1 robot-only 실행에서는 열지 않는다. replay의 중앙 실행에도 적용한다.
- 실행기가 모니터 프로세스를 소유하며 Ctrl+C 정상 종료 시 함께 정리한다. 모니터 창을 닫거나 서버가 종료되어도 driver·mapper를 종료시키지 않는다.
- GUI가 필요 없으면 `--no-alignment-viewer`를 추가한다. `--no-rviz`와 독립된 옵션이다.
- Python 실행기와 웹 페이지 변경이므로 이 변경만 적용할 때 colcon 재빌드는 필요 없다. 실행 중인 기존 실행기는 자동 실행 기능을 소급 적용하지 않는다.

```bash
# r0: driver + mapping + fusion + RViz + 정합 모니터
bash scripts/run_experiment_04_r0.sh

# 실행 중인 중앙에 모니터만 붙이기. 실제 fusion 출력 cycle 경로를 지정한다.
bash scripts/view_experiment_04_startup_alignment.sh \
  --run-directory outputs/experiment_04_multi_agent_mapping/runs/<실행명>/cycle_001
```

- 모니터 URL은 해당 cycle의 `alignment_viewer.json`과 `alignment_viewer.log`에 기록한다. 자동 브라우저 열기가 지원되지 않는 환경에서는 해당 URL을 수동으로 연다.
- HTTP 서버는 **127.0.0.1에만** 바인드한다. 다른 PC에 지도를 공개하지 않는다. 같은 PC의 웹 브라우저에서 확인한다.
- 수동으로 붙인 모니터는 해당 터미널의 Ctrl+C로 종료한다. `--owner-pid <실행기 PID>`를 함께 주면 해당 실행기 종료도 감시한다.

## 표시 내용과 좌표

- 왼쪽: r0 누적 global occupancy에서 실제 검색에 사용한 target 패치.
- 가운데: r1 누적 global occupancy에서 실제 검색에 사용한 source 패치.
- 오른쪽: r0 패치와 r1 경계를 변환 전/선택 후보 적용 후로 비교한다. 각 입력 패치는 그 agent의 초기 anchor 좌표이고, overlay는 r0 패치 좌표다. 후보의 patch transform은 최종 mapping_odom 간 transform과 구분한다.
- unknown/free/occupied를 구분하고 r0/r1 경계를 서로 다른 색으로 표시한다. 붉은 점은 occupied 경계가 상대 지도에서 free로 관측된 곳이다.
- 붉은 점은 충돌 지표의 **경계→free 부분을 전체 경계로 표시**한다. 실제 승인 비율은 sampling과 내부 free→occupied 비교도 포함하므로 붉은 점 수로 전체 충돌률을 역산하지 않는다.
- 후보별 overlap·RMSE·free conflict·대응점·observability와 실제 설정의 허용 기준을 함께 표시한다. 전체 검색의 승인/거절 사유도 표시한다.
- 최신 검색 자동 추적, 이전/다음 검색, 후보 선택, 변환 전/후, free/충돌 표시, 연결된 확대·이동을 지원한다. 화면 조작은 정합 transform을 변경하거나 후보를 강제 승인하지 않는다.

## 갱신과 비용

- 실제 `alignment_cases/*/{result.json,r0_patch.npz,r1_patch.npz}`를 읽으므로 **수초짜리 검색이 완료되고 저장된 뒤** 갱신된다. 검색 solver의 매 iteration마다 화면이 바뀌는 기능은 아니다.
- 첫 검색 전에는 live 상태와 입력 대기를 표시한다. 승인 후에는 선택된 검색 결과를 유지하고 live 상태에 정합 완료를 표시한다.
- 작은 `/multi_agent/alignment_status`만 구독한다. raw cloud·IMU·local/global map 토픽은 추가 구독하지 않는다. 지도 자료는 r0 디스크에서 읽고 HTTP도 localhost 안에서 전달한다.
- 직전 4개 검색 payload만 메모리에 cache한다. 큰 cloud·grid serialization을 매핑 콜백에 추가하지 않는다. 기존 정합 저장 worker를 그대로 사용한다.
- 현재 실행이 종료됐으면 상태 수신 없음/지연을 표시한다. 저장된 마지막 검색을 보여주는 것을 현재 정합 진행으로 해석하지 않는다.
- 기존 성공/거절 기준·정합 알고리즘·지도 통신 정책은 변경하지 않는다.
