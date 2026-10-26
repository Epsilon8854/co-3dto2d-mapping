# 실험 4 — Livox driver 중복 실행 방지

## 확인한 원인

- 2026-10-22 r0에 실제 driver binary가 2개 있었음
  - 이전 PID 13863은 실행 부모가 종료되어 user systemd 아래에 남았고 현재 실행기의 PID 14285와 센서/UDP endpoint를 공유
  - `ros2 run` wrapper를 driver로 중복 집계한 것이 아님
- 두 driver가 떠 있을 때 8초 동안 LiDAR/IMU 입력이 없었음
  - 잔류 PID는 SIGINT와 SIGTERM에 응답하지 않아 해당 PID만 SIGKILL로 정리
  - 이후 6초 동안 LiDAR 59개, IMU 1199개가 수신됐고 실제 driver/발행자는 1개
- 기존 `stop_process()`는 `ros2 run` leader의 `wait()`가 끝나면 자식 driver가 살아 있어도 추가 SIGTERM까지만 전달
  - leader가 먼저 종료되면 자식에 대한 SIGKILL 경로를 빠져나갈 수 있었음
- 기존 시작 검사는 ROS topic publisher만 조회함
  - driver가 센서 handshake 이전에 멈춰 publisher를 아직 만들지 않은 경우 중복을 놓침
- 최초 부모 종료가 Ctrl+C, terminal 종료, crash 중 무엇이었는지는 당시 자료로 확정하지 않음

## 수정

1. **프로세스 검사:** 실행 전 `/proc/*/exe`의 실제 Livox binary를 확인
    - 명시 config가 있으면 센서/host UDP endpoint가 겹치는 driver를 확인
    - config가 생략된 runtime 사전 검사에서는 기존 로컬 driver가 있으면 보수적으로 시작을 거절함
2. **원자적 실행 잠금:** 공용 `scripts/lib/run_livox_driver.sh`는 config의 sensor IP와 host UDP endpoint별 `flock`을 획득함
    - domain ID·robot namespace·workspace를 바꿔도 같은 장치의 잠금은 동일
    - 서로 다른 장치/endpoint는 별개
3. **직접 실행:** guard가 actual driver binary로 `exec`함
    - 실행기에서 관리하는 child PID가 실제 driver PID가 되며 중간 `ros2 run` parent는 만들지 않음
    - ROS/Livox 환경 source와 remap/parameter는 유지
4. **종료 확인:** leader뿐 아니라 소유 process group 전체가 사라졌는지 확인
    - SIGINT 대기 후에도 남으면 SIGTERM, 이후 SIGKILL까지 진행함
    - 이름 전체 `pkill`은 사용하지 않음
5. **부모 crash 대응:** Linux `PR_SET_PDEATHSIG`로 실행 부모가 사라지면 새 driver도 SIGKILL됨
    - terminal 강제 종료나 실행기 crash 시 orphan이 남는 경로를 축소
6. **예외 정리:** 상태 표시기 teardown에서 예외가 발생하더라도 실행기 소유 프로세스 cleanup을 수행

## 사용과 적용

- 기존 명령을 사용
  - 새 옵션은 불필요
- 중복이면 새 driver를 자동으로 추가하거나 기존 driver를 임의 종료하지 않고 PID/config가 있는 오류를 출력
  - 의도적으로 별도 driver를 유지한다면 매핑 실행에 `--no-start-driver`를 명시
- driver 시작 로그에 `Livox driver guarded: pid=...`가 표시됨
- 잠금 파일은 `~/.cache/co_3dto2d_mapping/driver_locks/`에 유지
  - 파일 존재가 실행 중을 뜻하지 않으며 커널 lock은 프로세스 종료 시 해제됨
  - 실행 중인 잠금 파일을 삭제하면 안 됨
- `/proc` 조회와 lock은 현재 Linux 사용자 기준
  - guard를 통하지 않는 임의의 수동 binary 실행, 다른 OS 사용자/host에서의 시작까지 원자적으로 막는 기능은 아님
- Linux의 종료 불가능한 D-state나 OS/network 자체의 문제를 해결하는 기능은 아님
- 현재 이미 실행 중인 구버전 프로세스에는 소급 적용되지 않음
  - 변경 작업 중 정상 매핑·driver를 재시작하거나 종료하지 않음
- r0/r1 각각 최신 코드를 받고 설치본도 사용한다면 빌드/source 후 **다음 실행부터** 적용됨
  - `scripts/lib/driver_guard.py`는 표준 Python 모듈만 사용
  - driver 실행기는 저장소의 해당 파일을 직접 사용

```bash
git pull --ff-only
source /opt/ros/humble/setup.bash
colcon build --packages-select co_3dto2d_mapping --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/local_setup.bash
# 이후 기존 실행 명령 사용
```

## 확인 범위

- 원인 진단 중 실제 프로세스·UDP socket·ROS 수신을 확인했고 잔류 driver 정리 후 수신 복구를 관측함
- 재발 방지 코드는 구문/빌드 확인만 수행
  - 현재 센서에 새 driver를 실행하거나 Ctrl+C/crash 재현 테스트를 하지 않음
