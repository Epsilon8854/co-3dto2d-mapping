# 실험 4 — 센서망과 PC 간 DDS 통신 분리

## 확인한 문제와 범위

- 2026-10-23 r0에서 rqt가 `/r1/livox/lidar`를 Reliable로 구독하고 있었음
  - 중앙 매핑·녹화기는 이 시점에 실행 중이 아님
  - 해당 r0 rqt를 종료해 원격 raw 구독을 제거함
- 기존 Fast DDS 2.6.12 프로세스는 Wi-Fi와 센서 Ethernet 모두에 DDS 소켓을 생성
  - 커널 송수신 최대 버퍼는 각각 212992 bytes
- r0의 10초 UDP 카운터 측정에서 `SndbufErrors/RcvbufErrors` 증가는 없었음
  - **r1 송신 포화나 센서 UDP 수신 중단을 r0 결과로 확정하지 않음**
- 센서 UDP 수신, SDK 처리, ROS 발행, 원격 DDS 전달은 서로 다른 단계
  - `ros2 topic hz=0`만으로 센서 패킷이 PC에 도착하지 않는다고 단정하지 않음

## 적용 정책

- `config/dds.yaml`을 공통 정책으로 사용
- `scripts/lib/dds.py`가 ROS 초기화 전에 현재 PC의 IPv4 기본 경로 NIC를 선택함
  - 2026-10-23 r0에서 선택된 Wi-Fi `wlp128s20f3/192.168.50.143`
  - IP를 Git config에 하드코딩하지 않음
- Fast DDS UDP whitelist에는 선택한 IP만 등록
  - `useBuiltinTransports=false`로 무제한 기본 UDP transport의 추가 생성을 막고, **명시적 SHM transport는 유지**함
  - Livox SDK의 센서 Ethernet 설정은 바꾸지 않음
- `RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS`를 기본으로 설정해 애플리케이션 발행과 네트워크 송신을 분리
  - 기존 명시적 publication mode는 유지
  - 비동기화만으로 모든 backpressure나 손실을 없애지는 못함
- UDP `non_blocking_send=false`를 유지
  - 포화 시 패킷을 조용히 버리는 모드로 문제를 숨기지 않음
  - ROS Reliable/Best Effort 정책도 변경하지 않음
- UDP 버퍼 요청은 4MiB를 목표로 하되 현재 OS `wmem_max/rmem_max` 중 작은 값으로 제한함
  - 적용값·OS 한도·NIC·IP·publication mode·XML hash를 출력하고 저장
  - **현재 OS 한도가 작으면 실질적인 증설은 아님**
  - 커널 설정을 자동 변경하거나 sudo를 요청하지 않음
- 사용자 지정 RMW/XML과 충돌하면 기존 설정을 덮어쓰지 않고 명확한 오류를 출력
  - 별도 관리 프로파일은 `EXPERIMENT_04_DDS_MODE=external`로 선택함

## 기존 실행 명령 유지

- 운영 실행기 `scripts/run_multi_agent.sh`, 도구 `scripts/tools/replay_bag.sh`·`record_bag.sh`·`monitor_pipeline.sh`에서 ROS 초기화 전 정책 적용
  - runtime이 시작한 driver·mapper·fusion·RViz는 같은 환경을 상속함
- 수정은 저장소의 Python 실행기이므로 이번 DDS 변경만 적용하는 데 colcon 재빌드는 불필요
  - 다른 node 변경을 함께 받았다면 해당 변경의 빌드 요구를 준수
- **양쪽 PC의 ROS 프로세스를 재시작해야 적용됨**
  - 실행 중인 driver, rqt, ROS daemon은 설정 파일을 바꿔도 자동으로 바뀌지 않음
- 기존 standalone driver가 있으면 driver까지 정리하고 `--start-driver`로 한 번만 시작함
  - 자동으로 다른 실행기의 driver를 종료하지 않음

```bash
cd ~/co-3dto2d-mapping
git pull --ff-only
# 이후 각 PC에서 기존 실험4 실행 명령 사용.
```

- VPN·복수 NIC·기본 경로와 다른 LAN을 쓰면 아래와 같이 그 PC의 NIC 또는 IP를 지정

```bash
export EXPERIMENT_04_DDS_INTERFACE=wlp128s20f3
# r1은 r1 자신의 NIC/IP를 지정. r0의 이름/IP를 복사하지 않는다.
```

## CLI·진단 도구

```bash
# NIC/IP·생성 XML·요청 버퍼 확인. ROS 노드를 시작하지 않는다.
python3 scripts/lib/dds.py --describe

# 같은 DDS 설정으로 CLI 또는 GUI 실행.
bash scripts/tools/with_dds.sh ros2 topic list --no-daemon
bash scripts/tools/with_dds.sh rqt
```

- rqt에서 상대 agent의 raw cloud를 체크하면 원격 전송이 다시 시작됨
  - whitelist는 토픽 접근 제어가 아님
  - 평소에는 pose·진단·병합 지도만 구독
- 기본 녹화는 raw LiDAR/IMU를 구독하지 않음
  - 다만 두 agent의 mapper 입력 filtered cloud를 기록하므로 녹화 중에는 그에 따른 원격 대역폭이 추가됨
- 기존 ROS daemon이 다른 설정으로 떠 있으면 CLI discovery 관찰과 실노드 상태가 달라질 수 있음
  - `--no-daemon`을 사용하거나 daemon만 별도로 재시작함
  - 매핑 노드 재시작과 혼동하지 않음

## 저장과 재확인

- run/capture의 `configs/dds_profile.xml`과 metadata의 `dds_transport`에 실제 적용 설정을 저장
  - monitor JSON에도 같은 metadata를 등록
- 재시작 후 `ss -uapn`에서 **DDS 포트**가 PC 간 NIC에만 바인드됐는지 확인
  - 센서 SDK의 56101/56201/56301/56401 및 SDK 전용 포트는 Ethernet에 계속 존재하는 것이 정상
- r1에서 센서 UDP 56301/56401 수신량과 드라이버 발행을 함께 확인
  - r0의 remote topic 수신만으로 r1 ingress를 판정하지 않음
- 원격 raw 구독을 끈 상태에서 local cloud/IMU, ICP, mapper, 전역 pose가 회복되는지 확인
  - 그 뒤 녹화를 추가해 부하 영향을 따로 확인
- 필요 시 관리자 측에서 OS 버퍼 한도를 검토 가능
  - 한도를 올린 뒤에는 실행기를 재시작해야 DDS 소켓의 요청값도 다시 계산됨
  - 버퍼 증설만으로 지속적인 처리율 부족을 해결할 수는 없음

## 공식 근거

- [Fast DDS 2.6 UDP transport](https://fast-dds.docs.eprosima.com/en/v2.6.11/fastdds/transport/udp/udp.html): 송신 버퍼·blocking/non-blocking 동작
- [Interface whitelist](https://fast-dds.docs.eprosima.com/en/v2.6.11/fastdds/transport/whitelist.html): 통신 인터페이스 제한
- [ROS rmw_fastrtps Humble](https://github.com/ros2/rmw_fastrtps/tree/humble): publication mode 환경 설정
