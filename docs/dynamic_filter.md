# 동적 장애물 흔적 제거: config 스위치 하나

## 사용

양쪽 로봇 PC에서 실제로 사용하는 occupancy YAML의 다음 값만 바꾸고,
양쪽 mapping과 fusion 프로세스를 재시작합니다. 기본값은 기존과 같이 false입니다.

```yaml
/**:
  ros__parameters:
    dynamic_filter_enabled: true  # ON; false는 OFF
```

이것은 기존 파일에서 수정할 부분이지, 전체 occupancy.yaml을 대체하는 파일이 아닙니다.
두 PC는 파일 내용이 자동 동기화되지 않으므로 동일하게 설정해야 합니다.
`--mapping-config /absolute/path/to/occupancy.yaml` 또는
`occupancy_config_file:=/absolute/path/to/occupancy.yaml`로 선택한 파일이 적용됩니다.

ON이면 각 로봇의 global map과 병합 map 모두 시간 누적 필터를 사용합니다.
OFF이면 각 로봇은 기존 누적 방식을, 병합 노드는 기존 occupied-priority union을 사용합니다.
별도 merged 활성화 스위치는 필요하지 않습니다. 옛 `merged_temporal_filter_enabled`와
`merged_dynamic_*` 값이 YAML에 남아 있어도 공통 `dynamic_*` 설정이 우선합니다.
단, record republisher를 `ros2 run`으로 직접 실행하면 launch의 설정 변환이 없으므로
기존 `merged_*` ROS 파라미터를 명시해야 합니다.

지원 실행 경로는 `two_live_mapping.launch.py`(설치된 공개 wrapper 포함),
`two_live_combined_bag_mapping.launch.py`, `two_bag_mapping.launch.py`입니다.
단일 로봇 mapper는 기존처럼 같은 YAML을 직접 읽습니다.

## 삭제 기준

기존 occupancy.yaml의 숫자 기본값은 그대로 유지했습니다.

```yaml
    enable_raycast_free_space: true
    raycast_clear_occupied: false
    dynamic_free_clear_count: 10
    dynamic_occupied_confirm_count: 1
    dynamic_counter_decay: 1
    dynamic_evidence_timeout_frames: 30
    merged_free_observation_inflation_m: 0.0
```

사람이나 물체가 이동한 자리를 **실제로 free로 재관측**해야 흔적을 지웁니다.
기본적으로 해당 셀의 free 증거가 10회 쌓이면 삭제하며, 중간에 실제 occupied
관측이 들어오면 확정된 장애물 셀의 free 증거는 초기화됩니다. 10회는 10초가 아닙니다.
병합 단계는 각 로봇에서 수신한 새 local map의 관측을 세므로 로봇별 mapper와
벽시계 기준 삭제 시점이 같지는 않습니다. 동일 stamp의 연속 재발행은 새 증거로 세지 않습니다.

`dynamic_occupied_confirm_count: 1`이므로 현재 관측한 장애물은 일단 기록합니다.
이는 사람 검출, tracking, point-cloud segmentation이나 현재 보이는 사람을
처음부터 완전히 제외하는 기능이 아닙니다. 가려졌거나 시야 밖인 셀은 자동으로
지우지 않습니다. timeout은 관측 카운터의 유효 기간이며 map 셀 자체의 TTL이 아닙니다.

병합의 free-space dilation은 기본 0m입니다. 실제 관측하지 않은 이웃 셀을 free로
확장해 얇은 벽을 지우는 것을 피합니다. 필요한 실험에서는
`merged_free_observation_inflation_m`을 명시적으로 조정할 수 있습니다.

## 함께 수정한 사항

- launch에서 공통 config의 on/off와 네 가지 관측 카운터를 병합 노드에 전달합니다.
- 설정 파일 누락, 문자열 boolean, 잘못된 카운터, ON + raycast OFF 조합을 명확한 오류로 처리합니다.
- 다른 로봇의 global map이 늦게 초기화에 사용되어도 이미 live 관측한 셀을 덮어쓰지 않습니다.
- fusion core에서 unknown(-1)을 free 증거로 해석하지 않습니다.
- 로봇 간 ICP는 이전 변경의 2D map 전용 경로를 유지합니다.

## 적용 및 확인

코드를 갱신한 뒤 양쪽 PC에서 한 번 rebuild/source합니다. YAML은 그 다음 수정합니다.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select co_3dto2d_mapping
source install/local_setup.bash
```

설치된 YAML 사본인지 소스 YAML인지 혼동되지 않게 사용자 config의 절대 경로를
`--mapping-config`로 지정하는 방식을 권장합니다. 소스 YAML을 수정했지만 설치본이
별도 복사본이라면 다시 빌드하거나 실제 실행에 지정한 파일을 수정해야 합니다.

실행 후 ON 상태는 다음과 같이 확인합니다.

```bash
ros2 param get /r0/occupancy_mapper dynamic_filter_enabled
ros2 param get /r1/occupancy_mapper dynamic_filter_enabled
ros2 param get /toy_record_republisher merged_temporal_filter_enabled
ros2 param get /toy_record_republisher merged_dynamic_free_clear_count
```

각 스위치는 true, 기본 clear count는 10이어야 합니다. merger의 ROS 파라미터 이름은
기존 호환을 위해 merged 이름을 유지하지만, 값의 출처는 공통 dynamic 설정입니다.
시작 로그의 `dynamic_filter=true` 및 `Temporal merged occupancy enabled=true`도 확인합니다.

YAML 실시간 감시나 `ros2 param set`에 의한 hot toggle은 지원하지 않습니다.
설정을 변경하면 프로세스를 재시작해야 합니다. 실행 중 값만 바꾸는 것은 내부 필터
상태까지 안전하게 전환하는 방법이 아닙니다.

## 검증 범위

```bash
python -m pytest -q test/test_dynamic_filter_config.py \
  test/test_two_live_startup_gating.py test/test_temporal_fusion.py
```

설정 변환, 실제 launch 함수의 전달 인자, 실제 temporal callback 및 fusion core를
테스트합니다. ROS parameter/clock/message 서비스와 launch action은 테스트 대역입니다.
ROS 2/DDS 통신, RTAB-Map, 실물 로봇 및 rosbag 주행 평가는 별도의 통합 검증이 필요합니다.
