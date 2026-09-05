# 한 bag에 기록된 두 로봇 live mapping 재실행

`run_two_live_combined_bag.sh`는 `/r0`와 `/r1` 센서가 함께 기록된 하나의 rosbag을
한 노트북에서 다시 실행하기 위한 스크립트입니다. 하나의 bag player를 사용하므로
두 로봇 사이의 기록 시각 관계가 유지됩니다.
재생 입력은 public `two_live_mapping.launch.py`로 전달되므로, live mode와 같은
startup ICP gate와 plane-height mapping을 사용합니다. 두 로봇은 시작 시 같은 장소에
있다고 가정하므로 후속 occupancy place recognition은 기본적으로 실행하지 않습니다.

기존 실행 결과와 새 실행 결과가 섞이지 않도록 다음 네 입력만 선택해서 재생합니다.

- r0 LiDAR
- r0 IMU
- r1 LiDAR
- r1 IMU

기록된 `/r0/odom`, `/r1/odom`, occupancy, `/toy/initial_xy_alignment`, `/tf`,
`/tf_static`은 재생하지 않습니다.

## 실행

MID-360 기본 데이터셋은 별도 인자 없이 단축 실행할 수 있습니다. 기본 bag은
`/mnt/ssd1/aibot/airoom_0901/260901_r2/airoom_chair_r2`, 기본 domain은 `173`,
기본 재생 속도는 `0.5`입니다.

```bash
bash scripts/run_two_mid360_2d_mapping_bag.sh --rviz
```

`--bag`, `--domain-id`, `--rate`로 이 기본값을 바꿀 수 있고, `--dry-run`,
`--imu-source`, `--launch-arg`, `--workspace` 등은 combined runner로 그대로
전달됩니다. build 뒤에는 `ros2 run co_3dto2d_mapping run_two_mid360_2d_mapping_bag.sh --dry-run`로도 확인할 수 있습니다.

`--enable-place-recognition`을 지정한 경우에만 후속 occupancy place recognition을
실행합니다. MID-360 단축 경로는 이때
`config/airoom_chair_replay_place_recognition.yaml`을 사용합니다. 이 bag은 r1이
실질적으로 정지해 있어, 검증된 첫 place pair 하나로 lock하도록
`min_known_ratio=0.09`, consensus=1/1을 사용합니다.

```bash
bash scripts/run_two_live_combined_bag.sh \
  --bag /mnt/ssd1/aibot/airoom_0901/260901_r2/airoom_chair_r2 \
  --domain-id 173 \
  --rate 0.5 \
  --rviz
```

시작 위치가 다르거나 주행 중 재정렬이 필요한 경우에만 다음처럼 명시적으로 켭니다.

```bash
bash scripts/run_two_live_combined_bag.sh \
  --bag /mnt/ssd1/aibot/airoom_0901/260901_r2/airoom_chair_r2 \
  --domain-id 173 \
  --rate 0.5 \
  --enable-place-recognition
```

먼저 실제 ROS 프로세스를 시작하지 않고 선택 결과를 확인할 수 있습니다.

```bash
bash scripts/run_two_live_combined_bag.sh \
  --bag /mnt/ssd1/aibot/airoom_0901/260901_r2/airoom_chair_r2 \
  --domain-id 173 \
  --rate 0.5 \
  --dry-run
```

기본 `--imu-source auto`는 robot별 raw/filtered IMU 중 메시지가 더 많은 스트림을
선택합니다. filtered IMU를 선택한 경우 Madgwick filter를 다시 적용하지 않고 frame만
정규화해 odometry로 전달합니다. raw IMU를 강제로 시험하려면
`--imu-source raw`를 사용합니다.

## 재생 속도와 초기화 시간

`--sensor-warmup`, `--alignment-warmup`, `--alignment-period`는 bag의 기록 시간
기준이며 스크립트가 재생 속도에 맞춰 launch의 wall timer로 변환합니다. 기본 public
live mode는 startup ICP가 승인될 때까지 odometry/mapping을 gate하므로,
`mapping_startup_delay_sec`는 이 gate가 활성화된 기본 실행에서는 0으로 대체됩니다.
ICP의 기록 시간 1초 대기와 2초 재시도 주기는 기본 `--rate 0.5`에서 각각 wall time
2초와 4초로 변환됩니다. 기존 timer 기반 시작을 비교할 때만
`--launch-arg wait_for_initial_alignment:=false`를 추가하세요.

## 확인

```bash
ROS_DOMAIN_ID=173 ros2 topic hz /r0/odom
ROS_DOMAIN_ID=173 ros2 topic hz /r1/odom
ROS_DOMAIN_ID=173 ros2 topic echo /toy/initial_xy_alignment --once
```

실행 로그는 `/tmp/co3dto2d-two-live.*` 아래에 저장됩니다.
