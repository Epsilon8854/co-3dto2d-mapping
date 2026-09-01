# 한 bag에 기록된 두 로봇 live mapping 재실행

`run_two_live_combined_bag.sh`는 `/r0`와 `/r1` 센서가 함께 기록된 하나의 rosbag을
한 노트북에서 다시 실행하기 위한 스크립트입니다. 하나의 bag player를 사용하므로
두 로봇 사이의 기록 시각 관계가 유지됩니다.

기존 실행 결과와 새 실행 결과가 섞이지 않도록 다음 네 입력만 선택해서 재생합니다.

- r0 LiDAR
- r0 IMU
- r1 LiDAR
- r1 IMU

기록된 `/r0/odom`, `/r1/odom`, occupancy, `/toy/initial_xy_alignment`, `/tf`,
`/tf_static`은 재생하지 않습니다.

## 실행

```bash
bash scripts/run_two_live_combined_bag.sh \
  --bag /mnt/ssd1/aibot/airoom_0901/260901_r2/airoom_chair_r2 \
  --domain-id 173 \
  --rate 0.5 \
  --rviz
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
기준입니다. 스크립트가 재생 속도에 맞춰 launch의 wall timer로 변환합니다. 기본값
`--rate 0.5 --startup-delay 5 --sensor-warmup 10`에서는 bag을 시작하고 기록 시간
10초가 지난 시점에 odometry가 시작되도록 `mapping_startup_delay_sec=25`가
적용됩니다. ICP의 기록 시간 3초 대기와 2초 재시도 주기는 각각 wall time 6초와
4초로 변환됩니다.

## 확인

```bash
ROS_DOMAIN_ID=173 ros2 topic hz /r0/odom
ROS_DOMAIN_ID=173 ros2 topic hz /r1/odom
ROS_DOMAIN_ID=173 ros2 topic echo /toy/initial_xy_alignment --once
```

실행 로그는 `/tmp/co3dto2d-two-live.*` 아래에 저장됩니다.
