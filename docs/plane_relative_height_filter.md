# 지면 또는 고정 장착높이 기준 mapping height filter

기본 mapping 경로는 더 이상 LiDAR/센서 frame의 고정 `z_min`, `z_max`,
`invert_z_slice` 조합으로 점을 선택하지 않습니다.

```text
raw PointCloud2 + filtered IMU
        │
        ▼
observed ground plane 또는 fixed sensor height
        │
        ▼
signed height h = n · p + d
        │
        ├─ h < 0.05 m   : 바닥/바닥 노이즈 제거
        ├─ configured range : mapping에 사용
        └─ 그 밖의 높이     : 제거
```

바닥이 충분히 관측되는 환경에서는 관측 평면을 사용할 수 있습니다. 바닥점이 보이지
않는 고정 장착 환경에서는 IMU up 방향과 실측 센서 높이로 가상 바닥을 정의합니다.
Experiment 1의 deployment profile은 센서 높이 `0.30 m`, 바닥 기준
`0.30–1.20 m`를 사용합니다. 이는 gravity-aligned 센서 좌표 기준 `0.00–0.90 m`
필터와 같습니다.

`n · p + d = 0`에서 `n`은 IMU가 제공한 up 방향을 향하는 단위 법선이고,
`d`는 `base_link` 원점에서 지면까지의 수직 거리입니다. 따라서 로봇이 기울거나
센서 장착 roll/pitch가 있어도 높이 판정은 검출된 지면에 대한 수직 거리로 이루어집니다.

## 기본 토픽

로봇별 raw mapping cloud:

```text
/r0/mapping/lidar
/r1/mapping/lidar
```

평면 높이 필터 결과:

```text
/r0/mapping/plane_height_filtered
/r1/mapping/plane_height_filtered
```

`occupancy_mapper`는 두 번째 토픽을 입력으로 사용합니다. 고정 Z 필터는 호환성을 위해
파라미터 선언만 남아 있으며, 기본 launch가 `z=[-1000, 1000]`,
`invert_z_slice=false`로 강제해 사실상 통과시킵니다.

## 주요 파라미터

```yaml
ground_plane_height_filter_enabled: true
ground_plane_height_reference_mode: "fixed_sensor_height"
lidar_mount_height_above_floor_m: 0.30
ground_plane_filtered_cloud_topic: "mapping/plane_height_filtered"
ground_plane_filter_min_height_m: 0.30
ground_plane_filter_max_height_m: 1.20
ground_plane_filtered_cloud_min_points: 20
ground_plane_filtered_cloud_log_period_ms: 2000
```

- `ground_plane_height_reference_mode`: `estimated_plane`은 cloud에서 바닥을 찾고, `fixed_sensor_height`는 바닥을 찾지 않습니다.
- `lidar_mount_height_above_floor_m`: LiDAR 원점에서 물리적 바닥까지의 실측 수직 거리입니다.
- `ground_plane_filter_min_height_m`, `ground_plane_filter_max_height_m`: 가상 바닥에서 잰 통과 높이입니다.
- `ground_plane_filtered_cloud_min_points`: 너무 희소한 frame을 mapper에 넘기지 않는 최소 점 수입니다.

`fixed_sensor_height`에서는 평면 inlier를 찾지 않으며, IMU up을 얻지 못한 scan만
필터 출력을 중단합니다. `estimated_plane`에서는 평면 검출 실패 시
`ground_plane_hold_timeout_sec` 동안 마지막 유효 평면을 사용합니다.

## 확인

```bash
ros2 topic hz /r0/mapping/plane_height_filtered
ros2 topic echo /r0/mapping/floor_based_odometry --once
ros2 topic echo /r0/mapping/occupancy_mapper_odometry --once
```

로그:

```text
Gravity-height cloud fixed_sensor_height: ... floor_band=[0.30, 1.20]m sensor_height=0.30m
```

이 profile은 높이 0.30 m 아래 장애물을 의도적으로 버립니다. 해당 높이의 장애물이
충돌 위험이라면 이 필터를 단일 안전 센서로 사용하면 안 됩니다.
