# 지면 평면 기준 1 m mapping height filter

기본 mapping 경로는 더 이상 LiDAR/센서 frame의 고정 `z_min`, `z_max`,
`invert_z_slice` 조합으로 점을 선택하지 않습니다.

```text
raw PointCloud2 + filtered IMU
        │
        ▼
gravity-constrained ground plane
        │
        ▼
signed height h = n · p + d
        │
        ├─ h < 0.05 m   : 바닥/바닥 노이즈 제거
        ├─ 0.05–1.00 m  : mapping과 local-window ICP에 사용
        └─ h > 1.00 m   : 높은 구조물 제거
```

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
ground_plane_filtered_cloud_topic: "mapping/plane_height_filtered"
ground_plane_filter_min_height_m: 0.05
ground_plane_filter_max_height_m: 1.00
ground_plane_filtered_cloud_min_points: 20
ground_plane_filtered_cloud_log_period_ms: 2000
```

- `ground_plane_filter_min_height_m`: 바닥 자체를 occupancy endpoint로 넣지 않기 위한 여유입니다.
- `ground_plane_filter_max_height_m`: 지면으로부터 사용할 최대 높이입니다. 기본값이 요청한 1 m입니다.
- `ground_plane_filtered_cloud_min_points`: 너무 희소한 frame을 mapper에 넘기지 않는 최소 점 수입니다.

평면 검출이 한 frame에서 실패하면 `ground_plane_hold_timeout_sec` 동안 마지막 유효
평면을 사용합니다. 그 시간도 지나면 raw cloud로 되돌아가지 않고 filtered cloud
발행을 중단합니다. 따라서 잘못된 고정-Z fallback으로 지도가 오염되지 않습니다.

## 확인

```bash
ros2 topic hz /r0/mapping/plane_height_filtered
ros2 topic echo /r0/toy/planar_odometry --once
ros2 topic echo /r0/toy/corrected_odometry --once
```

로그:

```text
Plane-height cloud accepted: ... band=[0.05, 1.00]m ...
```

필터 점이 너무 적으면 범위를 넓히기 전에 sensor static TF와 지면 평면 inlier 수를
먼저 확인해야 합니다. 실내 바닥 노이즈가 많이 들어오면 최소 높이를 0.08–0.12 m로
올리고, 책상 아래 구조까지 필요하면 최대 높이를 1.2 m 정도로 늘릴 수 있습니다.
