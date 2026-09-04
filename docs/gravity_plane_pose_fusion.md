# IMU 중력 제약 지면 평면 기반 6-DoF pose 조합

현재 mapping의 평면 보정 결과는 다음 두 단계로 나뉩니다.

1. `occupancy_mapper`의 local-window ICP가 `x`, `y`, `yaw`를 계산하여
   `/rN/toy/planar_odometry`로 publish합니다.
2. `gravity_plane_pose_fusion.py`가 같은 시각의 PointCloud2와 filtered IMU를 사용해
   지면 평면을 검출하고, 그 평면에서 얻은 `z`, `roll`, `pitch`를 결합하여
   `/rN/toy/corrected_odometry`로 publish합니다.

최종 pose의 자유도 출처는 다음과 같습니다.

| 자유도 | 출처 |
| --- | --- |
| `x`, `y` | 2D local-window ICP / planar mapping |
| `yaw` | 2D local-window ICP / planar mapping |
| `z` | 검출된 지면 평면까지의 거리 |
| `roll`, `pitch` | 검출된 지면 평면의 법선 |

`record_republisher.py` 실행 파일은 최종 `/rN/toy/corrected_odometry`가 한 번이라도
들어오면 이를 `/toy_record/rN/odom`과 TF의 pose 원본으로 사용합니다. 최종 pose가
아직 없을 때만 `/rN/odom`을 startup fallback으로 사용합니다.

## 좌표와 수식

filtered IMU의 orientation은 REP-145에 따라 센서 frame의 world frame에 대한 자세로
해석합니다. 따라서 world의 `+Z`를 orientation의 역회전으로 IMU frame에 표현하고,
static TF를 이용해 `local_frame_id`로 변환하여 `up_local`을 얻습니다.

가속도 fallback을 사용할 때는 정지 상태의 accelerometer가 body `+Z` 방향으로
`+g` specific force를 출력한다는 REP-145 규약을 사용합니다. 측정 크기가
`ground_plane_acceleration_min_mps2`와 `ground_plane_acceleration_max_mps2` 사이일
때만 up 방향 후보로 사용합니다.

평면은 local frame에서 다음 식으로 표현합니다.

```text
n · p + d = 0
```

- `n`: IMU up 방향을 향하도록 부호가 정규화된 단위 법선
- `d`: local frame 원점에서 평면까지의 양의 수직 거리

RANSAC 후보는 `n`과 IMU up 사이 각도가
`ground_plane_max_normal_deviation_deg` 이내인 경우에만 평가합니다. 최종 inlier는
중력 방향에 직교하는 두 tangent 축과 up 축으로 좌표를 바꾼 뒤 least-squares로
다시 평면을 맞춥니다.

ZYX Euler convention에서 body frame에 표현된 world-up 벡터를
`n=(nx, ny, nz)`라고 하면 다음과 같이 roll과 pitch를 얻습니다.

```text
roll  = atan2(ny, nz)
pitch = atan2(-nx, sqrt(ny² + nz²))
```

이 계산은 yaw와 독립적이므로 yaw는 planar odometry 값을 그대로 유지합니다.

## 기본 z 의미

기본값은 다음과 같습니다.

```yaml
ground_plane_z_mode: "height_above_plane"
ground_plane_reference_z_m: 0.0
ground_plane_z_offset_m: 0.0
```

따라서 최종 z는 다음과 같습니다.

```text
z = 기준 지면 z + local frame 원점의 지면 위 높이 + offset
```

평평한 실내 바닥을 global `z=0`으로 둘 때 권장되는 모드입니다. `base_link` 원점이
바닥이 아니라 LiDAR 또는 차체 중심에 있다면 그 실제 높이가 pose의 z로 출력됩니다.

기존 odometry의 초기 z를 0으로 유지하면서 높이 변화만 반영하려면 다음을 사용합니다.

```yaml
ground_plane_z_mode: "relative_to_initial"
```

평면에서 roll/pitch만 사용하고 z는 입력 odometry 값을 유지하려면 다음을 사용합니다.

```yaml
ground_plane_z_mode: "passthrough"
```

## 후보점 선택

점군은 먼저 `local_frame_id`로 변환합니다. 그 후 IMU up 방향으로 투영한 signed height와
중력 직교 평면상의 반경으로 후보를 고릅니다.

```yaml
ground_plane_candidate_min_range_m: 0.30
ground_plane_candidate_max_range_m: 6.0
ground_plane_candidate_height_min_m: -2.5
ground_plane_candidate_height_max_m: -0.05
```

local frame 원점보다 바닥이 아래에 있으므로 일반적인 ground point의 signed height는
음수입니다. 바닥이 후보 범위에 들어오지 않으면 이 범위를 실제 센서 장착 높이에 맞게
조정해야 합니다.

책상 상판이 바닥보다 더 자주 선택된다면 다음 순서로 조정합니다.

1. `ground_plane_candidate_height_max_m`를 더 낮은 값으로 내립니다.
2. `ground_plane_min_height_m`를 실제 base 높이 근처로 올립니다.
3. `ground_plane_lowest_score_weight`를 조금 올립니다.
4. `ground_plane_candidate_center_exclusion_m`를 키워 로봇 자체를 제외합니다.

## 품질 및 시간 필터

다음 조건을 모두 만족해야 새 평면을 사용합니다.

- 최소 inlier 개수
- 최소 inlier 비율
- 최대 point-to-plane 거리
- IMU up과 법선 사이 최대 각도
- 허용 가능한 base 높이 범위

새 결과가 직전 결과에서 갑자기 크게 변하면 폐기합니다.

```yaml
ground_plane_max_height_jump_m: 0.20
ground_plane_max_tilt_jump_deg: 10.0
ground_plane_filter_gain: 0.25
```

일시적인 sparse cloud나 TF 지연은 `ground_plane_hold_timeout_sec` 동안 마지막 유효
평면을 유지합니다. 그보다 오래 평면이 없으면 입력 planar odometry의 z/roll/pitch를
그대로 내보냅니다. 따라서 plane fit 실패가 잘못된 6-DoF 보정으로 이어지지 않습니다.

## 주요 파라미터

| 파라미터 | 기본값 | 설명 |
| --- | ---: | --- |
| `ground_plane_pose_enabled` | `true` | 평면 기반 z/roll/pitch 조합을 활성화합니다. `false`면 planar odometry를 그대로 전달합니다. |
| `ground_plane_gravity_source` | `orientation_then_acceleration` | IMU up 추출 방법입니다. |
| `ground_plane_imu_timeout_sec` | `0.30` | cloud와 IMU timestamp의 최대 차이입니다. |
| `ground_plane_max_normal_deviation_deg` | `18.0` | 평면 법선이 IMU up에서 벗어날 수 있는 최대 각도입니다. |
| `ground_plane_distance_threshold_m` | `0.04` | RANSAC inlier의 point-to-plane 거리입니다. |
| `ground_plane_min_inliers` | `80` | 유효 평면에 필요한 최소 점 수입니다. |
| `ground_plane_min_inlier_ratio` | `0.08` | 후보점 중 최소 inlier 비율입니다. |
| `ground_plane_min_height_m` | `0.05` | local frame 원점과 바닥 사이 최소 거리입니다. |
| `ground_plane_max_height_m` | `2.5` | local frame 원점과 바닥 사이 최대 거리입니다. |
| `ground_plane_ransac_iterations` | `120` | RANSAC 반복 횟수입니다. |
| `ground_plane_max_points` | `4000` | plane fit에 사용할 최대 후보점 수입니다. |
| `ground_plane_z_mode` | `height_above_plane` | z를 절대 높이, 초기값 대비 변화, 또는 passthrough 중 하나로 계산합니다. |

전체 기본값은 `config/occupancy.yaml`에 있습니다.

## 확인

```bash
ros2 topic echo /r0/toy/planar_odometry --once
ros2 topic echo /r0/toy/corrected_odometry --once
ros2 topic echo /r1/toy/planar_odometry --once
ros2 topic echo /r1/toy/corrected_odometry --once
```

로그에서 다음 항목을 확인합니다.

```text
Ground-plane pose initialized/accepted
candidates
inliers
ratio
rmse
normal_delta
z, roll, pitch
```

평면을 찾지 못하면 `fit_failed`, IMU/TF가 준비되지 않으면 planar fallback 로그가
출력됩니다.

## 한계

`height_above_plane` 모드는 검출된 local ground plane을 global `z=reference_z`로
간주합니다. 여러 층, 계단, 경사로처럼 지면 자체의 global elevation이 변하는 환경에서
절대 z를 연속적으로 복원하려면 별도의 고도 상태 추정 또는 3D map constraint가
필요합니다. 그런 환경에서는 우선 `relative_to_initial`을 사용하고, 장기 z drift는
별도 estimator에서 다루는 것이 안전합니다.
