# 파라미터 참고 문서

이 문서는 현재 `co_3dto2d_mapping` 브랜치의 실제 노드 선언, launch 재정의,
`config/occupancy.yaml`, `config/rerun.yaml`을 기준으로 설명합니다. 거리 단위는 m,
시간은 이름에 표시된 s 또는 ms, `_deg`는 degree, `_rad`는 radian입니다.

## 적용 우선순위

```text
노드 선언 기본값 < YAML 파라미터 파일 < launch의 명시적 파라미터 딕셔너리
```

`single_bag_mapping.launch.py`는 `occupancy_config_file`을 `occupancy_mapper`와
`gravity_plane_pose_fusion.py` 양쪽에 전달하고, 실행마다 달라지는 입력 토픽, frame,
중간/최종 odometry 토픽만 launch에서 덮어씁니다. 상대 토픽은 노드 namespace 안에서
해석됩니다. robot 0에서 `toy/corrected_odometry`는
`/r0/toy/corrected_odometry`입니다.

## 현재 pose 흐름

```text
/rN/odom
  RTAB-Map LiDAR odometry
        │
        ▼
occupancy_mapper local-window planar ICP
  x, y, yaw 보정
        │
        ▼
/rN/toy/planar_odometry
        │
        ├── /rN/mapping/lidar
        └── /rN/mapping/imu_filtered
                  │
                  ▼
gravity_plane_pose_fusion.py
  x, y, yaw: planar odometry
  z, roll, pitch: IMU 중력 제약 지면 평면
                  │
                  ▼
/rN/toy/corrected_odometry
                  │
                  ▼
/toy_record/rN/odom 및 TF
```

평면 검출이 실패하거나 IMU/TF가 준비되지 않으면 최종 노드는 planar odometry를 그대로
전달합니다. 잘못된 plane을 강제로 pose에 넣지 않는 것이 기본 정책입니다.

---

# 1. `occupancy_mapper`

기본 파일은 [`config/occupancy.yaml`](../config/occupancy.yaml)입니다.

## 1.1 입력, frame, 동기화

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `scan_cloud_topic` | `/livox/lidar` | 매핑에 사용할 `PointCloud2`입니다. live 2대에서는 `/rN/mapping/lidar`로 덮어씁니다. |
| `odom_topic` | `odom` | RTAB-Map odometry 입력입니다. namespace 기준 상대 토픽입니다. |
| `local_frame_id` | `base_link` | cloud 변환, local map, plane pose의 로봇 기준 frame입니다. |
| `global_frame_id` | `odom` | global occupancy와 planar odometry의 기준 frame입니다. |
| `use_odom_header_frame` | `true` | 첫 odometry의 `header.frame_id`를 global frame으로 채택합니다. two-live에서는 보통 false입니다. |
| `sync_queue_size` | `100` | PointCloud/Odometry approximate-time 동기화 큐입니다. |
| `transform_cloud_to_local_frame` | `true` | cloud frame이 local frame과 다르면 TF로 점을 변환합니다. TF가 없으면 해당 frame을 건너뜁니다. |
| `alignment_required` | `false` | true이면 initial alignment를 받기 전 occupancy update를 중단합니다. |
| `alignment_topic` | `/toy/initial_xy_alignment` | initial `x/y/yaw` alignment 입력입니다. |

## 1.2 Z slice, 거리, 본체 제거

| 파라미터 | YAML 기본값 | 설명 |
| --- | ---: | --- |
| `z_min`, `z_max` | `0.4`, `0.8` | Z slice 범위입니다. `z_min <= z_max`여야 합니다. |
| `slice_in_global_frame` | `false` | true이면 global 변환 후 Z를 사용합니다. |
| `slice_z_in_cloud_frame` | `true` | global slice가 아닐 때 raw cloud Z를 쓸지 결정합니다. false이면 corrected local Z를 씁니다. |
| `invert_z_slice` | `true` | true이면 `[z_min,z_max]` 바깥 점을 유지합니다. 센서 축과 목표 장애물 높이에 맞게 RViz에서 확인해야 합니다. |
| `center_box_filter_half_extent_m` | `0.80` | local XY에서 `|x|,|y| <= 값`인 중앙 정사각형을 제거합니다. 0이면 끕니다. |
| `range_min_m`, `range_max_m` | `0.80`, `12.0` | occupied endpoint로 사용할 XY 범위입니다. max가 0이면 상한을 끕니다. |

## 1.3 Occupancy와 raycast

| 파라미터 | 기본값 | 설명 |
| --- | ---: | --- |
| `grid_resolution` | `0.05` | cell 크기입니다. 작을수록 정밀하지만 메모리와 처리량이 증가합니다. |
| `local_map_size_m` | `20.0` | local occupancy 한 변 크기입니다. |
| `enable_raycast_free_space` | `true` | 센서 원점부터 반환점까지 free-space ray를 누적합니다. |
| `raycast_min_range_m`, `raycast_max_range_m` | `0.80`, `12.0` | free-space ray 거리 범위입니다. max 0은 상한 비활성화입니다. |
| `raycast_clear_occupied` | `false` | true이면 ray가 기존 occupied cell을 즉시 free로 덮을 수 있습니다. temporal filter 사용 시에도 false가 보수적입니다. |
| `raycast_free_value` | `0` | free 출력값입니다. |
| `raycast_occupied_value` | `100` | occupied 출력값입니다. |
| `raycast_unknown_value` | `-1` | unknown 출력값입니다. 세 값은 `[-1,100]`에서 서로 달라야 합니다. |
| `occupied_threshold_points` | `1` | 한 frame에서 cell을 occupied 관측으로 인정할 최소 점 수입니다. |
| `global_map_padding_m` | `5.0` | global grid 확장 시 관측 주변 padding입니다. |
| `publish_period_ms` | `200` | local/global occupancy publish 주기입니다. |
| `log_z_slice_stats` | `true` | Z slice 통계를 로그로 출력합니다. |
| `publish_slice_debug_points` | `true` | 유지/제거 점 debug cloud를 publish합니다. |
| `slice_debug_points_max_points` | `80000` | 각 debug cloud 최대 점 수입니다. |

## 1.4 Dynamic occupancy filter

| 파라미터 | 기본값 | 설명 |
| --- | ---: | --- |
| `dynamic_filter_enabled` | `true` | 시간 누적 evidence 필터를 켭니다. |
| `dynamic_free_clear_count` | `4` | occupied cell을 지우기 전에 필요한 서로 다른 frame의 free 관측 횟수입니다. 작으면 빨리 지우지만 정적 장애물도 지울 수 있습니다. |
| `dynamic_occupied_confirm_count` | `3` | free/unknown cell을 occupied로 확정할 반복 관측 횟수입니다. |
| `dynamic_counter_decay` | `1` | 반대 evidence가 기존 카운터를 줄이는 양입니다. |
| `dynamic_evidence_timeout_frames` | `30` | 이 frame 수 동안 갱신되지 않은 evidence를 만료합니다. 0은 timeout 비활성화입니다. |

## 1.5 Local-window planar ICP

현재 scan의 XY 점을 최근 corrected scan들의 local submap에 맞춥니다. 이 단계가 보정하는
자유도는 `x`, `y`, `yaw`입니다. 품질 gate 실패 시 RTAB-Map prediction을 사용합니다.

| 파라미터 | 기본값 | 설명 |
| --- | ---: | --- |
| `enable_local_window_icp` | `false` | local-window ICP를 활성화합니다. 기본값에서는 raw odometry를 그대로 사용합니다. |
| `icp_window_size` | `10` | submap에 유지할 최근 scan 수입니다. |
| `icp_min_window_frames` | `3` | ICP 시작에 필요한 최소 window frame 수입니다. |
| `icp_voxel_size_m` | `0.12` | source/submap XY voxel입니다. 크게 하면 빠르지만 정밀도가 낮아집니다. |
| `icp_min_source_points` | `80` | ICP에 사용할 최소 scan 점 수입니다. |
| `icp_max_source_points` | `1200` | source 최대 점 수입니다. |
| `icp_max_submap_points` | `8000` | submap 최대 점 수입니다. |
| `icp_max_iterations` | `12` | fine ICP 반복 상한입니다. |
| `icp_max_correspondence_distance_m` | `0.45` | correspondence 최대 거리입니다. 너무 작으면 실패하고 너무 크면 다른 구조에 붙을 수 있습니다. |
| `icp_min_correspondences` | `50` | 성공에 필요한 최소 대응점 수입니다. |
| `icp_min_overlap_ratio` | `0.20` | source 중 대응점을 얻어야 하는 최소 비율입니다. |
| `icp_trim_ratio` | `0.75` | 가까운 correspondence 중 최적화에 유지할 비율입니다. 낮추면 outlier에 강하지만 정보가 줄어듭니다. |
| `icp_max_rmse_m` | `0.20` | 최종 RMSE 상한입니다. |
| `icp_max_rmse_increase_m` | `0.01` | 초기 prediction보다 RMSE가 나빠질 수 있는 최대량입니다. |
| `icp_correction_gain` | `0.70` | prediction에서 ICP 결과로 적용하는 보간 gain입니다. |
| `icp_max_correction_translation_m` | `0.35` | frame당 translation correction 상한입니다. |
| `icp_max_correction_yaw_deg` | `8.0` | frame당 yaw correction 상한입니다. |
| `icp_reset_translation_jump_m` | `1.50` | raw odometry가 이만큼 점프하면 ICP window를 reset합니다. |
| `icp_reset_yaw_jump_deg` | `35.0` | raw yaw가 이만큼 점프하면 window를 reset합니다. |
| `icp_coarse_translation_range_m` | `0.10` | predicted pose 주변 coarse translation 탐색 반경입니다. 0은 비활성화입니다. |
| `icp_coarse_translation_step_m` | `0.05` | coarse translation step입니다. |
| `icp_coarse_rotation_range_deg` | `3.0` | predicted yaw 주변 coarse 탐색 범위입니다. |
| `icp_coarse_rotation_step_deg` | `1.0` | coarse yaw step입니다. |
| `icp_log_period_ms` | `2000` | accept/reject 로그 throttle입니다. |
| `publish_corrected_odometry` | `true` | planar corrected odometry publish 여부입니다. |
| `corrected_odometry_topic` | `toy/planar_odometry` (YAML) | mapper의 중간 `x/y/yaw` 결과입니다. 노드 선언 기본값은 `toy/corrected_odometry`지만 현재 pipeline은 plane fusion을 위해 별도 중간 토픽을 사용합니다. |

---

# 2. `gravity_plane_pose_fusion.py`

상세 수식은 [`gravity_plane_pose_fusion.md`](gravity_plane_pose_fusion.md)에 있습니다.

## 2.1 입출력과 동기화

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `ground_plane_pose_enabled` | `true` | false이면 planar odometry를 그대로 최종 토픽으로 전달합니다. |
| `ground_plane_pointcloud_topic` | `/livox/lidar` | ground 후보 cloud입니다. launch가 robot별 mapping cloud로 덮어씁니다. |
| `ground_plane_imu_topic` | `/livox/imu_filtered` | 중력 prior를 얻을 filtered IMU입니다. |
| `ground_plane_planar_odometry_topic` | `toy/planar_odometry` | `x/y/yaw` 입력입니다. |
| `ground_plane_output_odometry_topic` | `toy/corrected_odometry` | 최종 6-DoF pose 출력입니다. |
| `ground_plane_local_frame_id` | `base_link` | plane normal/height를 표현할 로봇 frame입니다. odometry child frame과 일치시키는 것이 안전합니다. |
| `ground_plane_sync_queue_size` | `30` | cloud/planar odometry 동기화 큐입니다. |
| `ground_plane_sync_slop_sec` | `0.08` | 두 입력 stamp의 최대 허용 차이입니다. 정상 mapper는 동일 cloud stamp를 사용합니다. |

## 2.2 IMU 중력 방향

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `ground_plane_gravity_source` | `orientation_then_acceleration` | `orientation`, `acceleration`, `orientation_then_acceleration`, `blend` 중 하나입니다. 기본은 filtered orientation을 우선하고 불가능할 때 acceleration을 씁니다. |
| `ground_plane_imu_timeout_sec` | `0.30` | cloud와 cached IMU stamp의 최대 차이입니다. 0은 검사 비활성화입니다. |
| `ground_plane_acceleration_min_mps2` | `6.0` | acceleration을 gravity 후보로 인정할 최소 크기입니다. |
| `ground_plane_acceleration_max_mps2` | `13.0` | acceleration 후보 최대 크기입니다. 충격과 큰 선형 가속을 제외합니다. |
| `ground_plane_acceleration_blend` | `0.15` | `blend`에서 orientation up에서 acceleration up 쪽으로 섞는 비율입니다. |

방향 벡터는 IMU frame에서 `ground_plane_local_frame_id`로 TF 회전만 적용합니다.

## 2.3 후보점 선택

점은 IMU up 축과 이에 직교하는 tangent plane으로 분해됩니다. signed height는 up 방향
좌표이며 local origin보다 아래 바닥은 보통 음수입니다.

| 파라미터 | 기본값 | 설명 |
| --- | ---: | --- |
| `ground_plane_candidate_min_range_m` | `0.30` | tangent plane 최소 반경입니다. |
| `ground_plane_candidate_max_range_m` | `6.0` | tangent plane 최대 반경입니다. |
| `ground_plane_candidate_height_min_m` | `-2.5` | signed height 하한입니다. |
| `ground_plane_candidate_height_max_m` | `-0.05` | signed height 상한입니다. 0 근처 차체/센서 점을 제외합니다. |
| `ground_plane_candidate_center_exclusion_m` | `0.25` | 중심부 추가 제외 반경입니다. |
| `ground_plane_max_points` | `4000` | RANSAC 최대 후보점 수입니다. |

## 2.4 Gravity-constrained plane fit

plane은 `n·p+d=0`으로 표현하고 normal `n`의 부호를 IMU up 방향으로 맞춥니다. 양의
`d`는 local origin과 plane 사이 수직 거리, 즉 base 높이입니다.

| 파라미터 | 기본값 | 설명 |
| --- | ---: | --- |
| `ground_plane_ransac_iterations` | `120` | 3점 plane hypothesis 반복 횟수입니다. |
| `ground_plane_distance_threshold_m` | `0.04` | point-to-plane inlier 거리입니다. |
| `ground_plane_max_normal_deviation_deg` | `18.0` | plane normal과 IMU up 사이 최대 각도입니다. 수직 벽은 이 gate에서 제거됩니다. |
| `ground_plane_min_inliers` | `80` | 최소 inlier 수입니다. |
| `ground_plane_min_inlier_ratio` | `0.08` | 후보점 중 최소 inlier 비율입니다. |
| `ground_plane_min_height_m` | `0.05` | origin-plane 최소 거리입니다. |
| `ground_plane_max_height_m` | `2.5` | origin-plane 최대 거리입니다. |
| `ground_plane_lowest_score_weight` | `0.03` | inlier 수가 비슷하면 더 아래 plane을 선호하는 score입니다. 책상 상판 선택 방지에 사용합니다. |
| `ground_plane_random_seed` | `7` | 재현 가능한 RANSAC seed입니다. |

최종 inlier는 gravity tangent 좌표에서 least-squares로 다시 fit합니다.

## 2.5 시간 필터와 jump gate

| 파라미터 | 기본값 | 설명 |
| --- | ---: | --- |
| `ground_plane_filter_gain` | `0.25` | 새 normal/height를 기존 상태에 적용하는 gain입니다. 낮을수록 부드럽지만 지연이 커집니다. |
| `ground_plane_max_height_jump_m` | `0.20` | 한 번에 허용할 height 변화입니다. |
| `ground_plane_max_tilt_jump_deg` | `10.0` | 직전/신규 normal 사이 허용 각도입니다. |
| `ground_plane_hold_timeout_sec` | `0.75` | sparse cloud 또는 일시적인 TF/IMU 지연 시 마지막 유효 plane을 유지하는 시간입니다. |
| `ground_plane_state_reset_timeout_sec` | `3.0` | 이 시간 이상 신규 plane이 없으면 filter와 초기 height 기준을 reset합니다. |
| `ground_plane_log_period_ms` | `2000` | 품질 로그 throttle입니다. |

## 2.6 z와 covariance

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `ground_plane_z_mode` | `height_above_plane` | `height_above_plane`, `relative_to_initial`, `passthrough` 중 하나입니다. |
| `ground_plane_reference_z_m` | `0.0` | `height_above_plane`에서 ground의 global 기준 z입니다. |
| `ground_plane_z_offset_m` | `0.0` | 계산 z에 더하는 calibration offset입니다. |
| `ground_plane_z_stddev_min_m` | `0.02` | fresh plane의 z covariance 최소 표준편차입니다. RMSE가 더 크면 RMSE를 사용합니다. |
| `ground_plane_orientation_stddev_rad` | `0.03` | plane-derived roll/pitch covariance 표준편차입니다. |

```text
height_above_plane:
  z = reference_z + measured_height + offset

relative_to_initial:
  z = initial_pose_z + (measured_height - initial_height) + offset

passthrough:
  z = input_planar_pose_z + offset
```

평평한 단일층에서 바닥을 `z=0`으로 두려면 `height_above_plane`이 직접적입니다. 계단,
경사로, 여러 층은 지면 자체의 global elevation 추정이 별도로 필요하므로
`relative_to_initial` 또는 외부 고도 estimator를 고려합니다.

---

# 3. Initial two-robot XY alignment

설치된 `initial_xy_icp_alignment.py`는 constrained wrapper이며 occupancy를 XY 점으로
바꾸고 `x/y/yaw`만 정합합니다. `z/roll/pitch`는 대상이 아닙니다.

| 파라미터 | 기본값 | 설명 |
| --- | ---: | --- |
| `input_mode` | `cloud_initial` | `cloud_initial` 또는 `global_occupancy`입니다. two-live는 global occupancy를 사용합니다. |
| `robot0_cloud_topic`, `robot1_cloud_topic` | `/r0/livox/lidar`, `/r1/livox/lidar` | cloud mode 입력입니다. |
| `robot0_map_topic`, `robot1_map_topic` | `/r0/toy/global_occupancy`, `/r1/toy/global_occupancy` | map mode 입력입니다. |
| `alignment_topic` | `/toy/initial_xy_alignment` | 결과 transform 토픽입니다. |
| `target_frame_id`, `source_frame_id` | `odom`, `r1/odom` | 결과 부모/자식 frame입니다. two-live 부모는 `map`입니다. |
| `frame_count` | `5` | cloud initial submap frame 수입니다. |
| `voxel_size` | `0.10` | 2D voxel입니다. two-live 기본은 `0.05`입니다. |
| `max_points` | `30000` | 입력별 최대 점 수입니다. |
| `max_correspondence_distance` | `0.75` | correspondence 상한입니다. |
| `min_correspondences` | `100` | 최소 대응점 수입니다. |
| `min_fitness`, `max_rmse` | `0.05`, `0.40` | 결과 수용 기준입니다. |
| `max_iterations` | `80` | ICP 반복 상한입니다. |
| `recompute_period_sec` | `5.0` | map mode 재시도 주기입니다. two-live 기본은 `2.0`입니다. |
| `occupied_threshold` | `50` | occupancy를 ICP 점으로 바꾸는 임계값입니다. |
| `startup_delay_sec` | `0.0` | 양쪽 입력이 보인 뒤 정합 전 대기입니다. two-live 기본은 `3.0`입니다. |
| `retry_on_failure` | `true` | 실패 후 다음 입력으로 재시도합니다. |
| `lock_after_first_alignment` | `false` | 최초 승인 transform 고정 여부입니다. two-live는 true입니다. |
| `required_consistent_results` | `1` | publish 전 연속 일치 후보 수입니다. two-live는 2입니다. |
| `max_consistency_translation_m` | `0.25` | 연속 후보 translation 허용 차이입니다. |
| `max_consistency_rotation_rad` | `0.0873` | 연속 후보 yaw 허용 차이, 약 5도입니다. |
| `initialize_from_centroids` | `false` | identity 외 centroid translation 초기값을 시험합니다. two-live는 true입니다. |
| `invert_result` | `false` | 역변환을 publish할지 결정합니다. |

## 180도 반대 방향 방지 환경 변수

| 환경 변수 | 기본값 | 설명 |
| --- | ---: | --- |
| `CO3DTO2D_ENFORCE_HEADING_PRIOR` | `true` | 물리적으로 가능한 상대 heading을 강제합니다. |
| `CO3DTO2D_EXPECTED_RELATIVE_YAW_DEG` | `0` | 기대 r1/r0 상대 yaw입니다. |
| `CO3DTO2D_MAX_RELATIVE_YAW_DEVIATION_DEG` | `90` | 기대 yaw 허용 편차입니다. 180도 반대 해를 차단합니다. |
| `CO3DTO2D_INITIAL_YAW_OFFSETS_DEG` | `0,-30,30` | 기대 yaw 주변 초기 후보입니다. |
| `CO3DTO2D_MAX_ICP_ROTATION_STEP_DEG` | `60` | iteration당 최대 회전입니다. |
| `CO3DTO2D_HEADING_PRIOR_WEIGHT` | `0.05` | 품질이 비슷하면 기대 heading에 가까운 후보를 선호하는 가중치입니다. |

---

# 4. RTAB-Map `icp_odometry`

`launch/rtabmap_mid360_odometry.launch.py`의 wrapper 인자는 다음과 같습니다.

| 인자 | 기본값 | 설명 |
| --- | ---: | --- |
| `frame_id` | `base_link` | odometry child frame입니다. |
| `odom_topic` | `odom` | odometry 출력입니다. |
| `scan_cloud_topic` | `/livox/lidar` | PointCloud 입력입니다. |
| `imu_topic` | `/livox/imu` | IMU 입력입니다. 실제 pipeline은 filtered IMU를 전달합니다. |
| `wait_imu_to_init` | `true` | 첫 scan 전 IMU 초기화를 기다립니다. |
| `expected_update_rate` | `10.0` | 입력률 watchdog입니다. 느린 bag replay는 0으로 끕니다. |
| `startup_delay_sec` | `0.0` | odometry node 시작 지연입니다. two-live 기본은 10초입니다. |
| `qos`, `qos_imu` | `0`, `0` | RTAB-Map wrapper QoS 값입니다. |
| `publish_tf` | `true` | odometry TF publish 여부입니다. two-live는 frame 충돌 방지를 위해 false입니다. |

현재 launch의 주요 고정 RTAB-Map 값은 다음과 같습니다.

```text
Icp/PointToPlane=true
Icp/Iterations=10
Icp/VoxelSize=0.1
Icp/Epsilon=0.001
Icp/PointToPlaneK=20
Icp/MaxTranslation=2
Icp/MaxCorrespondenceDistance=1
Icp/Strategy=1
Icp/OutlierRatio=0.7
Icp/CorrespondenceRatio=0.01
Odom/ScanKeyFrameThr=0.4
OdomF2M/ScanSubtractRadius=0.1
OdomF2M/ScanMaxSize=15000
OdomF2M/BundleAdjustment=false
```

이 값들은 현재 `occupancy.yaml`이 아니라 launch 딕셔너리에 있습니다.

---

# 5. 센서 전처리

## Madgwick IMU filter

| 파라미터 | 현재값 | 설명 |
| --- | ---: | --- |
| `use_mag` | `false` | magnetometer를 사용하지 않습니다. |
| `publish_tf` | `false` | filter 자체 TF를 끕니다. |
| `reverse_tf` | `false` | orientation 방향을 뒤집지 않습니다. |
| `world_frame` | `enu` | world 축 convention입니다. |
| `remove_gravity_vector` | `false` | acceleration에 gravity를 유지합니다. acceleration fallback에 필요합니다. |
| `imu_input_is_filtered` | `false` | true이면 bag의 기존 filtered IMU를 Madgwick에 다시 넣지 않습니다. |

## Rear-sector filter

| 파라미터 | 기본값 | 설명 |
| --- | ---: | --- |
| `enabled` / `enable_rear_lidar_filter` | node `true`, two-live `false` | 후방 섹터 제거입니다. |
| `rear_filter_angle_deg` | `120.0` | 제거 섹터 전체 폭입니다. |
| `rear_axis` / `rear_filter_axis` | `-x` | 중심축으로 `x`, `-x`, `y`, `-y`를 사용합니다. |
| `min_xy_range_m` | `0.0` | 이 거리 이하 점은 섹터 안이어도 유지합니다. |
| `log_period` | `100` | 통계 로그 주기입니다. 0은 로그 비활성화입니다. |
| `output_frame_id` | empty | 설정하면 header frame만 교체하며 좌표는 회전하지 않습니다. |

---

# 6. Record republisher와 merged map

설치되는 `record_republisher.py`는 temporal merged-map cleanup을 유지하면서 최종
`/rN/toy/corrected_odometry`를 우선합니다.

| 파라미터 | 기본값 | 설명 |
| --- | ---: | --- |
| `target_frame_id`, `common_frame_id` | `odom`, `map` | robot odom suffix와 공통 frame입니다. |
| `alignment_topic` | `/toy/initial_xy_alignment` | map/r1 odom planar alignment입니다. |
| `publish_period_ms` | `200` | 재게시 및 TF 주기입니다. |
| `occupied_threshold` | `50` | merged occupied 임계값입니다. |
| `merged_padding_m` | `1.0` | merged grid padding입니다. |
| `robot_ids` | `[0,1]` | 처리 robot 목록입니다. |
| `output_prefix` | `/toy_record` | 출력 루트입니다. |
| `robot_odom_frame_format` | `r{robot_id}/odom` | robot odom frame 형식입니다. |
| `robot_base_frame_format` | `r{robot_id}/base_link` | base frame 형식입니다. |
| `publish_tf` | `true` | 공통 TF를 publish합니다. |
| `publish_merged_global` | node `false`, two-live `true` | merged occupancy publish입니다. |
| `prefer_ground_fused_odometry` | `true` | 최종 corrected odometry를 pose/TF 원본으로 우선합니다. |
| `ground_fused_odometry_topic_format` | `/r{robot_id}/toy/corrected_odometry` | robot별 최종 pose 토픽입니다. |

최종 pose가 아직 없는 robot은 `/rN/odom`을 startup fallback으로 사용합니다. 최종 pose가
한 번 들어오면 raw odometry가 다시 덮어쓰지 않습니다.

## Temporal merged occupancy

| 파라미터 | 기본값 | 설명 |
| --- | ---: | --- |
| `merged_temporal_filter_enabled` | `true` | merged temporal evidence를 활성화합니다. |
| `merged_dynamic_free_clear_count` | `4` | merged occupied cell clear에 필요한 free 관측 수입니다. |
| `merged_dynamic_occupied_confirm_count` | `3` | merged occupied 확정 관측 수입니다. |
| `merged_dynamic_counter_decay` | `1` | 반대 evidence 카운터 감소량입니다. |
| `merged_dynamic_evidence_timeout_frames` | `30` | 오래된 evidence 만료 frame 수입니다. |
| `merged_free_observation_inflation_m` | `0.05` | local free 관측 확장 거리입니다. |
| `merged_alignment_reset_translation_m` | `0.05` | alignment translation 변화에 따른 state reset 기준입니다. |
| `merged_alignment_reset_yaw_deg` | `0.5` | alignment yaw 변화 reset 기준입니다. |

---

# 7. 실행 및 bag 인자

| 인자 | 기본값 | 설명 |
| --- | ---: | --- |
| `use_bag` | single `true`, live `false` | internal bag player 사용 여부입니다. |
| `use_sim_time` | bag `true`, live `false` | `/clock` 사용 여부입니다. |
| `bag_path` | empty | rosbag2 디렉터리입니다. |
| `rate` | `1.0` | playback rate입니다. |
| `storage_id` | `sqlite3` | storage plugin입니다. |
| `mapping_startup_delay_sec` | single `0.0`, two-live `10.0` | driver/IMU filter는 실행하되 odometry, mapper, plane fusion 시작을 지연합니다. |
| `planar_odometry_topic` | `toy/planar_odometry` | pipeline 내부 planar pose입니다. |
| `corrected_odometry_topic` | `toy/corrected_odometry` | 최종 6-DoF pose입니다. |
| `occupancy_config_file` | package `config/occupancy.yaml` | occupancy, local ICP, ground-plane 파라미터 파일입니다. |

두 로봇은 같은 occupancy config를 사용하는 것이 권장됩니다. ground-plane 후보 높이와
sensor static TF는 실제 장착 높이와 축 방향에 맞춰야 합니다.

---

# 8. 증상별 조정

## Ground plane을 못 찾음

1. `/rN/mapping/lidar`와 `local_frame_id` 사이 TF를 확인합니다.
2. IMU orientation/acceleration covariance 첫 값이 `-1`인지 확인합니다.
3. 실제 바닥이 `ground_plane_candidate_height_min_m/max_m` 안에 있는지 확인합니다.
4. `ground_plane_min_inliers`와 `ground_plane_min_inlier_ratio`를 완화합니다.
5. `ground_plane_distance_threshold_m`를 조금 늘립니다.

## 책상이나 의자 면을 선택함

1. `ground_plane_candidate_height_max_m`를 더 음수로 내립니다.
2. 실제 base 높이를 알면 `ground_plane_min_height_m`를 올립니다.
3. `ground_plane_lowest_score_weight`를 올립니다.
4. `ground_plane_min_inlier_ratio`를 강화합니다.

## roll/pitch가 흔들림

1. `ground_plane_filter_gain`을 낮춥니다.
2. `ground_plane_max_normal_deviation_deg`를 줄입니다.
3. `ground_plane_distance_threshold_m`를 줄입니다.
4. 후보 최대 거리를 줄여 먼 point noise를 제외합니다.

## z 의미가 맞지 않음

- base의 바닥 위 절대 높이: `height_above_plane`
- 초기 odom z를 유지하고 높이 변화만 사용: `relative_to_initial`
- z는 유지하고 plane roll/pitch만 사용: `passthrough`
- 고정 calibration 차이: `ground_plane_z_offset_m`

## Local-window ICP가 자주 reject됨

1. correspondence 거리와 coarse search 범위를 조금 늘립니다.
2. 최소 correspondence/overlap을 완화합니다.
3. voxel 크기를 데이터 밀도에 맞춥니다.
4. 큰 오보정을 막는 correction translation/yaw gate는 마지막에 완화합니다.

## Initial ICP가 180도 반대로 붙음

1. `CO3DTO2D_EXPECTED_RELATIVE_YAW_DEG`를 실제 배치에 맞춥니다.
2. `CO3DTO2D_MAX_RELATIVE_YAW_DEVIATION_DEG`를 물리 범위로 줄입니다.
3. 불가능한 initial yaw offset을 제거합니다.
4. heading prior를 끄지 않고 map의 비대칭 구조를 더 축적합니다.
