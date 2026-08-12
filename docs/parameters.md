# 파라미터 참고 문서

이 문서는 현재 패키지의 파라미터를 **핵심 파라미터**와 **그 외 파라미터**로 나누어 설명합니다. 실제 mapping 결과에 먼저 영향을 주는 값은 핵심 파라미터에서 확인하고, 나머지 값은 노드별 목록과 기본값을 참고하세요. 최종 동작은 노드 선언부, `config/occupancy.yaml`, `config/rerun.yaml`, launch 파일의 실제 설정을 기준으로 합니다.

## 우선순위와 공통 규칙

`occupancy_mapper`의 최종 값은 다음 순서로 결정됩니다: **노드 선언 기본값 → YAML 파일 → launch 재정의**. `single_bag_mapping`은 `occupancy_config_file`을 읽은 뒤 명시적인 파라미터 딕셔너리를 적용하므로, launch 딕셔너리의 값이 YAML보다 우선합니다. 다른 Python 노드는 launch 딕셔너리나 YAML에서 값을 제공하지 않는 한 선언 기본값을 사용합니다.

거리의 단위는 m, 주기의 단위는 이름에 표시된 ms 또는 s, `_deg` 접미사의 단위는 도, 각도 수렴 조건의 단위는 rad입니다. `/`로 시작하는 토픽은 절대 토픽이며, 매퍼의 상대 출력 토픽은 노드 네임스페이스 안에서 해석됩니다. 점유 격자 값은 **미관측 `-1`**, **빈 공간 `0`**, **점유 `100`** 규약을 따릅니다.

## 핵심 파라미터

### 1. 입력 토픽과 frame

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `scan_cloud_topic` | `/livox/lidar` | `sensor_msgs/msg/PointCloud2` 입력 토픽입니다. |
| `imu_raw_topic`, `imu_filtered_topic` | `/livox/imu`, `/livox/imu_filtered` | IMU raw 입력과 filtering 결과 토픽입니다. |
| `odom_topic` | `odom` | Odometry 입력 토픽입니다. |
| `local_frame_id` | `base_link` | 클라우드 변환과 로컬 mapping에 사용할 frame입니다. |
| `global_frame_id` | `odom` | 전역 격자 frame입니다. |
| `sensor_parent_frame`, `sensor_child_frame` | `base_link`, `livox_frame` | 센서 정적 TF의 부모/자식 frame입니다. |
| `sensor_tf_x/y/z` | `0`, `0`, `0` | `sensor_parent_frame -> sensor_child_frame`의 translation입니다. 실제 측정값으로 바꾸세요. |
| `sensor_tf_yaw/pitch/roll` | `0`, `0`, `3.141592653589793` | 센서 정적 TF의 회전값입니다. 실제 측정 extrinsic을 사용하세요. |
| `publish_sensor_static_tf` | `true` | 센서 정적 TF를 publish할지 결정합니다. |

### 2. Occupancy map 생성

| 파라미터 | 기본값 | 설명 및 검증 |
| --- | --- | --- |
| `grid_resolution` | `0.05` | 격자 셀 한 변의 길이입니다. 양수여야 합니다. |
| `local_map_size_m` | `20.0` | 로컬 격자의 한 변 크기이며 `grid_resolution`보다 커야 합니다. |
| `z_min`, `z_max` | `0.4`, `0.8` | Z 슬라이스 범위이며 `z_min <= z_max`여야 합니다. |
| `invert_z_slice` | `true` (YAML) | 선택한 Z 범위 안쪽이 아닌 바깥쪽의 점을 유지합니다. 노드 선언 기본값은 `false`입니다. |
| `transform_cloud_to_local_frame` | `true` | TF를 사용해 클라우드 점을 `local_frame_id`로 변환합니다. |
| `center_box_filter_half_extent_m` | `0.80` (launch) | 센서 주변 중앙 정사각형 영역을 제거합니다. 노드 선언 기본값은 `0.0`입니다. |
| `range_min_m`, `range_max_m` | `0.80`, `12.0` | 점유 endpoint에만 적용되는 XY 범위입니다. max가 0이면 상한을 사용하지 않습니다. |
| `enable_raycast_free_space` | `true` | 센서 원점에서 반환점 방향으로 빈 공간 근거를 추적합니다. |
| `raycast_min_range_m`, `raycast_max_range_m` | `0.80`, `12.0` | `range_*`와 독립적인 빈 공간 ray 범위입니다. 두 범위를 혼동하지 마세요. |
| `raycast_clear_occupied` | `false` | 빈 공간 ray가 점유 셀을 덮어쓸 수 있게 할지 결정합니다. false이면 점유 셀을 보호합니다. |
| `occupied_threshold_points` | `1` | 셀을 점유로 표시하는 데 필요한 점 개수입니다. 최소 1입니다. |

세 raycast 값(`raycast_unknown_value`, `raycast_free_value`, `raycast_occupied_value`)은 `[-1, 100]` 범위에서 서로 달라야 합니다. `range_min_m/range_max_m`는 점유 endpoint 범위이고, `raycast_min_range_m/raycast_max_range_m`는 센서 원점에서 빈 공간을 추적하는 범위입니다.

### 3. 필터와 두 로봇 정렬

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `enable_rear_lidar_filter` | `false` (live), `true` (bag) | 후방 섹터 LiDAR 필터를 켭니다. |
| `rear_filter_angle_deg` | `120.0` | 제거할 후방 섹터의 폭입니다. |
| `rear_filter_axis` | `-x` | 후방 섹터 중심 축입니다. `x`, `-x`, `y`, `-y` 중 하나입니다. |
| `rear_filter_min_xy_range_m` | `0.0` | 이 범위 이하의 점은 필터에서 유지합니다. |
| `robot_delay_s` | `20.0` | 두 bag 실행에서 두 번째 로봇 재생을 시작할 지연 시간입니다. |
| `alignment_voxel_size` | `0.05` (two-bag launch) | 초기 XY ICP 정렬에 사용하는 XY voxel 크기입니다. |
| `alignment_min_fitness`, `alignment_max_rmse` | `0.05`, `0.40` | ICP 정렬 결과를 수용하는 기준입니다. |
| `alignment_max_iterations` | `80` | ICP 반복 횟수의 상한입니다. |
| `alignment_recompute_period_sec` | `5.0` | 정렬 재계산 주기입니다. |
| `record_output_prefix` | `/toy_record` | 기록/재게시 토픽의 출력 루트입니다. |
| `record_publish_merged_global` | `true` (two-bag launch) | 병합 전역 occupancy를 publish합니다. 노드 선언 기본값은 `false`입니다. |

### 4. 실행 시간 설정

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `use_bag` | `false` (live), `true` (single-bag) | 실시간 센서 입력과 bag 입력 중 하나를 선택합니다. |
| `use_sim_time` | `false` (live) | ROS clock 대신 bag clock을 사용할 때 `true`로 설정합니다. |
| `bag_path` | 없음 | bag mode에서 `metadata.yaml`이 있는 rosbag2 디렉터리입니다. |
| `rate` | `1.0` | bag 재생 속도입니다. |
| `storage_id` | `sqlite3` | rosbag2 storage 형식입니다. |

occupancy map 기본값은 [`config/occupancy.yaml`](../config/occupancy.yaml), launch 기본값과 연결 관계는 아래의 그 외 파라미터 목록 및 [`README.md`](../README.md)를 참고하세요.

## 그 외 파라미터

아래 값들은 특수한 센서, 디버깅, 성능 조정 또는 선택 기능에 사용할 때 확인합니다. 설명을 반복하지 않고 파라미터명과 기본값만 정리합니다.

### C++ `occupancy_mapper`

| 파라미터 | 기본값 |
| --- | --- |
| `use_odom_header_frame` | `true` |
| `slice_in_global_frame` | `false` |
| `slice_z_in_cloud_frame` | `true` |
| `log_z_slice_stats` | `true` |
| `publish_slice_debug_points` | `true` |
| `slice_debug_points_max_points` | `80000` |
| `raycast_free_value` | `0` |
| `raycast_occupied_value` | `100` |
| `raycast_unknown_value` | `-1` |
| `publish_period_ms` | `200` |
| `global_map_padding_m` | `5.0` |
| `sync_queue_size` | `100` |
| `alignment_required` | `false` |
| `alignment_topic` | `/toy/initial_xy_alignment` |

### 초기 XY ICP 정렬 노드

`initial_xy_icp_alignment.py`의 그 외 파라미터는 다음과 같습니다.

| 파라미터 | 기본값 |
| --- | --- |
| `input_mode` | `cloud_initial` |
| `robot0_cloud_topic`, `robot1_cloud_topic` | `/r0/livox/lidar`, `/r1/livox/lidar` |
| `robot0_map_topic`, `robot1_map_topic` | `/r0/toy/global_occupancy`, `/r1/toy/global_occupancy` |
| `alignment_topic` | `/toy/initial_xy_alignment` |
| `target_frame_id`, `source_frame_id` | `odom`, `r1/odom` |
| `local_frame_id` | `base_link` |
| `transform_cloud_to_local_frame` | `true` |
| `z_min`, `z_max`, `invert_z_slice` | `0.4`, `0.8`, `true` |
| `frame_count` | `5` |
| `invert_result` | `false` |
| `center_box_half_extent_m` | `0.0` |
| `voxel_size` | `0.10` |
| `max_points` | `30000` |
| `max_correspondence_distance` | `0.75` |
| `min_correspondences` | `100` |
| `min_fitness`, `max_rmse` | `0.05`, `0.40` |
| `max_iterations` | `80` |
| `recompute_period_sec` | `5.0` |
| `occupied_threshold` | `50` |
| `convergence_translation_m`, `convergence_rotation_rad` | `1e-4`, `1e-4` |
| `publish_period_sec` | `1.0` |

### record 재전달 노드

| 파라미터 | 기본값 |
| --- | --- |
| `target_frame_id`, `common_frame_id` | `odom`, `map` |
| `alignment_topic` | `/toy/initial_xy_alignment` |
| `publish_period_ms` | `200` |
| `occupied_threshold` | `50` |
| `merged_padding_m` | `1.0` |
| `robot_ids` | `[0, 1]` |
| `output_prefix` | `/toy_record` |
| `robot_odom_frame_format` | `r{robot_id}/odom` |
| `robot_base_frame_format` | `r{robot_id}/base_link` |
| `publish_tf` | `true` |
| `publish_merged_global` | `false` |

### 후방 섹터 필터와 IMU 재전달 노드

| 노드 | 파라미터 | 기본값 |
| --- | --- | --- |
| `pointcloud_rear_sector_filter.py` | `input_topic`, `output_topic` | `/livox/lidar_raw`, `/livox/lidar` |
|  | `enabled` | `true` |
|  | `rear_filter_angle_deg` | `120.0` |
|  | `rear_axis` | `-x` |
|  | `min_xy_range_m` | `0.0` |
|  | `log_period` | `100` |
|  | `output_frame_id` | 비어 있음 |
| `imu_frame_republisher.py` | `input_topic`, `output_topic` | `/livox/imu_filtered_raw_frame`, `/livox/imu_filtered` |
|  | `output_frame_id` | 비어 있음 |

### Rerun 매핑 노드

| 파라미터 | 기본값 |
| --- | --- |
| `spawn_viewer` | `true` |
| `rerun_port` | `9876` |
| `occupancy_point_radius` | `0.045` |
| `slice_point_radius` | `0.025` |
| `odometry_point_radius` | `0.04` |

### Launch 인자 중 자주 쓰지 않는 값

| Launch 파일 | 파라미터/인자 |
| --- | --- |
| `live_mapping.launch.py` | `robot_id`, `publish_tf_odom`, `expected_update_rate`, `bag_lidar_topic`, `imu_filtered_topic`, `alignment_topic`, `transform_cloud_to_local_frame`, `center_box_filter_half_extent_m`, `slice_z_in_cloud_frame` |
| `bag_mid360.launch.py` | `lidar_topic`, `imu_topic`, `play_tf_static` |
| `single_bag_mapping.launch.py` | `robot_id`, `publish_tf_odom`, `publish_sensor_static_tf`, `rear_filter_log_period`, `alignment_topic`, sensor TF 인자 전체 |
| `two_bag_mapping.launch.py` | `record_publish_period_ms`, `alignment_startup_delay_s`, `alignment_z_min`, `alignment_z_max`, `alignment_invert_z_slice`, `alignment_frame_count`, `alignment_invert_result`, `alignment_center_box_half_extent_m`, `alignment_max_points`, `alignment_min_correspondences`, `alignment_convergence_translation_m`, `alignment_convergence_rotation_rad`, 센서 TF 인자 전체 |
| `rerun_mapping.launch.py` | `config_file`, `spawn_viewer`, `rerun_port` |

단일 로봇 live mode에서는 [`rviz/single_robot_mapping.rviz`](../rviz/single_robot_mapping.rviz)를 사용하고, 두 로봇 bag mode에서는 [`rviz/two_robot_mapping.rviz`](../rviz/two_robot_mapping.rviz)를 사용합니다.
