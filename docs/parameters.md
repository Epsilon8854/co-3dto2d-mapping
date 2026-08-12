# 파라미터 참고 문서

이 문서는 현재 패키지의 기준 문서입니다. 값은 노드 선언부, `config/occupancy.yaml`, `config/rerun.yaml`, 그리고 저장소에 포함된 launch 파일에서 가져왔습니다. 기존 Swarm-SLAM의 `docs/toy-occupancy-map.md`는 역사적 배경 자료이며 이 패키지의 인터페이스 문서가 아닙니다.

## 우선순위와 표기 규칙

`occupancy_mapper`의 최종 값은 다음 순서로 결정됩니다: **노드 선언 기본값 → YAML 파일 → launch 재정의**. `single_bag_mapping`은 먼저 `occupancy_config_file`을 전달하고 그 다음 명시적인 파라미터 딕셔너리를 전달하므로, 딕셔너리의 값이 `config/occupancy.yaml`보다 우선합니다. 다른 Python 노드는 launch 딕셔너리나 YAML에서 값을 제공하지 않는 한 선언 기본값을 사용합니다.

거리의 단위는 m이며, 주기는 이름에 표시된 대로 ms 또는 s입니다. `_deg` 접미사는 도, 각도 수렴 조건의 단위는 rad입니다. `/`로 시작하는 토픽은 절대 토픽이며, 매퍼의 상대 출력 토픽은 노드 namespace 안에서 해석됩니다. 점유 격자 값은 **미관측(unknown) `-1`**, **빈 공간(free) `0`**, **점유(occupied) `100`** 규약을 따릅니다.

## C++ 점유 격자 매퍼

노드: `occupancy_mapper` (노드 이름 `occupancy_mapper`). PointCloud2와 Odometry를 근사 동기화한 뒤 `toy/local_occupancy`, `toy/global_occupancy`, `toy/slice_kept_points`, `toy/slice_rejected_points`를 발행합니다. `/r0` 네임스페이스에서는 이 토픽들이 `/r0/toy/*`가 됩니다.

| 파라미터 | 선언 기본값 | 동작 및 검증 |
| --- | --- | --- |
| `scan_cloud_topic` | `/livox/lidar` | PointCloud2 입력 토픽입니다. |
| `odom_topic` | `odom` | Odometry 입력 토픽입니다. |
| `local_frame_id` | `base_link` | 로컬 클라우드 변환과 매핑에 사용할 프레임입니다. |
| `global_frame_id` | `odom` | 격자 프레임입니다. 첫 Odometry header의 frame이 있으면 이 값이 대체될 수 있습니다. |
| `use_odom_header_frame` | `true` | 사용 가능한 경우 첫 Odometry header에서 전역 프레임을 가져옵니다. |
| `grid_resolution` | `0.10` | 셀 한 변의 길이이며 양수여야 합니다. |
| `local_map_size_m` | `20.0` | 로컬 격자의 한 변 크기이며 `grid_resolution`보다 커야 합니다. |
| `z_min`, `z_max` | `0.4`, `1.2` | Z 슬라이스 범위이며 `z_min <= z_max`여야 합니다. |
| `slice_in_global_frame` | `false` | 로컬/클라우드 좌표가 아닌 전역 좌표에서 Z를 선택합니다. |
| `slice_z_in_cloud_frame` | `true` | 전역 좌표를 사용하지 않을 때 보정된 로컬 좌표가 아닌 입력 클라우드에서 Z를 선택합니다. |
| `invert_z_slice` | `false` | 선택한 Z 범위 안쪽이 아닌 바깥쪽의 점을 유지합니다. |
| `log_z_slice_stats` | `true` | 슬라이스의 점 개수와 범위를 로그로 출력합니다. |
| `publish_slice_debug_points` | `true` | 유지/제거된 점의 디버그 PointCloud2 토픽을 발행합니다. |
| `slice_debug_points_max_points` | `80000` | 디버그 점 개수의 상한이며 최소 1로 제한됩니다. |
| `transform_cloud_to_local_frame` | `true` | TF를 사용해 클라우드 점을 `local_frame_id`로 변환합니다. |
| `center_box_filter_half_extent_m` | `0.0` | 중앙 정사각형 영역을 제거하며 음수가 되지 않도록 제한됩니다. |
| `range_min_m`, `range_max_m` | `0.0`, `0.0` | **점유 끝점에만** 적용되는 XY 범위입니다. max가 0이면 상한을 사용하지 않습니다. 두 값은 음수가 되지 않으며, 상한을 사용하면 max가 min보다 작을 수 없습니다. |
| `enable_raycast_free_space` | `true` | 센서 원점에서 유효한 반환점 방향으로 빈 공간 근거를 추적합니다. |
| `raycast_free_value` | `0` | 빈 공간 셀에 사용할 격자 값입니다. |
| `raycast_occupied_value` | `100` | 히트 셀에 사용할 격자 값입니다. |
| `raycast_unknown_value` | `-1` | 초기 또는 미관측 격자 값입니다. |
| `raycast_max_range_m`, `raycast_min_range_m` | `12.0`, `0.80` | 독립적인 빈 공간 ray 범위입니다. 음수가 될 수 없고, 상한을 사용하면 max가 min보다 작을 수 없습니다. |
| `raycast_clear_occupied` | `false` | 빈 공간 ray가 점유 셀을 덮어쓸 수 있게 합니다. false이면 히트 셀을 보호합니다. |
| `occupied_threshold_points` | `1` | 셀을 점유로 표시하는 데 필요한 점 개수이며 최소 1로 제한됩니다. |
| `publish_period_ms` | `200` | 격자 발행 주기이며 최소 1 ms로 제한됩니다. |
| `global_map_padding_m` | `5.0` | 누적 격자가 확장될 때 사용할 테두리이며 음수가 되지 않도록 제한됩니다. |
| `sync_queue_size` | `100` | 근사 시간 동기화 큐의 깊이이며 최소 1로 제한됩니다. |
| `alignment_required` | `false` | 매퍼 설정에 따라 초기 XY 정렬 토픽을 구독합니다. |
| `alignment_topic` | `/toy/initial_xy_alignment` | 필요한 경우 구독하는 transient-local reliable TransformStamped 입력 토픽입니다. |

세 raycast 값은 `[-1, 100]` 범위에 있어야 하며 서로 달라야 합니다. `range_min_m/range_max_m`와 `raycast_min_range_m/raycast_max_range_m`를 혼동하지 마세요. 점유 히트 범위 밖의 반환점도 끝점 이전 구간에 빈 공간 근거를 만들 수 있지만, 해당 끝점 자체는 빈 공간이나 점유로 기록되지 않습니다.

### `config/occupancy.yaml`

```yaml
/**:
  ros__parameters:
    scan_cloud_topic: "/livox/lidar"
    odom_topic: "odom"
    local_frame_id: "base_link"
    global_frame_id: "odom"
    grid_resolution: 0.10
    local_map_size_m: 20.0
    z_min: 0.4
    z_max: 1.2
    slice_in_global_frame: false
    slice_z_in_cloud_frame: true
    invert_z_slice: true
    log_z_slice_stats: true
    publish_slice_debug_points: true
    slice_debug_points_max_points: 80000
    transform_cloud_to_local_frame: true
    center_box_filter_half_extent_m: 0.80
    range_min_m: 0.80
    range_max_m: 12.0
    enable_raycast_free_space: true
    raycast_free_value: 0
    raycast_occupied_value: 100
    raycast_unknown_value: -1
    raycast_max_range_m: 12.0
    raycast_min_range_m: 0.80
    raycast_clear_occupied: false
    occupied_threshold_points: 1
    publish_period_ms: 200
    global_map_padding_m: 5.0
    sync_queue_size: 100
```

YAML은 `invert_z_slice`의 선언 기본값 `false`를 `true`로 바꾸고 `center_box_filter_half_extent_m`를 `0.0`에서 `0.80`으로 바꾸며, 선언부에서 `0.0`으로 둔 점유 범위도 지정합니다. `use_odom_header_frame`, `alignment_required`, `alignment_topic`은 YAML에 없으므로 노드를 직접 실행하면 선언 기본값이 유지됩니다. single/two-bag launch는 `center_box_filter_half_extent_m=0.80`과 `slice_z_in_cloud_frame=true`도 명시적으로 재정의합니다.

## Python 노드

### 초기 XY ICP 정렬

노드: `initial_xy_icp_alignment.py`. 초기 클라우드(`input_mode=cloud_initial`) 또는 전역 격자(`global_occupancy`)를 입력으로 받아 transient-local TransformStamped를 발행합니다.

| 파라미터 | 기본값 | 동작 / 제한 |
| --- | --- | --- |
| `input_mode` | `cloud_initial` | `cloud_initial` 또는 `global_occupancy`만 사용할 수 있으며, 다른 값이면 오류가 발생합니다. |
| `robot0_cloud_topic`, `robot1_cloud_topic` | `/r0/livox/lidar`, `/r1/livox/lidar` | 클라우드 모드에서 사용할 클라우드 입력입니다. |
| `robot0_map_topic`, `robot1_map_topic` | `/r0/toy/global_occupancy`, `/r1/toy/global_occupancy` | 맵 모드에서 사용할 OccupancyGrid 입력입니다. |
| `alignment_topic` | `/toy/initial_xy_alignment` | 정렬 결과를 발행할 토픽입니다. |
| `target_frame_id`, `source_frame_id` | `odom`, `r1/odom` | 결과 Transform의 부모/자식 프레임 ID입니다. |
| `local_frame_id` | `base_link` | 선택적 클라우드 TF 변환의 대상 프레임입니다. |
| `transform_cloud_to_local_frame` | `true` | 클라우드 프레임을 TF로 변환할지 결정합니다. |
| `z_min`, `z_max`, `invert_z_slice` | `0.4`, `1.2`, `true` | 클라우드 Z 슬라이스 설정입니다. |
| `frame_count` | `5` | 초기 프레임을 캐시하는 개수이며 최소 1로 제한됩니다. |
| `invert_result` | `false` | 추정한 평면 변환을 반전합니다. |
| `center_box_half_extent_m` | `0.0` | 중앙 정사각형 영역을 제외하며 음수가 되지 않도록 제한됩니다. |
| `voxel_size` | `0.10` | XY voxel 크기이며 음수가 되지 않도록 제한됩니다. |
| `max_points` | `30000` | 점 개수 상한이며 최소 100으로 제한됩니다. |
| `max_correspondence_distance` | `0.75` | ICP 대응점 거리입니다. |
| `min_correspondences` | `100` | 허용되는 최소 대응점 개수이며 최소 3으로 제한됩니다. |
| `min_fitness`, `max_rmse` | `0.05`, `0.40` | 정렬 결과를 수용할 기준입니다. |
| `max_iterations` | `80` | ICP 반복 횟수 상한이며 최소 1로 제한됩니다. |
| `recompute_period_sec` | `5.0` | 재계산 주기이며 최소 0.1 s로 제한됩니다. |
| `occupied_threshold` | `50` | 이 값 이상인 격자 값을 정렬 점으로 사용합니다. |
| `convergence_translation_m`, `convergence_rotation_rad` | `1e-4`, `1e-4` | ICP의 이동/회전 수렴 허용 오차입니다. |
| `publish_period_sec` | `1.0` | 결과 발행 timer 주기이며 최소 0.1 s를 사용합니다. |

### record 재전달 노드

노드: `record_republisher.py`. 로봇별 `/rN/odom`, `/rN/toy/*`와 정렬 결과를 입력으로 받아 `output_prefix` 아래에 다시 발행합니다. 로봇별 맵 프레임은 `robot_odom_frame_format`을 사용하고, 병합 맵은 `common_frame_id`를 사용합니다.

| 파라미터 | 기본값 | 동작 / 제한 |
| --- | --- | --- |
| `target_frame_id`, `common_frame_id` | `odom`, `map` | 입력 target에 대한 기대값과 병합/TF에 사용할 공통 프레임입니다. |
| `alignment_topic` | `/toy/initial_xy_alignment` | 정렬 TransformStamped 입력 토픽입니다. |
| `publish_period_ms` | `200` | 출력 timer 주기이며 최소 1 ms로 제한됩니다. |
| `occupied_threshold` | `50` | 병합 시 점유로 간주할 격자 값의 기준입니다. |
| `merged_padding_m` | `1.0` | 병합 격자의 테두리이며 음수가 되지 않도록 제한됩니다. |
| `robot_ids` | `[0, 1]` | 구독하고 발행할 정수 로봇 ID 목록입니다. |
| `output_prefix` | `/toy_record` | 출력 루트이며 끝의 slash는 제거됩니다. |
| `robot_odom_frame_format` | `r{robot_id}/odom` | 로봇별 맵 프레임 템플릿입니다. |
| `robot_base_frame_format` | `r{robot_id}/base_link` | TF에 사용할 로봇별 base 프레임 템플릿입니다. |
| `publish_tf` | `true` | 정렬/로봇 TF를 발행합니다. |
| `publish_merged_global` | `false` | `<prefix>/merged_global_occupancy`를 발행합니다. |

### 후방 섹터 필터와 IMU 재전달 노드

| 노드 | 파라미터 | 기본값 | 동작 / 검증 |
| --- | --- | --- | --- |
| `pointcloud_rear_sector_filter.py` | `input_topic`, `output_topic` | `/livox/lidar_raw`, `/livox/lidar` | PointCloud2 입력과 출력 토픽입니다. |
|  | `enabled` | `true` | false이면 입력을 변경하지 않고 통과시킵니다. |
|  | `rear_filter_angle_deg` | `120.0` | 제거할 섹터의 폭이며, 절반으로 나누기 전에 `[0, 360]`으로 제한됩니다. |
|  | `rear_axis` | `-x` | 섹터 중심 축입니다. `x`, `-x`, `y`, `-y` 중 하나여야 하며 다른 값이면 오류가 발생합니다. |
|  | `min_xy_range_m` | `0.0` | 이 범위 이하의 점은 유지합니다. |
|  | `log_period` | `100` | 양수이면 N개 메시지마다 누적 통계를 로그로 출력합니다. |
|  | `output_frame_id` | 비어 있음 | 비어 있지 않을 때만 클라우드 프레임을 교체합니다. |
| `imu_frame_republisher.py` | `input_topic`, `output_topic` | `/livox/imu_filtered_raw_frame`, `/livox/imu_filtered` | IMU 입력과 출력 토픽입니다. |
|  | `output_frame_id` | 비어 있음 | 비어 있으면 입력 프레임을 유지하고, 그렇지 않으면 지정한 프레임으로 교체합니다. |

### Rerun 매핑 노드

노드: `rerun_mapping_node.py` (`co_3dto2d_rerun`). `/toy_record/r0/*`, `/toy_record/r1/*`, 선택적인 병합 점유 격자, `/toy/initial_xy_alignment`을 구독합니다. 이 노드는 선택 사항입니다.

| 파라미터 | 선언 / `config/rerun.yaml` 기본값 | 동작 |
| --- | --- | --- |
| `spawn_viewer` | `true` | 호환되는 `rerun` 뷰어 프로세스를 시작합니다. |
| `rerun_port` | `9876` | 뷰어/프록시 포트입니다. |
| `occupancy_point_radius` | `0.045` | 점유 셀을 그릴 때 사용할 반경입니다. |
| `slice_point_radius` | `0.025` | 슬라이스 점을 그릴 때 사용할 반경입니다. |
| `odometry_point_radius` | `0.04` | Odometry 점을 그릴 때 사용할 반경입니다. |

## Launch 인자 매트릭스

| Launch 파일 | 주요 인자(기본값) | 네임스페이스/토픽/프레임 동작 |
| --- | --- | --- |
| `live_mapping.launch.py` | `robot_id=0`, `use_sim_time=false`, `scan_cloud_topic=/livox/lidar`, `imu_raw_topic=/livox/imu`, `enable_rear_lidar_filter=false` | `use_bag=false`로 설정하고 실시간 Livox 토픽을 입력으로 받아 `/r<robot_id>` 아래에서 단일 로봇 매퍼를 시작합니다. |
| `bag_mid360.launch.py` | `bag_path` 필수, `rate=1.0`, `storage_id=sqlite3`, `lidar_topic=/livox/lidar`, `imu_topic=/livox/imu`, `play_tf_static=true` | `metadata.yaml`을 검증하고 LiDAR, IMU, 선택적인 `/tf_static`을 재생한 뒤 remap합니다. |
| `single_bag_mapping.launch.py` | `robot_id=0`, `use_bag=true`, bag 모드에서 `bag_path` 필수, `rate=1.0`, `storage_id=sqlite3`, `occupancy_config_file=config/occupancy.yaml` | 네임스페이스는 `/r<robot_id>`이며 기본 매퍼 프레임은 `base_link`와 `odom`입니다. 실시간 입력으로 이 파이프라인을 포함할 때는 `use_bag=false`로 설정합니다. |
|  | `enable_rear_lidar_filter=true`, `rear_filter_angle_deg=120`, `rear_filter_axis=-x`, `rear_filter_min_xy_range_m=0` | 필터링된 LiDAR 라우팅을 설정합니다. IMU raw/filtered 기본 토픽은 `/livox/imu`와 `/livox/imu_filtered`입니다. |
|  | `center_box_filter_half_extent_m=0.80`, `slice_z_in_cloud_frame=true`, `transform_cloud_to_local_frame=true` | YAML 이후 적용되는 명시적인 매퍼 launch 재정의입니다. |
| `two_bag_mapping.launch.py` | `bag_path_0`, `bag_path_1` 필수, `rate=1.0`, `storage_id=sqlite3`, `robot_delay_s=20` | `/r0`, `/r1`을 만들고 `rN/base_link`, `rN/livox_frame`, `rN/odom` 프레임을 사용합니다. |
|  | `enable_record_republisher=true`, `record_output_prefix=/toy_record`, `record_publish_merged_global=true` | `map` 프레임 기반 RViz/기록 화면을 활성화합니다. |
|  | `alignment_*` | 정렬은 전역 격자를 읽으며, `alignment_voxel_size=0.05`가 ICP 선언 기본값 `0.10`을 재정의합니다. |
| `rerun_mapping.launch.py` | `config_file=config/rerun.yaml`, `spawn_viewer=true`, `rerun_port=9876` | 선택적인 비네임스페이스 Rerun 구독 노드를 시작하며 launch 값이 YAML보다 우선합니다. |

공통 센서 정적 transform 인자는 `sensor_tf_x/y/z`, `sensor_tf_yaw/pitch/roll`, `sensor_parent_frame`, `sensor_child_frame`, `publish_sensor_static_tf`입니다. 기본값은 `sensor_tf_roll=3.141592653589793`을 제외하면 이동량과 각도가 모두 0입니다. 실제 데이터를 사용할 때는 측정한 MID-360 외부 파라미터(extrinsic) 값으로 교체하세요.
