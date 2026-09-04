# Initial 3D ICP의 180도 뒤집힘과 과도한 tilt 방지

두 로봇이 보는 구조가 복도나 사각형 방처럼 대칭적인 경우, 기하학적 fitness만 보는
ICP는 `r1` cloud를 약 180도 회전한 해를 선택할 수 있습니다. 현재 live mode의
initial alignment는 RTAB-Map 입력에서 만든 cropped XYZ submap을 사용하므로, 기존
상대 yaw prior에 더해 full 3D 회전이 비현실적으로 기울어지는 것도 제한합니다.

기본 동작은 다음과 같습니다.

- 기대 상대 yaw: `0 deg`
- 허용 yaw 편차: `±90 deg`
- 초기 yaw 후보: `0, -30, +30 deg`
- 한 ICP 반복에서 허용하는 최대 3D 회전량: `60 deg`
- 최종 허용 tilt: `15 deg`
- 반복 중 heading/tilt 허용 영역을 벗어나면 후보 폐기
- 비슷한 fitness의 후보에서는 기대 yaw에 가까운 결과 우선

따라서 180도 결과의 fitness나 RMSE가 더 좋아도 publish되지 않습니다. 잘못된 결과는
고정하지 않고 fresh cropped XYZ submap을 모아 다시 시도합니다. 정합 자체는 full
3D이고, 통과한 결과는 merged 2D occupancy 인터페이스에 맞춰 x/y/yaw로 투영됩니다.

실제 초기 배치의 상대 방향이 0도가 아니라면 fusion host에서 실행 전에 환경 변수로
yaw prior를 조정할 수 있습니다.

```bash
# 예: r1이 r0 기준 약 25도 돌아가 있는 배치
export CO3DTO2D_EXPECTED_RELATIVE_YAW_DEG=25
export CO3DTO2D_MAX_RELATIVE_YAW_DEVIATION_DEG=60

bash scripts/run_two_mid360_2d_mapping.sh --robot-number 2 --mapping-host
```

사용 가능한 yaw 관련 환경 변수:

| 환경 변수 | 기본값 | 설명 |
| --- | ---: | --- |
| `CO3DTO2D_ENFORCE_HEADING_PRIOR` | `true` | `false`이면 yaw prior를 비활성화합니다. |
| `CO3DTO2D_EXPECTED_RELATIVE_YAW_DEG` | `0` | publish되는 `map -> r1/odom`의 기대 yaw입니다. |
| `CO3DTO2D_MAX_RELATIVE_YAW_DEVIATION_DEG` | `90` | 기대 yaw에서 허용할 최대 편차입니다. |
| `CO3DTO2D_INITIAL_YAW_OFFSETS_DEG` | `0,-30,30` | 쉼표로 구분한 초기 yaw offset 후보입니다. |
| `CO3DTO2D_MAX_ICP_ROTATION_STEP_DEG` | `60` | 한 반복의 최대 3D 회전량입니다. `0`이면 step 제한만 끕니다. |
| `CO3DTO2D_HEADING_PRIOR_WEIGHT` | `0.05` | 유사한 후보 중 기대 yaw에 가까운 후보를 선호하는 가중치입니다. |

`two_live_mapping.launch.py`에서는 tilt 제한을 launch argument로 조정합니다.

```bash
bash scripts/run_two_mid360_2d_mapping.sh --robot-number 2 --mapping-host \
  --launch-arg alignment_enforce_tilt_prior:=true \
  --launch-arg alignment_max_tilt_deviation_rad:=0.1745329252
```

위 예시는 최대 tilt를 10도로 제한합니다. 센서 외부 파라미터가 정확하고 두 로봇이
평탄한 바닥에 있다면 10~15도가 일반적으로 충분합니다. 경사면 실험에서는 실제
상대 자세보다 작게 제한하지 않도록 값을 늘려야 합니다.

로그에서 다음 메시지가 보이면 heading 또는 tilt가 물리 prior로 차단된 것입니다.

```text
ICP rejected by geometric prior
ICP left the allowed heading/tilt region
```

두 로봇이 실제로 90도보다 크게 벌어진 방향으로 시작하는 실험에서는 기대 yaw를 실제
배치에 가깝게 지정하는 것이 좋습니다. 허용 편차만 180도로 넓히면 대칭 환경의 뒤집힌
해를 다시 허용하게 되므로 권장하지 않습니다.
