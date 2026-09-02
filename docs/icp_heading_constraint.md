# Initial ICP의 180도 뒤집힘 방지

두 로봇의 초기 occupancy가 복도나 사각형 방처럼 대칭적인 경우, 기하학적 fitness만
보는 ICP는 `r1` 지도를 약 180도 회전한 해를 선택할 수 있습니다. 실증 배치에서 두
로봇이 반대 방향으로 놓이지 않는다는 물리 조건을 initial alignment에 prior로
적용합니다.

기본 동작은 다음과 같습니다.

- 기대 상대 yaw: `0 deg`
- 허용 편차: `±90 deg`
- 초기 yaw 후보: `0, -30, +30 deg`
- 한 ICP 반복에서 허용하는 최대 회전: `60 deg`
- 반복 중 허용 영역을 벗어나거나 최종 yaw가 반대 반구에 있으면 결과 폐기
- 비슷한 fitness의 후보에서는 기대 yaw에 가까운 결과 우선

따라서 180도 결과의 fitness나 RMSE가 더 좋아도 publish되지 않습니다. 잘못된 결과는
고정하지 않고 다음 map 주기에 다시 시도합니다.

기존 launch 명령은 변경하지 않아도 됩니다. 실제 초기 배치의 상대 방향이 0도가
아니라면 실행 전에 환경 변수로 prior를 조정할 수 있습니다.

```bash
# 예: r1이 r0 기준 약 25도 돌아가 있는 배치
export CO3DTO2D_EXPECTED_RELATIVE_YAW_DEG=25
export CO3DTO2D_MAX_RELATIVE_YAW_DEVIATION_DEG=60

bash scripts/run_two_mid360_2d_mapping.sh --robot-number 2 --mapping-host
```

사용 가능한 환경 변수는 다음과 같습니다.

| 환경 변수 | 기본값 | 설명 |
| --- | ---: | --- |
| `CO3DTO2D_ENFORCE_HEADING_PRIOR` | `true` | `false`이면 yaw prior를 비활성화합니다. |
| `CO3DTO2D_EXPECTED_RELATIVE_YAW_DEG` | `0` | publish되는 `map -> r1/odom`의 기대 yaw입니다. |
| `CO3DTO2D_MAX_RELATIVE_YAW_DEVIATION_DEG` | `90` | 기대 yaw에서 허용할 최대 편차입니다. |
| `CO3DTO2D_INITIAL_YAW_OFFSETS_DEG` | `0,-30,30` | 쉼표로 구분한 초기 yaw offset 후보입니다. |
| `CO3DTO2D_MAX_ICP_ROTATION_STEP_DEG` | `60` | 한 ICP 반복에서 허용할 최대 회전량입니다. `0`이면 step 제한만 끕니다. |
| `CO3DTO2D_HEADING_PRIOR_WEIGHT` | `0.05` | 유사한 기하 후보 중 기대 yaw에 가까운 후보를 선호하는 가중치입니다. |

로그에서 다음 메시지가 보이면 반대 방향 후보가 정상적으로 차단된 것입니다.

```text
XY ICP rejected by heading prior
ICP left the allowed heading hemisphere
```

두 로봇이 실제로 90도보다 크게 벌어진 방향으로 시작하는 실험에서는 기대 yaw를 실제
배치에 가깝게 지정하는 것이 좋습니다. 허용 편차만 180도로 넓히면 대칭 환경의 뒤집힌
해를 다시 허용하게 되므로 권장하지 않습니다.
