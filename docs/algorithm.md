# TurtleBot WPT 자동 정합 알고리즘

## 1. 상태 머신

```text
ALIGN_COIL → FINAL_STOP → CHARGING
```

`layout_mode: four_coil_map`에서는 4개 코일 주변의 16개 marker 중 목표 코일의 pair를 직접 사용하므로 최종 정합 상태에서 시작합니다. 기존 shelf/head-tag 방식과 station map 방식은 backwards compatibility 용도로 남아 있습니다.

## 2. 상태별 판단 및 구동

| 상태 | 판단 기준 | 구동 명령 |
|---|---|---|
| `ALIGN_COIL` | target coil의 tag pair midpoint/angle 오차 | `/cmd_vel`로 전후진/회전 보정 |
| `FINAL_STOP` | N프레임 연속 정합 완료 | 정지 명령 |
| `CHARGING` | 정지 상태 유지 | WPT 충전 인터페이스 ON |

## 3. 오차 계산

```text
pair_midpoint_x = (tag_a_x + tag_b_x) / 2
pair_midpoint_y = (tag_a_y + tag_b_y) / 2
pair_angle = atan2(tag_b_y - tag_a_y, tag_b_x - tag_a_x)

x_error = pair_midpoint_x - target_x
y_error = pair_midpoint_y - target_y
angle_error = normalize(pair_angle - target_angle)
```

최종 WPT 정합에는 West/East pair를 기본으로 사용합니다. `coil_1`에서는 실제 marker ID `14`, `12`를 동시에 인식해야 합니다.
90도 회전 후 방향 검증에는 North/South pair를 사용합니다. `coil_1`에서는 marker `11`, `13`입니다.

## 4. 속도 변환

TurtleBot은 차동구동이므로 좌우 평행이동 대신 회전으로 보정합니다.

```text
linear.x  = k_y * y_error
angular.z = k_x * x_error + k_angle * angle_error
```

카메라 장착 방향에 따라 YAML의 `invert_linear`, `invert_angular`로 부호를 바꿉니다.

## 5. 최종 정지 조건

```text
abs(x_error) <= threshold_x_px
abs(y_error) <= threshold_y_px
abs(angle_error) <= threshold_angle_deg
stable_count >= stable_frames_required
```

두 tag가 모두 보이지 않거나 pair 조건이 깨지면 `stable_count`를 0으로 되돌리고 정지합니다.

초기 추천값은 x/y 5~10 px, angle 1~3 deg, stable 10 frame입니다.

## 6. 안전 조건

1. tag가 일정 시간 이상 사라지면 즉시 정지합니다.
2. 실제 publish 전에는 `dry_run: true`로 확인합니다.
3. 정밀 정합 속도는 `0.005~0.02 m/s` 범위에서 시작합니다.
4. WPT 송신부는 `FINAL_STOP` 이후에만 켭니다.
