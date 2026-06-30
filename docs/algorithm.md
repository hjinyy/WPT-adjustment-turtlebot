# TurtleBot WPT 자동 정합 알고리즘

## 1. 상태 머신

```text
SEARCH_HEAD_TAG → APPROACH_SHELF → ENTER_SHELF → ALIGN_COIL → FINAL_STOP → CHARGING
```

## 2. 상태별 판단 및 구동

| 상태 | 판단 기준 | 구동 명령 |
|---|---|---|
| `SEARCH_HEAD_TAG` | 목표 head tag ID 인식 여부 | 안 보이면 복도 저속 전진 |
| `APPROACH_SHELF` | head tag 중심/각도 오차 | 중심으로 맞추며 접근 |
| `ENTER_SHELF` | 목표 선반의 coil tag 인식 여부 | 선반 내부로 매우 저속 진입 |
| `ALIGN_COIL` | coil tag 중심/각도 오차 | `/cmd_vel`로 전후진/회전 보정 |
| `FINAL_STOP` | N프레임 연속 정합 완료 | 정지 명령 |
| `CHARGING` | 정지 상태 유지 | WPT 충전 인터페이스 ON |

## 3. 오차 계산

```text
x_error = tag_x - target_x
y_error = tag_y - target_y
angle_error = normalize(tag_angle - target_angle)
```

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

초기 추천값은 x/y 5~10 px, angle 1~3 deg, stable 10 frame입니다.

## 6. 안전 조건

1. tag가 일정 시간 이상 사라지면 즉시 정지합니다.
2. 실제 publish 전에는 `dry_run: true`로 확인합니다.
3. 정밀 정합 속도는 `0.005~0.02 m/s` 범위에서 시작합니다.
4. WPT 송신부는 `FINAL_STOP` 이후에만 켭니다.
