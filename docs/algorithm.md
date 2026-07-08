# TurtleBot WPT 자동 정합 알고리즘

> 이 문서는 `ros2 run`으로 도는 ROS2 노드(`wpt_alignment_node`)의 정밀 pair 정합 로직을 설명합니다. 현재 실험은 이 노드 없이 라즈베리파이에 SSH로 직접 접속해 스크립트를 실행하는 방식(3x3 그리드 정합)으로 진행 중이며, 그 절차는 README를 참고하세요. 이 문서의 상태 머신/오차 계산은 나중에 ROS2 패키지로 정식 자동화할 때 쓸 참고 자료입니다.

## 1. 상태 머신

```text
ENTER_SHELF → ALIGN_COIL → FINAL_STOP → CHARGING
```

head tag를 더 이상 사용하지 않으므로(2x2 레이아웃, coil tag 16개만 인쇄), `SEARCH_HEAD_TAG`/`APPROACH_SHELF` 상태는 제거되었습니다. `ENTER_SHELF`가 초기 상태이며, 목표 선반의 coil tag가 보일 때까지 저속으로 전진하는 탐색 역할까지 겸합니다.

로봇이 선반에 어느 방향으로 진입할지 미리 알 수 없으므로, 단일 태그가 아니라 마주보는 두 태그를 한 쌍(pair)으로 인식해서 정합합니다. 정합 대상 페어는 `config/wpt_alignment.yaml`의 `alignment.final_pair`(기본값 `west_east`)로 지정합니다. 전방 카메라 1대 + 하부/측면 카메라 2대 중 페어의 두 태그가 모두 보이는 카메라를 골라 사용하며(면적 합이 가장 큰, 즉 가장 가까운 카메라 우선), 페어 중 한쪽 태그만 보이는 경우에는 지나치지 않도록 정지하고, 둘 다 안 보일 때만 저속 전진합니다.

## 2. 상태별 판단 및 구동

| 상태 | 판단 기준 | 구동 명령 |
|---|---|---|
| `ENTER_SHELF` | 목표 페어(예: west+east) 인식 여부 (초기 탐색 포함) | 둘 다 안 보이면 복도 저속 전진, 한쪽만 보이면 정지 대기, 둘 다 보이면 `ALIGN_COIL`로 전환 |
| `ALIGN_COIL` | 페어 중점(midpoint)/페어 각도(pair angle) 오차 | `/cmd_vel`로 전후진/회전 보정 |
| `FINAL_STOP` | N프레임 연속 정합 완료 | 정지 명령 |
| `CHARGING` | 정지 상태 유지 | WPT 충전 인터페이스 ON |

## 3. 오차 계산

두 태그(예: west/east)의 중점과, 두 태그 중심을 잇는 각도를 기준으로 오차를 계산합니다.

```text
midpoint_x = (tag_a_x + tag_b_x) / 2
midpoint_y = (tag_a_y + tag_b_y) / 2
pair_angle = atan2(tag_b_y - tag_a_y, tag_b_x - tag_a_x)

x_error = midpoint_x - target_x
y_error = midpoint_y - target_y
angle_error = normalize(pair_angle - target_angle)
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
