# TurtleBot WPT 자동 정합 알고리즘

> 이 문서는 (선택 사항인) `ros2 run`으로 도는 ROS2 노드(`wpt_alignment_node`)와, ROS 없이 터미널에서 바로 실행하는 `scripts/check_camera_alignment.py`가 공유하는 정밀 pair 정합 로직을 설명합니다. 실제 로봇을 움직이지 않고 카메라 정합 판정만 확인하려면 ROS2/터틀봇 브링업 없이 `scripts/check_camera_alignment.py`만 실행하면 됩니다 — README의 "터미널 단독 카메라 정합 테스트" 절 참고.

## 0. layout_mode

`config/wpt_alignment.yaml`의 `layout_mode`로 태그 매핑 방식을 고릅니다.

| layout_mode | 대상 | 태그 ID |
|---|---|---|
| `four_coil_map` (기본값) | 2x2 코일 스테이지, `target_coil: coil_1~4` | 11-14/21-24/31-34/41-44 (이미 인쇄된 태그와 동일, 재인쇄 불필요) |
| `station_map` | 충전 관제 서버 노드(A02 등), `target_station` | 서버 쪽 마커 번호 체계 (station별 5~36) |
| (레거시) shelf/head-tag | 과거 실험용, `target_shelf` | `100+shelf*10+position`, head tag `100+shelf` — 지금은 안 씀 |

`four_coil_map`이 기본값이며, 현재 실제로 인쇄해서 쓰고 있는 태그(11~44)와 그대로 호환됩니다.

### 전역 나침반 정렬 (global compass)

과거에는 각 WPT가 "선반을 바라보는 쪽 = north"로 코일마다 방향이 달랐지만, 이제 **4개 코일 전부 하나의 동서남북**으로 정렬합니다.

```text
        north
    coil_1 | coil_2
    -------+-------      west <-> east
    coil_3 | coil_4
        south
```

즉 coil_1의 south 마커(12)와 coil_3의 south 마커(32)는 물리적으로 같은 방향을 봅니다. 태그를 붙일 때 이 기준으로 정렬하세요 (ID 자체는 그대로라 재인쇄는 불필요).

## 0.5 코일 간 이동 (transit): 라인 추종 + 마커 정지

코일 사이 경로에는 검정 테이프가 붙어 있고, `scripts/drive_between_coils.py`가 이 구간을 담당합니다 (`wpt_adjustment_turtlebot/coil_transit.py`가 순수 로직).

**실측 주행 거리** (코일 중심 ↔ 코일 중심): 세로(1↔3, 2↔4) **25.5cm**, 가로(1↔2, 3↔4) **45.3cm** — `coil_transit.py`의 `LEG_DISTANCE_*` 상수. 이 거리를 기준으로 구간을 2단계로 나눕니다.

예: coil_1 → coil_3 (이동 방향 = south, 25.5cm)

1. **head 마커**: 출발 코일에서 이동 방향에 해당하는 마커 — coil_1의 south(12). 로봇은 이 마커를 지나쳐 나가며 이동을 시작합니다.
2. **CRUISE (실측 거리의 `transit.cruise_fraction`, 기본 60%)**: 라인 추종 + 칼만 보정으로 순항 속도 주행. 양쪽 하부 카메라(left_bottom/right_bottom)가 테이프를 인식(`line_detection.LineDetector`)하고, 두 관측을 신뢰도 가중으로 합친 뒤 칼만 필터(`sensor_fusion.ErrorKalmanFilter`)로 평활화해서 `angular.z`를 연속 보정합니다 — 각도가 틀어져도 누적되지 않습니다. 주행 거리는 명령 속도 기반 추측항법으로 추정합니다. 한쪽 카메라가 잠깐 라인을 놓쳐도 필터의 predict로 이어가고, `line_lost_timeout_sec` 이상 양쪽 다 놓치면 안전 정지합니다.
3. **APPROACH (나머지 거리)**: `transit.approach_linear`(기본 0.02 m/s)로 감속해 정밀 도착 조건을 탐색합니다.
4. **정지 조건 (실험 기반)**: **양쪽 사이드 마커가 보이는 상태에서, 목표 코일의 이동 방향(head) 마커가 전방 카메라에 인식되는 순간 즉시 정지.** south 이동이면 사이드 마커 = coil_3의 west(33)/east(34), head = coil_3의 south(32). head 단독 인식은 코일 중심에 못 미친 스침 인식일 수 있어 정지하지 않고, 사이드 2개가 신선도 창(`side_marker_freshness_sec`, 기본 0.7초) 안에 함께 보여야만 정지합니다 — 실제 실험에서 정합 성공률이 가장 높았던 조합입니다. 마커 ID가 코일마다 다르므로 출발 코일 마커와 혼동하지 않으며, 이 조건은 CRUISE 중에도 검사되어 조기 도착 시에도 안전합니다.
5. 정지 후 정밀 정합은 기존 pair 로직(`wpt_alignment_node` / `check_camera_alignment.py`)으로 이어갑니다.

대각선 이동(coil_1 → coil_4)은 세로 → 가로 두 구간으로 나뉘며, 구간 사이 90도 회전은 아직 자동화하지 않았습니다(회전 중 라인/마커 가림 리스크) — 회전 후 두 번째 구간을 다시 실행하세요.

## 1. 상태 머신

```text
SEARCH_HEAD_TAG → APPROACH_SHELF → ENTER_SHELF ─┐
                                                  ▼
                                            SEARCH_COIL ⇄ ALIGN_COIL → FINAL_STOP → CHARGING
                                              ▲              │
                                              └── (pair 소실, 짧은 back-off 후) ──┘
```

`four_coil_map`/`station_map`처럼 목표 코일/스테이션이 명확히 지정된 경우, 헤드 태그 탐색 없이 바로 `SEARCH_COIL`에서 시작합니다(`_starts_at_alignment()`). 레거시 shelf 모드에서만 `SEARCH_HEAD_TAG`→`APPROACH_SHELF`→`ENTER_SHELF`를 거쳐 `SEARCH_COIL`로 넘어갑니다.

로봇이 선반에 어느 방향으로 진입할지 미리 알 수 없으므로, 단일 태그가 아니라 마주보는 두 태그를 한 쌍(pair)으로 인식해서 정합합니다. 정합 대상 페어는 `config/wpt_alignment.yaml`의 `alignment.final_pair`(기본값 `west_east`)로 지정합니다. 전방 카메라 1대 + 하부/측면 카메라 2대 중 페어의 두 태그가 모두 보이는 카메라를 골라 사용하며(면적이 가장 큰, 즉 가장 가까운 카메라 우선), 페어 중 한쪽 태그만 보이는 경우에는 지나치지 않도록 정지하고, 둘 다 안 보일 때만 저속 전진합니다.

### SEARCH_COIL과 ALIGN_COIL을 분리한 이유

하부/측면 카메라는 로봇 몸체에 가깝게 낮은 위치에 달려 있어서, **회전하는 동안** 바퀴나 몸체가 카메라 시야를 가리는 경우가 있습니다. `is_aligned` 판정에 필요한 pair 오차 계산은 두 마커가 모두 안정적으로 보여야 정확한데, 회전 중에 계속 이미지를 읽고 계속 회전 명령을 내보내는 연속(closed-loop) 방식은 이 가림을 스스로 만들어낼 위험이 있습니다. 그래서 pair를 아직 못 찾은 "탐색" 단계(`SEARCH_COIL`)와, pair가 안정적으로 보여서 미세 보정만 하면 되는 "정밀 정합" 단계(`ALIGN_COIL`)를 상태로 분리했습니다.

- **`SEARCH_COIL`**: 회전은 하지 않고, **직선 전진 버스트(`search.burst_duration_sec`, 기본 0.6초) → 정지 후 확인(`search.pause_duration_sec`, 기본 0.6초)** 을 반복합니다. 태그 판단은 정지 구간에서만 하므로, 이동 중 카메라가 가려지거나 흔들려도 오판정으로 이어지지 않습니다. pair 중 한쪽 마커만 보이면 더 전진하지 않고 그 자리에서 대기합니다(지나치거나 밀어붙이지 않음). 안전을 위해 `search.max_duration_sec`(기본 20초)을 넘기면 `ERROR`로 정지합니다.
- **`ALIGN_COIL`**: pair가 보이는 동안은 기존과 동일하게 픽셀 오차 기반 미세 보정(각속도 상한이 `coil_max_angular` 기본 0.08 rad/s ≈ 4.6°/s로 작아 큰 회전은 애초에 명령하지 않음). **pair가 갑자기 사라지면**(회전 중 바퀴에 가려진 경우 등), 곧바로 정지하거나 계속 같은 방향으로 도는 대신, **직전에 낸 명령을 그대로 반대로 짧게(`search.recover_reverse_duration_sec`, 기본 0.4초) 되돌리는 back-off**를 먼저 시도합니다 — 가림을 유발했을 가능성이 높은 마지막 움직임을 취소하는 셈입니다. back-off 후에도 pair가 안 보이면 `SEARCH_COIL`로 돌아가 처음부터 다시 직선 탐색합니다.

## 2. 상태별 판단 및 구동

| 상태 | 판단 기준 | 구동 명령 |
|---|---|---|
| `SEARCH_HEAD_TAG` | 목표 선반의 head tag 인식 여부 (레거시 모드에서만) | 안 보이면 복도 저속 전진 |
| `APPROACH_SHELF` | head tag 이미지 오차 | `/cmd_vel`로 접근 보정 |
| `ENTER_SHELF` | 목표 페어(예: west+east) 인식 여부 (레거시 모드에서만) | 둘 다 안 보이면 저속 전진, 한쪽만 보이면 정지 대기, 둘 다 보이면 `SEARCH_COIL`로 전환 |
| `SEARCH_COIL` | 목표 페어 인식 여부 | 회전 없이 직선 버스트→정지 확인 반복, 한쪽만 보이면 대기, pair 찾으면 `ALIGN_COIL`로 전환 |
| `ALIGN_COIL` | 페어 중점(midpoint)/페어 각도(pair angle) 오차 | 보이면 `/cmd_vel`로 미세 보정, 갑자기 사라지면 마지막 명령을 짧게 반대로(back-off) 후 `SEARCH_COIL`로 복귀 |
| `FINAL_STOP` | N프레임 연속 정합 완료 | 정지 명령 |
| `CHARGING` | 정지 상태 유지 | WPT 충전 인터페이스 ON |

## 3. 오차 계산

두 태그(예: west/east)의 중점과, 두 태그 중심을 잇는 각도를 기준으로 오차를 계산합니다.

```text
pair_midpoint_x = (tag_a_x + tag_b_x) / 2
pair_midpoint_y = (tag_a_y + tag_b_y) / 2
pair_angle = atan2(tag_b_y - tag_a_y, tag_b_x - tag_a_x)

x_error = pair_midpoint_x - target_x
y_error = pair_midpoint_y - target_y
angle_error = normalize(pair_angle - target_angle)
```

최종 WPT 정합에는 west/east pair를 기본으로 사용합니다(`coil_1`이면 실제 마커 ID `13`/`14`). 90도 회전 후 방향 검증에는 north/south pair(`rotation_verify_pair`)를 사용합니다(`coil_1`이면 마커 `11`/`12`).

두 tag가 모두 보이지 않거나 pair 조건이 깨지면 `stable_count`를 0으로 되돌리고 정지합니다.

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

초기 추천값은 x/y 3~7 px, angle 1~3 deg, stable 10 frame입니다.

## 6. 터미널 단독 정합 판정 (`check_camera_alignment.py`)

ROS2/브링업 없이 카메라만으로 위 오차 계산과 동일한 로직을 한 번 또는 여러 프레임 실행해 로그로 출력하는 진단 스크립트입니다. `/cmd_vel`을 publish하지 않으므로 로봇이 움직이지 않습니다.

- `--cross-camera-only`: 카메라별 상세 로그 없이 크로스카메라(예: 오른쪽 카메라의 west, 왼쪽 카메라의 east) pair 상태만 요약 출력
- `--require-center-cell`: pair의 두 마커가 각 카메라 화면의 3x3 격자 중앙 칸에 들어와야 정합으로 판단 (`camera_grid_alignment.py`와 같은 방식의 보조 조건)
- `--require-front-cell`: 전방 카메라에서 특정 마커(`--front-tag-id`)가 특정 격자 칸(`--front-cell`)에 들어와야 함을 추가 조건으로 요구
- `--frames N --log-file out.csv`: N프레임을 캡처하며 크로스카메라 상태를 CSV로 기록 (오프라인 분석/튜닝용)
- `--no-save`가 없으면 각 카메라 프레임에 인식된 마커와 ALIGNED/NOT ALIGNED 텍스트를 표시해 `camera_alignment_check/`에 저장

## 7. 안전 조건

1. tag가 `safety.tag_lost_timeout_sec`(기본 1초) 이상 완전히 안 보이면 즉시 정지합니다. `SEARCH_HEAD_TAG`/`SEARCH_COIL`/`CHARGING` 상태는 의도적으로 태그 없이 동작하는 구간이라 이 타임아웃에서 예외입니다 — 대신 `SEARCH_COIL`은 `search.max_duration_sec`(기본 20초)로 별도 상한을 둡니다.
2. 실제 publish 전에는 `dry_run: true`로 확인합니다.
3. 정밀 정합 속도는 `0.005~0.02 m/s` 범위에서 시작합니다.
4. WPT 송신부는 `FINAL_STOP` 이후에만 켭니다.
