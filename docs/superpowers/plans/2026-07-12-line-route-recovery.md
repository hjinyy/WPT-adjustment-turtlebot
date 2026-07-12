# 라인 중심 주행 및 회전 복구 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**목표:** 전방 카메라 상단 중앙 ROI의 검정 테이프 중심을 따라 주행하고, AprilTag 소실 중에도 일관된 방향으로 회전·재탐색하며 `43 -> 33` 마커 게이트로 코일 4에서 코일 3까지 안전하게 이동한다.

**구조:** 라인 모듈은 ROI의 세로 테이프만 직접 PD 조향으로 사용하며 칼만 필터를 사용하지 않는다. 전역 EKF는 전체 화면 AprilTag 관측과 `/odom` 예측만 담당하고, 상태기계가 출발 마커 확인, 0.5초 유예, 재탐색, 목표 마커 확인을 제어한다.

**기술:** Python 3.10, ROS 2 Humble, OpenCV, pupil-apriltags/OpenCV ArUco, pytest, YAML.

## 전역 제약

- 모든 문서와 사용자 로그 메시지는 한국어로 작성한다.
- 라인 ROI 초기값은 세로 `0~55%`, 가로 `20~80%`이며 라인 모듈에만 적용한다.
- AprilTag 전역 위치추정과 최종 정합은 전체 프레임을 사용한다.
- 라인에는 칼만 필터를 쓰지 않는다. EKF는 AprilTag와 `/odom`, 기존 칼만 필터는 최종 코일 정합에만 쓴다.
- 라인과 마커가 동시에 소실되었을 때 전진 유예 시간은 정확히 `0.5초`다.
- `coil_4 -> coil_3`은 `43`이 화면 중앙에 확인되기 전 선속도 0, `33` 검출 후 최종 정합으로 전환한다.
- COMPLETE, ERROR, stop, Ctrl+C는 항상 0 속도를 발행한다.
- 실제 모터 주행 전 pytest, dry-run 30초, 상단 중앙 ROI 프레임 확인을 수행한다.

---

### Task 1: 바퀴를 제외하는 라인 ROI 검출기

**Files:**
- Modify: `wpt_adjustment_turtlebot/line_detection.py`
- Modify: `config/wpt_alignment.yaml`
- Modify: `test/test_line_detection.py`

**Interfaces:**
- Produces: `LineDetector.detect(frame_bgr) -> LineObservation | None`
- Adds constructor inputs: `roi_left_ratio`, `roi_right_ratio`, `max_abs_angle_deg`, `min_vertical_span_ratio`.
- `LineObservation.center_x`는 원본 프레임 기준 x 좌표다.

- [ ] **Step 1: 실패하는 바퀴 거부 및 중앙 ROI 테스트 작성**

```python
def test_rejects_large_horizontal_robot_wheel_as_line():
    frame = _blank_frame()
    cv2.rectangle(frame, (10, 150), (310, 235), (0, 0, 0), -1)
    detector = LineDetector(
        threshold=80, min_area_px=100,
        roi_top_ratio=0.0, roi_bottom_ratio=0.55,
        roi_left_ratio=0.20, roi_right_ratio=0.80,
        max_abs_angle_deg=45.0, min_vertical_span_ratio=0.20,
    )
    assert detector.detect(frame) is None


def test_reports_original_frame_center_for_line_inside_central_roi():
    frame = _blank_frame()
    cv2.line(frame, (160, 10), (160, 125), (0, 0, 0), 12)
    detector = LineDetector(...same configuration...)
    observation = detector.detect(frame)
    assert observation is not None
    assert observation.center_x == pytest.approx(160, abs=2)
    assert observation.angle_error_deg == pytest.approx(0, abs=3)
```

- [ ] **Step 2: 테스트가 실패하는지 확인**

Run: `pytest test/test_line_detection.py -q`

Expected: 새 생성자 인자가 없거나 큰 가로 윤곽이 반환되어 FAIL.

- [ ] **Step 3: ROI와 세로 후보 선택 구현**

```python
def _candidate(contour, *, roi_height: int) -> tuple[float, float, float] | None:
    area = float(cv2.contourArea(contour))
    x, y, width, height = cv2.boundingRect(contour)
    if height / max(1, roi_height) < self.min_vertical_span_ratio:
        return None
    vx, vy, _, _ = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
    angle = -degrees(atan2(float(vx), float(vy)))
    if abs(angle) > self.max_abs_angle_deg:
        return None
    return area, angle, float(x + width / 2.0)
```

ROI 좌표를 `(x0:x1, y0:y1)`로 자르고, 조건을 통과한 후보 중 가장 큰 윤곽 하나만 사용한다. 모멘트 좌표에 `x0`, `y0`를 더해 원본 프레임 좌표로 복원한다.

- [ ] **Step 4: YAML 기본값 연결**

```yaml
line_tracking:
  roi_top_ratio: 0.0
  roi_bottom_ratio: 0.55
  roi_left_ratio: 0.20
  roi_right_ratio: 0.80
  max_abs_angle_deg: 45.0
  min_vertical_span_ratio: 0.20
```

`GlobalMapNavigator`와 기존 `WptAlignmentNode`의 `LineDetector` 생성 시 새 설정을 전달한다.

- [ ] **Step 5: 단위 테스트 확인**

Run: `pytest test/test_line_detection.py -q`

Expected: PASS.

### Task 2: 전역 방향과 출발·목표 마커 게이트

**Files:**
- Modify: `wpt_adjustment_turtlebot/global_map.py`
- Modify: `wpt_adjustment_turtlebot/global_route_control.py`
- Modify: `test/test_global_route_control.py`

**Interfaces:**
- Produces: `expected_route_marker_ids(start_coil: str, target_coil: str) -> tuple[int, int]`
- Produces: `MarkerRouteGuide(departure_marker_id, goal_marker_id, image_width, center_tolerance_px)`.
- `MarkerRouteGuide.update_departure(tag_centers) -> bool`, `goal_visible(tag_ids) -> bool`, `rotation_sign: float`.

- [ ] **Step 1: 실패하는 경로 마커·회전 부호 테스트 작성**

```python
def test_coil_4_to_3_uses_west_departure_and_goal_markers():
    assert expected_route_marker_ids("coil_4", "coil_3") == (43, 33)


def test_marker_guide_keeps_turn_direction_when_observation_disappears():
    guide = MarkerRouteGuide(43, 33, image_width=320, center_tolerance_px=45)
    assert not guide.update_departure([(41, 160.0)])
    assert guide.rotation_sign == 1.0
    assert not guide.update_departure([])
    assert guide.rotation_sign == 1.0


def test_marker_guide_allows_departure_only_for_centered_43():
    guide = MarkerRouteGuide(43, 33, image_width=320, center_tolerance_px=45)
    assert not guide.update_departure([(43, 80.0)])
    assert guide.update_departure([(43, 158.0)])
    assert guide.goal_visible([33])
```

- [ ] **Step 2: 테스트가 실패하는지 확인**

Run: `pytest test/test_global_route_control.py -q`

Expected: `expected_route_marker_ids` 또는 `MarkerRouteGuide` import 실패.

- [ ] **Step 3: 방향 suffix와 게이트 구현**

```python
def expected_route_marker_ids(start_coil: str, target_coil: str) -> tuple[int, int]:
    start_x, start_y = COIL_CENTERS_M[start_coil]
    target_x, target_y = COIL_CENTERS_M[target_coil]
    if abs(target_x - start_x) >= abs(target_y - start_y):
        suffix = 4 if target_x > start_x else 3
    else:
        suffix = 1 if target_y > start_y else 2
    return int(start_coil[-1]) * 10 + suffix, int(target_coil[-1]) * 10 + suffix
```

`MarkerRouteGuide`는 현재 코일 마커의 suffix로 북/동/남/서 방위를 얻고 목표 suffix와의 회전 부호를 정한다. 정확히 180도인 경우에는 `+1.0`을 선택해 결정론적으로 반시계 회전을 유지한다. 빈 관측은 기존 `rotation_sign`을 바꾸지 않는다.

- [ ] **Step 4: 단위 테스트 확인**

Run: `pytest test/test_global_route_control.py -q`

Expected: PASS.

### Task 3: 마커 소실에도 지속되는 회전과 0.5초 라인 유예 상태기계

**Files:**
- Modify: `wpt_adjustment_turtlebot/global_map_navigation.py`
- Modify: `config/wpt_alignment.yaml`
- Create: `test/test_navigation_recovery.py`

**Interfaces:**
- Consumes: `MarkerRouteGuide`, `RouteFollower`, `MapPoseEKF`, `LineObservation`.
- Adds states: `ACQUIRE_ROUTE`, `FOLLOW_LINE`, `REACQUIRE`, `APPROACH_TARGET`.
- Adds pure helper: `NavigationRecoveryPolicy(grace_sec: float)` with `line_or_marker_lost(now) -> bool` and `should_reacquire(now) -> bool`.

- [ ] **Step 1: 실패하는 회전·유예·목표 전환 테스트 작성**

```python
def test_recovery_policy_allows_only_half_second_without_line_or_marker():
    policy = NavigationRecoveryPolicy(grace_sec=0.5)
    policy.observe(now=10.0, line_seen=True, marker_seen=False)
    assert not policy.should_reacquire(now=10.49)
    assert policy.should_reacquire(now=10.51)


def test_acquire_route_keeps_last_rotation_sign_when_tags_disappear():
    state = RouteAcquisitionState(rotation_sign=-1.0)
    assert state.rotation_command() == pytest.approx(-0.18)
    state.observe_tags([])
    assert state.rotation_command() == pytest.approx(-0.18)


def test_goal_marker_transitions_to_final_alignment():
    state = NavigationStateMachine(... goal_marker_id=33 ...)
    assert state.step(tag_ids=[33], line_seen=True) == "ALIGN_COIL"
```

- [ ] **Step 2: 테스트가 실패하는지 확인**

Run: `pytest test/test_navigation_recovery.py -q`

Expected: import 실패.

- [ ] **Step 3: 작은 순수 상태 보조 객체 구현**

`global_route_control.py`에 ROS와 OpenCV 의존성이 없는 `NavigationRecoveryPolicy`와 `RouteAcquisitionState`를 추가한다. 이 객체들은 마지막 라인/마커 관측 시각, 마지막 회전 부호, 0.5초 유예 판정만 가진다. ROS 노드는 이 객체를 호출할 뿐 상태 판단을 중복하지 않는다.

```python
class NavigationRecoveryPolicy:
    def __init__(self, grace_sec: float = 0.5) -> None:
        self.grace_sec = float(grace_sec)
        self.last_seen_time: float | None = None

    def observe(self, *, now: float, line_seen: bool, marker_seen: bool) -> None:
        if line_seen or marker_seen:
            self.last_seen_time = float(now)

    def should_reacquire(self, *, now: float) -> bool:
        return self.last_seen_time is not None and now - self.last_seen_time > self.grace_sec
```

- [ ] **Step 4: `GlobalMapNavigator.step()`에 상태기계 연결**

`capture_observation()`은 전체 프레임 AprilTag 측정, 라인 ROI 관측, 각 처리 시간을 분리해 반환한다. 라인 각속도는 `ErrorKalmanFilter`를 거치지 않고 아래 식으로 계산한다.

```python
line_angular = clamp(
    line_kp_x * line.x_error + line_kp_angle * line.angle_error_deg,
    line_max_angular,
)
```

`ACQUIRE_ROUTE`에서는 출발 마커가 중앙에 오기 전 `VelocityCommand(linear_x=0.0, angular_z=rotation_sign * route_max_angular)`만 발행한다. AprilTag가 없을 때도 `/odom.angular.z` 또는 마지막 회전 명령으로 EKF를 계속 예측하며 회전 부호를 바꾸지 않는다.

`FOLLOW_LINE`에서는 라인 또는 마커가 보이는 동안 전진한다. 둘 다 소실된 뒤 0.5초 내에는 마지막 선 조향으로 전진하고, 초과하면 `REACQUIRE`에서 `linear_x=0.0` 및 마지막 조향 부호의 회전 명령을 발행한다. `33`이 보이면 `ALIGN_COIL`로 전환한다.

- [ ] **Step 5: YAML 정책값 추가**

```yaml
map_localization:
  route_marker_center_tolerance_px: 45
  acquire_timeout_sec: 12.0
  reacquire_timeout_sec: 12.0
  route_loss_grace_sec: 0.5
  reacquire_angular: 0.10
control:
  line:
    k_x_to_angular: -0.0020
    k_angle_to_angular: -0.0030
```

- [ ] **Step 6: 상태 보조 객체 테스트 확인**

Run: `pytest test/test_navigation_recovery.py test/test_global_route_control.py -q`

Expected: PASS.

### Task 4: 성능·관측 진단 로그

**Files:**
- Modify: `wpt_adjustment_turtlebot/run_logging.py`
- Modify: `wpt_adjustment_turtlebot/global_map_navigation.py`
- Modify: `test/test_run_logging.py`
- Modify: `test/test_navigation_logging_contract.py`

**Interfaces:**
- Adds telemetry columns: `frame_capture_ms`, `tag_detect_ms`, `line_detect_ms`, `control_cycle_ms`, `control_overrun`.
- Adds `RunLogger.performance_summary() -> dict[str, float]` with p50/p95/p99.

- [ ] **Step 1: 실패하는 성능 열 및 분위수 테스트 작성**

```python
def test_run_logger_writes_performance_columns(tmp_path):
    logger = RunLogger(tmp_path, target_coil="coil_3")
    logger.telemetry(..., frame_capture_ms=8.0, tag_detect_ms=32.0,
                     line_detect_ms=2.0, control_cycle_ms=45.0,
                     control_overrun=False)
    logger.close()
    header = (logger.run_dir / "telemetry.csv").read_text().splitlines()[0]
    assert "control_cycle_ms" in header


def test_performance_summary_reports_p95(tmp_path):
    logger = RunLogger(tmp_path, target_coil="coil_3")
    for value in (10.0, 20.0, 30.0, 40.0, 50.0):
        logger.record_cycle_ms(value)
    assert logger.performance_summary()["p95_ms"] >= 40.0
```

- [ ] **Step 2: 테스트가 실패하는지 확인**

Run: `pytest test/test_run_logging.py test/test_navigation_logging_contract.py -q`

Expected: 새 열 또는 메서드 부재로 FAIL.

- [ ] **Step 3: 타이밍 계측 구현**

`time.perf_counter()`로 프레임 취득, AprilTag, 라인, `step()` 전체 시간을 각각 측정한다. 10Hz 설정 주기의 100ms를 넘으면 `control_overrun=True`를 기록한다. 종료 시 events.log에 p50/p95/p99과 초과 프레임 수를 한국어로 기록한다.

- [ ] **Step 4: 테스트 확인**

Run: `pytest test/test_run_logging.py test/test_navigation_logging_contract.py -q`

Expected: PASS.

### Task 5: 전체 회귀와 라즈베리파이 dry-run 검증

**Files:**
- Modify: `README.md` (실행·로그 확인 절차가 이미 있는 경우에만 최소 변경)
- Modify: `docs/algorithm.md` (상태기계 설명이 있는 경우에만 최소 변경)

- [ ] **Step 1: 전체 로컬 회귀 실행**

Run: `pytest -q`

Expected: 기존 최종 정합·전역 지도 테스트를 포함해 PASS.

- [ ] **Step 2: 정적 검증**

Run: `python -m compileall wpt_adjustment_turtlebot && git diff --check`

Expected: Python 컴파일 성공, 공백 오류 없음.

- [ ] **Step 3: 파이에 안전하게 업로드·빌드**

Run on Pi:

```bash
cd ~/wpt_ws/src/WPT-adjustment-turtlebot
git pull --ff-only origin gyoenx2
cd ~/wpt_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select wpt_adjustment_turtlebot
source install/setup.bash
```

Expected: `Finished <<< wpt_adjustment_turtlebot`.

- [ ] **Step 4: 30초 비구동 성능 검증**

Run on Pi terminal 1:

```bash
cd ~/wpt_ws/src/WPT-adjustment-turtlebot
WPT_DRY_RUN=true ./start_alignment.sh coil_3
```

Enter로 시작한 뒤 30초 관찰하고 Ctrl+C로 종료한다. 최신 `logs/*/telemetry.csv`에서 `control_cycle_ms`의 p95/p99, `control_overrun`, 라인 ROI 관측을 확인한다.

Expected: p95 <= 100ms, p99 <= 150ms. 프레임에 테이프가 있을 때만 중앙 세로 후보가 선택되고 바퀴는 선택되지 않는다.

- [ ] **Step 5: 집중 커밋과 푸시**

```bash
git add config/wpt_alignment.yaml wpt_adjustment_turtlebot test docs README.md
git commit -m "fix: recover route guidance and isolate line tracking"
git push origin gyoenx2
```

ZIP 파일, 과거 백업 디렉터리, 임시 카메라 프레임은 스테이징하지 않는다.
