# WPT Adjustment TurtleBot

AprilTag와 ROS2 `/cmd_vel`을 이용해 TurtleBot의 WPT 수신 코일을 선반/스테이지의 송신 코일에 자동 정합시키기 위한 예제 패키지입니다.

## 제공 범위

1. `apriltag_sheet.pdf` 기반 station map ID 규칙
2. 전방 카메라 1대 + 하부/측면 카메라 2대 구조
3. ROS2 Python 상태 머신 노드
4. `/cmd_vel` 기반 저속 전진/후진/회전 보정 알고리즘
5. N프레임 연속 정합 완료 판단
6. tag lost / timeout 안전 정지
7. YAML 기반 카메라별 캘리브레이션 템플릿
8. 순수 Python 단위 테스트

## 전체 흐름

```text
station map의 목표 station 선택
→ 하부/측면 카메라로 station tag pair 탐색
→ pair midpoint 좌표/각도 오차 계산
→ ROS2 /cmd_vel로 저속 보정
→ N프레임 연속 정합 조건 만족
→ 정지
→ WPT 충전 시작
```

## ROS2 실행

```bash
mkdir -p ~/wpt_ws/src
cd ~/wpt_ws/src
git clone https://github.com/hjinyy/WPT-adjustment-turtlebot.git
cd ~/wpt_ws
colcon build --symlink-install
source install/setup.bash
ros2 run wpt_adjustment_turtlebot wpt_alignment_node \
  --ros-args \
  -p config_file:=$(pwd)/src/WPT-adjustment-turtlebot/config/wpt_alignment.yaml \
  -p target_station:=A02 \
  -p dry_run:=true
```

처음에는 `config/wpt_alignment.yaml`의 `dry_run: true` 상태로 로그만 확인하세요. 실제 구동 전 카메라 번호, `/cmd_vel` 토픽명, 제어 부호를 반드시 확인해야 합니다.

## Station Map Marker IDs

현재 물리 태그는 새로 생성한 `111/112/113/114` ID가 아니라 `apriltag_sheet.pdf`에 이미 인쇄된 숫자 ID를 그대로 사용합니다. 예를 들어 `A02N`은 사람이 읽는 label이고 detector가 반환하는 실제 numeric marker ID는 `5`입니다.

WPT 최종 정합은 기본적으로 West/East pair를 사용합니다. `A02`의 West/East pair는 marker `8`과 marker `6`입니다. 90도 회전 후 방향 검증에는 North/South pair를 쓰며, `A02`에서는 marker `5`와 marker `7`입니다.

## 문서

- `docs/apriltag_plan.md`: AprilTag ID/제작 계획
- `docs/algorithm.md`: 판단 및 구동 알고리즘
- `docs/calibration.md`: 카메라별 target 좌표 캘리브레이션 절차
