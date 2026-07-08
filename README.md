# WPT Adjustment TurtleBot

AprilTag와 ROS2 `/cmd_vel`을 이용해 TurtleBot의 WPT 수신 코일을 선반/스테이지의 송신 코일에 자동 정합시키기 위한 예제 패키지입니다.

## 제공 범위

1. AprilTag ID 규칙 및 출력용 태그 시트 생성 도구
2. 전방 카메라 1대 + 하부/측면 카메라 2대 구조
3. ROS2 Python 상태 머신 노드
4. `/cmd_vel` 기반 저속 전진/후진/회전 보정 알고리즘
5. N프레임 연속 정합 완료 판단
6. tag lost / timeout 안전 정지
7. YAML 기반 카메라별 캘리브레이션 템플릿
8. 순수 Python 단위 테스트

## 전체 흐름

선반 배치는 2x2(shelf 1~4)이며, coil alignment tag 16개(11~14, 21~24, 31~34, 41~44)만 사용합니다. 로봇이 선반에 어느 방향으로 진입할지 알 수 없으므로 마주보는 두 태그(예: west+east)를 한 쌍으로 인식해 중점/각도로 정합합니다.

```text
복도 주행 (목표 선반의 coil tag pair 탐색)
→ 전방/하부/측면 카메라 중 pair가 보이는 카메라 선택
→ pair 중점 좌표 / pair 각도 오차 계산
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
  -p target_shelf:=1
```

처음에는 `config/wpt_alignment.yaml`의 `dry_run: true` 상태로 로그만 확인하세요. 실제 구동 전 카메라 번호, `/cmd_vel` 토픽명, 제어 부호를 반드시 확인해야 합니다.

## 문서

- `docs/apriltag_plan.md`: AprilTag ID/제작 계획
- `docs/algorithm.md`: 판단 및 구동 알고리즘
- `docs/calibration.md`: 카메라별 target 좌표 캘리브레이션 절차
