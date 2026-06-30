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

```text
복도 주행
→ 전방 카메라로 head AprilTag 탐색
→ 목표 선반 ID 판단
→ 선반 앞 접근 및 진입
→ 하부/측면 카메라 2개로 coil alignment tag 탐색
→ tag 중심 좌표/각도 오차 계산
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
