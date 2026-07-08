# WPT Adjustment TurtleBot

AprilTag와 ROS2 `/cmd_vel`을 이용해 TurtleBot의 WPT 수신 코일을 선반/스테이지의 송신 코일에 자동 정합시키기 위한 예제 패키지입니다.

## 제공 범위

1. 4개 WPT 코일용 16개 AprilTag ID 규칙
2. 전방 카메라 1대 + 하부/측면 카메라 2대 구조
3. ROS2 Python 상태 머신 노드
4. `/cmd_vel` 기반 저속 전진/후진/회전 보정 알고리즘
5. N프레임 연속 정합 완료 판단
6. tag lost / timeout 안전 정지
7. YAML 기반 카메라별 캘리브레이션 템플릿
8. 순수 Python 단위 테스트

## 전체 흐름

```text
목표 코일 선택 (`coil_1`~`coil_4`)
→ 하부/측면 카메라로 coil tag pair 탐색
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
  -p target_coil:=coil_1 \
  -p dry_run:=true
```

처음에는 `config/wpt_alignment.yaml`의 `dry_run: true` 상태로 로그만 확인하세요. 실제 구동 전 카메라 번호, `/cmd_vel` 토픽명, 제어 부호를 반드시 확인해야 합니다.

## 3-Camera Dry-Run Check

ROS2 노드를 실행하기 전에 카메라 3대가 tag를 인식하는지, 목표 coil pair가 정합 범위에 들어왔는지 확인할 수 있습니다. 이 스크립트는 `/cmd_vel`을 publish하지 않습니다.

```bash
python3 scripts/check_camera_alignment.py \
  --target-coil coil_1 \
  --pair west_east \
  --frames 10 \
  --output-dir camera_alignment_check
```

현재 기본 카메라 번호는 `front=/dev/video4`, `right_bottom=/dev/video2`, `left_bottom=/dev/video0`입니다. 3개 카메라를 동시에 안정적으로 읽기 위해 기본 해상도는 `320x240`, `10fps`, `MJPG`로 낮춰 둡니다. 출력에는 각 카메라의 detected tag ID, `coil_1 west/east` 같은 tag 의미, pair midpoint, pair angle, x/y/angle error, `aligned=True/False`가 표시됩니다. 목표 pair 중 하나만 보이면 `missing_marker`로 표시되고 정합은 `False`입니다.

## Four Coil Marker IDs

현재 구조는 복도 주행 없이 4개 코일 사이에서 정합하는 단순 layout입니다. 각 코일은 north/east/south/west 4개 marker를 가지며 총 16개 marker를 사용합니다.

| Coil | Shelf 위치 | North | East | South | West |
|---|---|---:|---:|---:|---:|
| `coil_1` | (1,1) | 11 | 14 | 12 | 13 |
| `coil_2` | (1,2) | 21 | 24 | 22 | 23 |
| `coil_3` | (2,1) | 31 | 34 | 32 | 33 |
| `coil_4` | (2,2) | 41 | 44 | 42 | 43 |

WPT 최종 정합은 기본적으로 West/East pair를 사용합니다. 예를 들어 `coil_1`의 West/East pair는 marker `13`과 marker `14`입니다. 90도 회전 후 방향 검증에는 North/South pair를 쓰며, `coil_1`에서는 marker `11`과 marker `12`입니다.

## 문서

- `docs/apriltag_plan.md`: AprilTag ID/제작 계획
- `docs/algorithm.md`: 판단 및 구동 알고리즘
- `docs/calibration.md`: 카메라별 target 좌표 캘리브레이션 절차
