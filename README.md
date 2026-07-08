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

## 라즈베리파이 접속 및 카메라 실험

로봇에 올라간 라즈베리파이에서 실제 카메라 3대로 태그 인식/정합 실험을 할 수 있습니다.

```bash
ssh super2gl@192.168.0.5
# 비밀번호: 1234
```

> 이 저장소는 public이라 이 비밀번호도 그대로 노출됩니다. 다른 곳에 같은 비밀번호를 쓰고 있다면 바꾸는 걸 권장합니다.

### 카메라 포트 매핑

USB 카메라 3대가 아래처럼 연결되어 있습니다 (`/dev/video<번호>` 기준):

| 방향 | 포트(장치 번호) |
|---|---:|
| 정면 (front) | 0 |
| 오른쪽 (right) | 2 |
| 왼쪽 (left) | 4 |

(`/dev/video1`, `3`, `5`는 같은 카메라들의 메타데이터 노드라 캡처용으로 쓰지 않습니다.) 접속 후 아래로 장치가 3개 다 잡히는지 먼저 확인하세요.

```bash
ls -l /dev/video0 /dev/video2 /dev/video4
who   # 다른 세션이 카메라를 이미 쓰고 있는지 확인 (V4L2 장치는 동시에 못 엽니다)
```

라즈베리파이의 OpenCV(4.5.4)는 최신 `cv2.aruco.ArucoDetector` 클래스가 없어서, 구버전 `cv2.aruco.detectMarkers` API로 자동 대체되도록 코드에 이미 반영했습니다. 별도 설치 없이 바로 동작하지만, 인식 정확도를 높이고 싶다면 다음을 설치하세요.

```bash
pip3 install pupil-apriltags
```

### 3x3 그리드 정합 실험

각 카메라 화면을 3x3 격자로 나누고, 정면/오른쪽/왼쪽 카메라 모두에서 태그가 동시에 목표 칸(기본값: 중앙 (2,2))에 들어오면 "정합"으로 판단합니다. 목표 칸은 실제 실험 결과에 따라 바뀔 수 있으므로 `--target-row`/`--target-col`로 조정할 수 있게 만들어 두었습니다.

```bash
cd ~/wpt_ws/src/WPT-adjustment-turtlebot   # colcon build로 패키지가 설치되어 있어야 import가 됩니다
python3 scripts/camera_grid_alignment.py
# 특정 칸을 목표로 바꾸고 싶을 때
python3 scripts/camera_grid_alignment.py --target-row 2 --target-col 2
```

터미널에는 매 프레임 아래처럼 상태가 출력됩니다.

```text
target=(2, 2) front: id=None cell=None - | right: id=14 cell=(2, 2) ALIGNED | left: id=13 cell=(1, 2) - -> ALL_ALIGNED=False
```

화면으로 직접 보고 싶으면 `--show` 옵션을 추가하세요. 단, `cv2.imshow`는 GUI가 있어야 동작하므로 순수 SSH 터미널 세션만으로는 뜨지 않습니다. 아래 둘 중 하나가 필요합니다.

1. 라즈베리파이에 모니터를 연결하고 데스크톱 환경에서 직접 실행
2. Mac/Windows에서 X11 forwarding으로 접속: `ssh -X super2gl@192.168.0.5` (Mac은 XQuartz, Windows는 VcXsrv/MobaXterm 등 X서버가 미리 떠 있어야 합니다) 후 동일하게 `--show` 옵션으로 실행

```bash
python3 scripts/camera_grid_alignment.py --show
# 창에서 q 키로 종료
```

## 문서

- `docs/apriltag_plan.md`: AprilTag ID/제작 계획
- `docs/algorithm.md`: 판단 및 구동 알고리즘
- `docs/calibration.md`: 카메라별 target 좌표 캘리브레이션 절차
