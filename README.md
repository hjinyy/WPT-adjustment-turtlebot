# WPT Adjustment TurtleBot

AprilTag를 이용해 TurtleBot3의 WPT 수신 코일을 2x2 배치된 4개 선반(코일)의 송신 코일에 정합시키기 위한 실험용 패키지입니다.

로봇은 라즈베리파이(TurtleBot3, ROS2 Humble + OpenCR)에 SSH로 직접 접속해서 스크립트를 그대로 실행하는 방식으로 다룹니다. colcon으로 이 저장소를 빌드/설치할 필요는 없고, ROS2 환경만 `source` 하면 됩니다. (colcon 빌드가 필요한 ROS2 노드 버전은 선택 사항으로 맨 아래에 남겨뒀습니다.)

## 제공 범위

1. AprilTag ID 규칙 및 출력용 태그 시트 생성 도구 (`scripts/generate_tag_sheet.py`)
2. 전방(0)/오른쪽(2)/왼쪽(4) USB 카메라 3대 구조
3. 카메라 실시간 화면 확인 스크립트 (`scripts/camera_live_view.py`)
4. 3x3 그리드 기반 코일 정합 판단 스크립트 (`scripts/camera_grid_alignment.py`)
5. 2x2 코일 스테이지 물리 배치 + 오픈루프(dead-reckoning) 선반 간 이동 스크립트 (`scripts/drive_to_shelf.py`)
6. 순수 Python 단위 테스트
7. (선택) ROS2 패키지로 빌드해서 쓰는 정밀 pair 정합 노드

## 선반/코일 배치

선반 배치는 2x2(shelf 1~4)이며, coil alignment tag 16개(11~14, 21~24, 31~34, 41~44)만 사용합니다. head tag는 쓰지 않습니다 — 인쇄된 태그 모양과 ID 생성 규칙은 `docs/apriltag_plan.md` 참고.

```text
shelf 1   shelf 2
shelf 3   shelf 4
```

물리 치수는 `wpt_adjustment_turtlebot/shelf_layout.py`에 상수로 정의되어 있습니다.

| 값 | 현재 값 | 비고 |
|---|---:|---|
| 스테이지 크기 | 80cm x 60cm | 대략치 |
| 코일 간 가로 간격 (1-2, 3-4) | 45cm | 대략치 |
| 코일 간 세로 간격 (1-3, 2-4) | 30cm | 대략치 |

**정확한 치수는 아직 실측 전입니다.** 나중에 다시 재서 `shelf_layout.py`의 `STAGE_WIDTH_M`/`STAGE_HEIGHT_M`/`COIL_SPACING_X_M`/`COIL_SPACING_Y_M` 네 상수만 바꿔주면 됩니다. 다른 코드는 손댈 필요 없습니다.

## 정합 판단 방식: 3x3 카메라 그리드

로봇이 선반에 어느 방향으로 들어올지 미리 알 수 없기 때문에, 정밀한 픽셀 좌표 대신 각 카메라 화면을 3x3 격자로 나누고 아래 조건으로 "정합"을 판단합니다.

- 정면/오른쪽/왼쪽 카메라 각각에서 태그를 인식
- 세 카메라 **모두** 태그 중심이 중앙 칸 (2,2)에 들어와 있으면 정합으로 판단

목표 칸은 지금은 중앙 (2,2)로 가정한 것이고, 실제로 로봇을 움직여보면서 바뀔 수 있습니다. `scripts/camera_grid_alignment.py`를 실행할 때 `--target-row`/`--target-col`로 바꿀 수 있습니다.

## 라즈베리파이 접속

```bash
ssh super2gl@192.168.0.5
# 비밀번호: 1234
```

> 이 저장소는 public이라 이 비밀번호도 그대로 노출됩니다. 다른 곳에 같은 비밀번호를 쓰고 있다면 바꾸는 걸 권장합니다.

접속 후 이 저장소를 홈 디렉토리에 클론해두고 실험을 진행하세요.

```bash
git clone https://github.com/hjinyy/WPT-adjustment-turtlebot.git
cd WPT-adjustment-turtlebot
```

Python 패키지가 임포트되도록(`import wpt_adjustment_turtlebot...`) 아래 스크립트들은 모두 저장소 루트(`WPT-adjustment-turtlebot/`)에서 실행하세요.

### 카메라 포트 확인

USB 카메라 3대가 아래처럼 연결되어 있습니다 (`/dev/video<번호>` 기준):

| 방향 | 포트(장치 번호) |
|---|---:|
| 정면 (front) | 0 |
| 오른쪽 (right) | 2 |
| 왼쪽 (left) | 4 |

(`/dev/video1`, `3`, `5`는 같은 카메라들의 메타데이터 노드라 캡처용으로 쓰지 않습니다.) 접속 후 아래로 장치가 3개 다 잡히는지, 다른 세션이 이미 카메라를 쓰고 있지 않은지부터 확인하세요.

```bash
ls -l /dev/video0 /dev/video2 /dev/video4
who   # 다른 세션이 접속해 있는지 확인 (V4L2 장치는 동시에 못 엽니다)
```

라즈베리파이의 OpenCV(4.5.4)는 최신 `cv2.aruco.ArucoDetector` 클래스가 없어서, 구버전 `cv2.aruco.detectMarkers` API로 자동 대체되도록 코드에 이미 반영했습니다. 별도 설치 없이 바로 동작하지만, 인식 정확도를 높이고 싶다면 다음을 설치하세요.

```bash
pip3 install pupil-apriltags
```

### 카메라 실시간 화면 보기

태그 인식 없이 카메라가 제대로 잡히는지만 빠르게 볼 때 사용합니다.

```bash
python3 scripts/camera_live_view.py --device 0        # 카메라 하나만
python3 scripts/camera_live_view.py --all              # 정면/오른쪽/왼쪽 동시에
# 창에서 q 키로 종료
```

`cv2.imshow`는 GUI가 있어야 동작하므로 순수 SSH 터미널 세션만으로는 뜨지 않습니다. 아래 둘 중 하나가 필요합니다.

1. 라즈베리파이에 모니터를 연결하고 데스크톱 환경에서 직접 실행
2. Mac/Windows에서 X11 forwarding으로 접속: `ssh -X super2gl@192.168.0.5` (Mac은 XQuartz, Windows는 VcXsrv/MobaXterm 등 X서버가 미리 떠 있어야 합니다) 후 동일하게 실행

## 기본 주행: 선반 간 이동

TurtleBot3 브링업을 먼저 백그라운드로 띄웁니다 (모터 제어는 이 프로세스가 OpenCR 보드와 통신하며 처리합니다).

```bash
source /opt/ros/humble/setup.bash
ros2 launch turtlebot3_bringup robot.launch.py &
```

그 다음 목표 선반으로 이동합니다.

```bash
# 계획만 확인 (실제로 움직이지 않음)
python3 scripts/drive_to_shelf.py --shelf 3 --dry-run

# 선반 1에서 선반 3으로 실제 이동
python3 scripts/drive_to_shelf.py --shelf 3 --from-shelf 1
```

**주의**: 이 스크립트는 오픈루프(dead-reckoning)입니다. 오도메트리나 카메라 피드백 없이 `속도 x 시간`으로만 움직이므로, `--from-shelf`에 지정한 선반 위치에 로봇이 +x 방향(선반 배치 기준 오른쪽)을 보고 정확히 놓여 있다고 가정합니다. 대략 선반 근처까지 이동시키는 용도로만 쓰고, 마지막 정밀 정지는 아래 코일 정합 실험으로 확인하세요.

## 코일 정합 실험 절차

1. 위 "카메라 포트 확인"으로 카메라 3대가 잡히는지 확인
2. `scripts/drive_to_shelf.py`로 목표 선반 근처까지 이동 (또는 손으로 옮겨도 됩니다)
3. 아래로 3x3 그리드 정합 상태를 확인합니다.

   ```bash
   python3 scripts/camera_grid_alignment.py
   ```

   터미널에 매 프레임 아래처럼 출력됩니다.

   ```text
   target=(2, 2) front: id=None cell=None - | right: id=14 cell=(2, 2) ALIGNED | left: id=13 cell=(1, 2) - -> ALL_ALIGNED=False
   ```

4. `ALL_ALIGNED=True`가 나올 때까지 로봇을 조금씩 움직여 보면서, 어느 위치/방향에서 세 카메라가 동시에 목표 칸에 들어오는지 감을 잡습니다.
5. 목표 칸이 중앙 (2,2)가 아니라 다른 칸이 더 적합하다고 판단되면 조정해서 다시 확인합니다.

   ```bash
   python3 scripts/camera_grid_alignment.py --target-row 2 --target-col 1
   ```

6. 화면으로 직접 보면서 하고 싶다면 `--show`를 추가합니다 (GUI 필요, 위 "카메라 실시간 화면 보기" 참고).

   ```bash
   python3 scripts/camera_grid_alignment.py --show
   ```

7. 확정된 목표 칸과 실측한 스테이지/코일 간격을 각각 `camera_grid_alignment.py` 기본값과 `shelf_layout.py` 상수에 반영합니다.

## (선택) ROS2 패키지로 정밀 정합 실행

3x3 그리드보다 더 정밀한 픽셀 좌표 기반 정합(코일 태그 pair의 중점/각도로 `/cmd_vel`을 연속 보정)이 필요해지면, 이 저장소를 ROS2 패키지로 빌드해서 쓸 수 있습니다.

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

처음에는 `config/wpt_alignment.yaml`의 `dry_run: true` 상태로 로그만 확인하세요. 실제 구동 전 카메라 번호, `/cmd_vel` 토픽명, 제어 부호를 반드시 확인해야 합니다. 픽셀 target 좌표 캘리브레이션 절차는 `docs/calibration.md` 참고.

## 문서

- `docs/apriltag_plan.md`: AprilTag ID/제작 계획
- `docs/algorithm.md`: (선택 사항인 ROS2 노드의) 판단 및 구동 알고리즘
- `docs/calibration.md`: (선택 사항인 ROS2 노드의) 카메라별 target 좌표 캘리브레이션 절차
