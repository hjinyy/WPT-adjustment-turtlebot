#!/usr/bin/env bash
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT/../../install/setup.bash"
set -eo pipefail

TARGET_COIL="${1:-coil_3}"
DRY_RUN="${WPT_DRY_RUN:-false}"
if [[ ! "$TARGET_COIL" =~ ^coil_[1-4]$ ]]; then
  echo "사용법: ./start_alignment.sh coil_1|coil_2|coil_3|coil_4" >&2
  exit 2
fi

setsid ros2 run wpt_adjustment_turtlebot global_map_navigation --ros-args \
  -p config_file:="$ROOT/config/wpt_alignment.yaml" \
  -p target_coil:="$TARGET_COIL" \
  -p log_root:="$ROOT/logs" \
  -p dry_run:="$DRY_RUN" &
PID=$!
CLEANING=0

cleanup() {
  [[ "$CLEANING" -eq 1 ]] && return
  CLEANING=1
  trap - INT TERM EXIT
  kill -TERM -- "-$PID" >/dev/null 2>&1 || true
  for _ in $(seq 1 20); do
    kill -0 -- "-$PID" >/dev/null 2>&1 || break
    sleep 0.1
  done
  kill -KILL -- "-$PID" >/dev/null 2>&1 || true
  timeout 6 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true
  wait "$PID" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

for _ in $(seq 1 50); do
  ros2 service list 2>/dev/null | grep -qx '/wpt_alignment/start' && break
  sleep 0.1
done
read -r -p "$TARGET_COIL 이동을 시작하려면 Enter를 누르세요. "
ros2 service call /wpt_alignment/start std_srvs/srv/Trigger '{}'
wait "$PID"