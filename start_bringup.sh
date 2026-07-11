#!/usr/bin/env bash
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
set -eo pipefail
exec ros2 launch turtlebot3_bringup robot.launch.py
