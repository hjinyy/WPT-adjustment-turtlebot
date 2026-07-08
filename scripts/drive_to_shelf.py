#!/usr/bin/env python3
"""Open-loop drive from one shelf position to another on the 2x2 coil stage.

Standalone script, no colcon package build needed. Run turtlebot3_bringup
first (in another terminal or background), then run this directly:

    source /opt/ros/humble/setup.bash
    ros2 launch turtlebot3_bringup robot.launch.py &
    python3 scripts/drive_to_shelf.py --shelf 3

This is dead-reckoning only (speed * time), not corrected by odometry or
vision — it assumes the robot starts at --from-shelf's position facing the
+x axis (see wpt_adjustment_turtlebot/shelf_layout.py for the coordinate
convention). Treat it as a rough first move to get near the target shelf;
use scripts/camera_grid_alignment.py for the final precise stop once the
coil tags are in view.

The stage/coil-spacing numbers in shelf_layout.py are placeholders pending
a real measurement, so distances here will be off until that's updated.
"""

from __future__ import annotations

import argparse
import time
from math import radians

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from wpt_adjustment_turtlebot.shelf_layout import distance_and_heading, shelf_position

DEFAULT_LINEAR_SPEED_M_S = 0.05
DEFAULT_ANGULAR_SPEED_DEG_S = 20.0
LOOP_HZ = 10.0


class OpenLoopDriver(Node):
    def __init__(self) -> None:
        super().__init__("open_loop_shelf_driver")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def turn_to(self, angle_deg: float, speed_deg_s: float) -> None:
        if abs(angle_deg) < 1e-6:
            return
        duration_s = abs(angle_deg) / speed_deg_s
        cmd = Twist()
        cmd.angular.z = radians(speed_deg_s) if angle_deg > 0 else -radians(speed_deg_s)
        self._publish_for(cmd, duration_s)

    def drive_forward(self, distance_m: float, speed_m_s: float) -> None:
        if abs(distance_m) < 1e-6:
            return
        duration_s = abs(distance_m) / speed_m_s
        cmd = Twist()
        cmd.linear.x = speed_m_s if distance_m > 0 else -speed_m_s
        self._publish_for(cmd, duration_s)

    def stop(self) -> None:
        self.pub.publish(Twist())

    def _publish_for(self, cmd: Twist, duration_s: float) -> None:
        period_s = 1.0 / LOOP_HZ
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            self.pub.publish(cmd)
            time.sleep(period_s)
        self.stop()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--shelf", type=int, required=True, choices=[1, 2, 3, 4], help="target shelf")
    parser.add_argument(
        "--from-shelf",
        type=int,
        default=1,
        choices=[1, 2, 3, 4],
        help="shelf the robot is currently sitting at, facing +x (default: 1)",
    )
    parser.add_argument("--linear-speed", type=float, default=DEFAULT_LINEAR_SPEED_M_S, help="m/s")
    parser.add_argument("--angular-speed-deg", type=float, default=DEFAULT_ANGULAR_SPEED_DEG_S, help="deg/s")
    parser.add_argument("--dry-run", action="store_true", help="print the plan without publishing /cmd_vel")
    args = parser.parse_args()

    start = shelf_position(args.from_shelf)
    target = shelf_position(args.shelf)
    distance_m, heading_deg = distance_and_heading(start, target)

    print(
        f"from_shelf={args.from_shelf} {start} -> shelf={args.shelf} {target} "
        f"distance={distance_m:.3f}m heading={heading_deg:.1f}deg"
    )
    if args.dry_run:
        return
    if args.from_shelf == args.shelf:
        print("already at target shelf, nothing to drive")
        return

    rclpy.init()
    node = OpenLoopDriver()
    try:
        node.turn_to(heading_deg, args.angular_speed_deg)
        node.drive_forward(distance_m, args.linear_speed)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
