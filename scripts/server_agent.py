#!/usr/bin/env python3
"""Bridge the local 2x2 coil-alignment experiment with the charging-control
server at tserver.local:8000.

Run turtlebot3_bringup first (cmd_vel + /battery_state need it), then:

    source /opt/ros/humble/setup.bash
    ros2 launch turtlebot3_bringup robot.launch.py &
    python3 scripts/server_agent.py
    python3 scripts/server_agent.py --dry-run   # don't publish /cmd_vel, just log

This polls the server for "navigate_to" commands, drives to the target
shelf open-loop (see shelf_layout.py — dead reckoning, no odometry), checks
3x3-grid coil alignment across the three cameras, acks the command, and
periodically reports status (current node, battery, alignment) back.

Server node id <-> local shelf number (only 4 of the server's 8 grid nodes
are charging "Workspace" nodes; see README):
    A02 -> shelf 1
    B02 -> shelf 2
    A03 -> shelf 3
    B03 -> shelf 4

Tag numbering stays local-only: alignment is still checked against our own
printed tags (11-44, see tag_layout.py); only the resulting node_id and
alignment_state are reported to the server. The server's own AprilTag
marker-value registry (a different numbering) isn't used here — see README.
"""

from __future__ import annotations

import argparse
import time
from math import radians

import cv2
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from wpt_adjustment_turtlebot.controller_math import alignment_state_for_report, grid_cell
from wpt_adjustment_turtlebot.server_client import DEFAULT_BASE_URL, DEFAULT_ROBOT_ID, ServerClient
from wpt_adjustment_turtlebot.shelf_layout import NODE_TO_SHELF, SHELF_TO_NODE, distance_and_heading, shelf_position
from wpt_adjustment_turtlebot.wpt_alignment_node import AprilTagDetector

DEFAULT_LINEAR_SPEED_M_S = 0.05
DEFAULT_ANGULAR_SPEED_DEG_S = 20.0
LOOP_HZ = 10.0
REPORT_INTERVAL_S = 1.0  # matches the server's default event_polling_interval_ms
STABLE_FRAMES_FOR_LOCK = 10
ALIGN_TIMEOUT_S = 30.0


class RobotAgentNode(Node):
    def __init__(self, cameras: dict[str, cv2.VideoCapture], detector: AprilTagDetector, dry_run: bool) -> None:
        super().__init__("server_agent")
        self.dry_run = dry_run
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(BatteryState, "/battery_state", self._on_battery, 10)
        self.battery_percent: float | None = None
        self.battery_voltage: float | None = None
        self.charging: bool | None = None
        self.cameras = cameras
        self.detector = detector

    def _on_battery(self, msg: BatteryState) -> None:
        percent = msg.percentage
        self.battery_percent = percent * 100.0 if percent is not None and percent <= 1.0 else percent
        self.battery_voltage = msg.voltage
        self.charging = msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING

    def refresh_battery(self, duration_s: float = 0.2) -> None:
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

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
        if not self.dry_run:
            self.cmd_pub.publish(Twist())

    def _publish_for(self, cmd: Twist, duration_s: float) -> None:
        if self.dry_run:
            print(f"[dry-run] would publish {cmd} for {duration_s:.2f}s")
            return
        period_s = 1.0 / LOOP_HZ
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=period_s)
        self.stop()

    def check_alignment(self, target_cell: tuple[int, int], grid_size: int) -> tuple[bool, bool, list[str]]:
        """One-shot read of all 3 cameras. Returns (all_aligned, any_detected, detected_tag_ids)."""
        rclpy.spin_once(self, timeout_sec=0.01)
        any_detected = False
        all_aligned = True
        tag_ids: list[str] = []
        for cap in self.cameras.values():
            ok, frame = cap.read()
            if not ok:
                all_aligned = False
                continue
            h, w = frame.shape[:2]
            detections = self.detector.detect(frame)
            if not detections:
                all_aligned = False
                continue
            any_detected = True
            best = max(detections, key=lambda d: d.area_px)
            tag_ids.append(str(best.tag_id))
            if grid_cell(best.center[0], best.center[1], w, h, grid_size) != target_cell:
                all_aligned = False
        return all_aligned, any_detected, tag_ids


def open_camera(device: int, width: int, height: int) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap


def drive_to_node(node: RobotAgentNode, from_shelf: int, to_shelf: int, linear_speed: float, angular_speed_deg: float) -> None:
    start = shelf_position(from_shelf)
    target = shelf_position(to_shelf)
    distance_m, heading_deg = distance_and_heading(start, target)
    print(f"driving shelf {from_shelf} -> shelf {to_shelf}: distance={distance_m:.3f}m heading={heading_deg:.1f}deg")
    node.turn_to(heading_deg, angular_speed_deg)
    node.drive_forward(distance_m, linear_speed)


def wait_for_lock(node: RobotAgentNode, target_cell: tuple[int, int], grid_size: int, timeout_s: float) -> str:
    """Poll camera alignment until STABLE_FRAMES_FOR_LOCK consecutive locked frames or timeout."""
    stable_count = 0
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        all_aligned, any_detected, _ = node.check_alignment(target_cell, grid_size)
        stable_count = stable_count + 1 if all_aligned else 0
        if stable_count >= STABLE_FRAMES_FOR_LOCK:
            return "Locked"
        time.sleep(1.0 / LOOP_HZ)
    all_aligned, any_detected, _ = node.check_alignment(target_cell, grid_size)
    return alignment_state_for_report(any_detected, all_aligned, locked=False)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--server-url", default=DEFAULT_BASE_URL)
    parser.add_argument("--robot-id", default=DEFAULT_ROBOT_ID)
    parser.add_argument("--start-shelf", type=int, default=1, choices=[1, 2, 3, 4], help="shelf the robot starts at")
    parser.add_argument("--front-device", type=int, default=0)
    parser.add_argument("--right-device", type=int, default=2)
    parser.add_argument("--left-device", type=int, default=4)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--family", default="tag36h11")
    parser.add_argument("--grid-size", type=int, default=3)
    parser.add_argument("--target-row", type=int, default=2)
    parser.add_argument("--target-col", type=int, default=2)
    parser.add_argument("--linear-speed", type=float, default=DEFAULT_LINEAR_SPEED_M_S)
    parser.add_argument("--angular-speed-deg", type=float, default=DEFAULT_ANGULAR_SPEED_DEG_S)
    parser.add_argument("--report-interval", type=float, default=REPORT_INTERVAL_S)
    parser.add_argument("--dry-run", action="store_true", help="don't publish /cmd_vel, just log planned moves")
    args = parser.parse_args()

    target_cell = (args.target_row, args.target_col)
    client = ServerClient(base_url=args.server_url, robot_id=args.robot_id)

    cameras = {
        "front": open_camera(args.front_device, args.width, args.height),
        "right": open_camera(args.right_device, args.width, args.height),
        "left": open_camera(args.left_device, args.width, args.height),
    }
    detector = AprilTagDetector(args.family)
    print(f"AprilTag backend: {detector.backend}")

    rclpy.init()
    node = RobotAgentNode(cameras, detector, args.dry_run)
    current_shelf = args.start_shelf

    try:
        while rclpy.ok():
            command = None
            try:
                command = client.next_command()
            except Exception as exc:  # noqa: BLE001 - network hiccups shouldn't crash the agent
                print(f"failed to fetch next command: {exc}")

            if command and command.get("command") == "navigate_to":
                # MACS server sends camelCase targetNodeId/id; keep snake_case
                # fallbacks so a simpler test/mock server still works.
                target_node_id = command.get("targetNodeId") or command.get("target_node_id")
                command_id = command.get("id") or command.get("command_id")
                payload = command.get("payload") or {}
                path = payload.get("path")
                if path:
                    print(f"navigate_to {target_node_id} via path {path} (mode={payload.get('mode')})")
                target_shelf = NODE_TO_SHELF.get(target_node_id)
                if target_shelf is None:
                    print(f"unknown target_node_id '{target_node_id}', not in {list(NODE_TO_SHELF)}; failing command")
                    if command_id:
                        client.ack_command(command_id, status="failed", message=f"unknown node {target_node_id}")
                else:
                    try:
                        drive_to_node(node, current_shelf, target_shelf, args.linear_speed, args.angular_speed_deg)
                        current_shelf = target_shelf
                        state = wait_for_lock(node, target_cell, args.grid_size, ALIGN_TIMEOUT_S)
                        node.refresh_battery()
                        client.post_event(
                            node_id=SHELF_TO_NODE[current_shelf],
                            alignment_state=state,
                            battery_percent=node.battery_percent,
                            battery_voltage=node.battery_voltage,
                            charging=node.charging,
                            mode="Auto",
                        )
                        if command_id:
                            client.ack_command(command_id, status="acked" if state != "None" else "failed")
                    except Exception as exc:  # noqa: BLE001
                        print(f"navigate_to failed: {exc}")
                        if command_id:
                            client.ack_command(command_id, status="failed", message=str(exc))
                continue  # check for another queued command right away

            # idle: just report current status
            all_aligned, any_detected, tag_ids = node.check_alignment(target_cell, args.grid_size)
            node.refresh_battery()
            try:
                client.post_event(
                    node_id=SHELF_TO_NODE[current_shelf],
                    alignment_state=alignment_state_for_report(any_detected, all_aligned, locked=False),
                    battery_percent=node.battery_percent,
                    battery_voltage=node.battery_voltage,
                    charging=node.charging,
                    detected_tag_ids=tag_ids,
                    mode="Auto",
                )
            except Exception as exc:  # noqa: BLE001
                print(f"failed to post event: {exc}")
            time.sleep(args.report_interval)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        for cap in cameras.values():
            cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
