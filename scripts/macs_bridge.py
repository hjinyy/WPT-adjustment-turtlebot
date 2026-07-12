#!/usr/bin/env python3
"""10 Hz MACS <-> TurtleBot3 HTTP bridge.

Runs a fixed 10 Hz loop that:
  1. polls the MACS server for the next command (navigate_to / patrol /
     pause / resume / estop / clear_fault),
  2. drives a motion backend one step toward the goal,
  3. posts rich telemetry every tick (phase, heading, linear/angular velocity,
     detected AprilTags, alignment/charging state, and whether the last command
     was received/executing/done/failed).

Server -> robot: where to move (navigate_to with a node path), patrol hops
(same navigate_to with {patrol, dwell_sec}), and estop. Robot -> server: the
10 Hz status above, so the web UI shows the live movement direction and state.

Motion backends:
  --sim   : no ROS, simulates motion (default off-robot; lets you exercise the
            whole link + watch the web UI update without hardware).
  (ROS)   : on the robot, drop in a backend that publishes /cmd_vel and reads
            AprilTags -- see RosMotion below. Requires rclpy + turtlebot3_bringup.

Examples:
    # Off-robot: drive the real server's UI at 10 Hz with a simulated robot
    python3 scripts/macs_bridge.py --server http://192.168.0.7:8000 --sim

    # On the robot (once RosMotion is implemented):
    source /opt/ros/humble/setup.bash
    ros2 launch turtlebot3_bringup robot.launch.py &
    python3 scripts/macs_bridge.py --server http://192.168.0.7:8000
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from wpt_adjustment_turtlebot.macs_link import (  # noqa: E402
    ALIGNING,
    CHARGING,
    DRIVING,
    DWELLING,
    ESTOPPED,
    IDLE,
    STOPPED,
    LegPlan,
)
from wpt_adjustment_turtlebot.server_client import DEFAULT_BASE_URL, DEFAULT_ROBOT_ID, ServerClient  # noqa: E402

LOOP_HZ = 10.0


class SimMotion:
    """No-ROS motion model: advances along legs at a fixed speed, 'aligns' for
    a short settle time on arrival at a workspace. Lets the bridge run and post
    real 10 Hz telemetry without hardware.
    """

    def __init__(self, start_node: str, leg_seconds: float = 1.5, align_seconds: float = 1.0) -> None:
        self.node = start_node
        self.leg_seconds = leg_seconds
        self.align_seconds = align_seconds
        self.detected_tags: list[str] = []

    def drive_leg(self, from_node, to_node, heading, on_tick):
        """Advance one leg; call on_tick(fraction, linear, angular) each step."""
        steps = max(1, int(self.leg_seconds * LOOP_HZ))
        for i in range(steps):
            on_tick((i + 1) / steps, 0.05, 0.0)
            time.sleep(1.0 / LOOP_HZ)
        self.node = to_node

    def align(self, node, required_tags, on_tick):
        """Simulate settling onto the coil; reveal tags progressively."""
        steps = max(1, int(self.align_seconds * LOOP_HZ))
        tags = required_tags or []
        for i in range(steps):
            self.detected_tags = tags[: (i + 1) * len(tags) // steps] if tags else []
            on_tick((i + 1) / steps, 0.01, 0.02)
            time.sleep(1.0 / LOOP_HZ)
        self.detected_tags = list(tags)


class Bridge:
    def __init__(self, client: ServerClient, motion: SimMotion) -> None:
        self.client = client
        self.motion = motion
        self.phase = IDLE
        self.heading: str | None = None
        self.target_node: str | None = None
        self.estopped = False
        self.charging = False
        self.battery = 80.0

    # --- telemetry ---------------------------------------------------------
    def status(self, *, linear=0.0, angular=0.0, alignment=None, command_id=None, command_status=None):
        self.client.post_status(
            phase=self.phase,
            node_id=self.motion.node,
            target_node_id=self.target_node,
            heading=self.heading,
            linear_velocity=linear,
            angular_velocity=angular,
            alignment_state=alignment,
            detected_tag_ids=self.motion.detected_tags,
            charging=self.charging,
            battery_percent=round(self.battery, 1),
            command_id=command_id,
            command_status=command_status,
        )

    def milestone(self, message: str, *, severity="Info", alignment=None, event_type="Navigation"):
        self.client.post_event(
            node_id=self.motion.node,
            target_node_id=self.target_node,
            alignment_state=alignment,
            detected_tag_ids=self.motion.detected_tags,
            charging=self.charging,
            battery_percent=round(self.battery, 1),
            mode="Auto",
            phase=self.phase,
            heading=self.heading,
            message=message,
            severity=severity,
        )

    # --- command handling --------------------------------------------------
    def handle_estop(self, command_id):
        self.phase = ESTOPPED
        self.heading = None
        self.estopped = True
        self.charging = False
        if command_id:
            self.client.ack_command(command_id, status="acked", message="Emergency stop")
        self.milestone("Emergency stop activated", severity="Critical", event_type="Safety")
        self.status(command_id=command_id, command_status="done")

    def handle_clear_fault(self, command_id):
        self.estopped = False
        self.phase = IDLE
        if command_id:
            self.client.ack_command(command_id, status="acked", message="Fault cleared")
        self.milestone("Fault cleared", event_type="Robot")

    def handle_navigate(self, command):
        command_id = command.get("id")
        payload = command.get("payload") or {}
        self.target_node = command.get("targetNodeId")
        path = payload.get("path") or [self.motion.node, self.target_node]
        is_ws = payload.get("mode") == "workspace_alignment"
        dwell = payload.get("dwell_sec")
        try:
            plan = LegPlan.from_path(path, is_ws)
        except ValueError as exc:
            if command_id:
                self.client.ack_command(command_id, status="failed", message=str(exc))
            self.milestone(f"Rejected path: {exc}", severity="Warning")
            return
        if command_id:
            self.client.ack_command(command_id, status="acked", message=f"navigate_to {self.target_node}")
        self.status(command_id=command_id, command_status="received")

        for from_node, to_node, heading in plan.legs:
            if self.estopped:
                return
            self.phase = DRIVING
            self.heading = heading
            self.milestone(f"Driving {from_node} -> {to_node} ({heading})")
            self.motion.drive_leg(
                from_node, to_node, heading,
                lambda frac, lin, ang: self.status(linear=lin, angular=ang, command_id=command_id, command_status="executing"),
            )
            self.milestone(f"Arrived at node {to_node}", alignment="Aligned")

        if plan.is_workspace_target:
            self.phase = ALIGNING
            self.heading = None
            required = payload.get("required_tags") or []
            self.milestone(f"Workspace {self.target_node} alignment searching", alignment="Searching", event_type="Alignment")
            self.motion.align(
                self.target_node, required,
                lambda frac, lin, ang: self.status(linear=lin, angular=ang, alignment="Searching", command_id=command_id, command_status="executing"),
            )
            self.charging = True
            self.phase = DWELLING if dwell else CHARGING
            self.milestone(
                f"Workspace {self.target_node} alignment locked" + (f"; dwelling {dwell:.0f}s (patrol)" if dwell else ""),
                alignment="Locked",
                event_type="Charging",
            )
        else:
            self.phase = STOPPED

        if command_id:
            self.client.ack_command(command_id, status="acked", message=f"Command completed: arrived at {self.target_node}")
        self.status(alignment="Locked" if plan.is_workspace_target else None, command_id=command_id, command_status="done")

    # --- main loop ---------------------------------------------------------
    def run(self):
        print(f"MACS bridge up at {LOOP_HZ:.0f} Hz. robot={self.client.robot_id} start_node={self.motion.node}")
        period = 1.0 / LOOP_HZ
        while True:
            tick_start = time.monotonic()
            try:
                command = self.client.next_command()
            except Exception as exc:  # noqa: BLE001 - network hiccups shouldn't kill the bridge
                print(f"poll failed: {exc}")
                command = None

            if command:
                name = command.get("command")
                cid = command.get("id")
                if name == "estop":
                    self.handle_estop(cid)
                elif name == "clear_fault":
                    self.handle_clear_fault(cid)
                elif name in {"pause", "resume"}:
                    if cid:
                        self.client.ack_command(cid, status="acked", message=name)
                    self.phase = STOPPED if name == "pause" else IDLE
                    self.milestone(name.capitalize(), event_type="Robot")
                elif name == "navigate_to" and not self.estopped:
                    self.handle_navigate(command)
                elif name == "navigate_to" and self.estopped:
                    if cid:
                        self.client.ack_command(cid, status="failed", message="Ignored: e-stopped, send clear_fault first")
            else:
                # Idle heartbeat: keep the 10 Hz telemetry flowing.
                if not self.estopped and self.phase not in {CHARGING, DWELLING}:
                    self.phase = IDLE
                    self.heading = None
                self.status()

            elapsed = time.monotonic() - tick_start
            if elapsed < period:
                time.sleep(period - elapsed)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--server", default=DEFAULT_BASE_URL)
    parser.add_argument("--robot-id", default=DEFAULT_ROBOT_ID)
    parser.add_argument("--start-node", default="A02", help="node the robot starts on")
    parser.add_argument("--sim", action="store_true", help="simulate motion (no ROS); required off-robot")
    parser.add_argument("--leg-seconds", type=float, default=1.5, help="sim: seconds per one-node leg")
    args = parser.parse_args()

    if not args.sim:
        print("Only --sim is implemented here. On the robot, plug a ROS motion "
              "backend (publish /cmd_vel + read AprilTags) into Bridge in place "
              "of SimMotion; the 10 Hz link/telemetry stays identical.")
        return 2

    client = ServerClient(base_url=args.server, robot_id=args.robot_id)
    motion = SimMotion(args.start_node, leg_seconds=args.leg_seconds)
    bridge = Bridge(client, motion)
    # Announce our start position so the server/UI place the robot correctly.
    bridge.status()
    try:
        bridge.run()
    except KeyboardInterrupt:
        bridge.phase = STOPPED
        bridge.status()
        print("\nbridge stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
