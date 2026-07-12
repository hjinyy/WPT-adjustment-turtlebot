#!/usr/bin/env python3
"""Bridge the local 2x2 coil-alignment experiment with the charging-control
server at tserver.local:8000.

Run turtlebot3_bringup first (cmd_vel + /battery_state need it), then:

    source /opt/ros/humble/setup.bash
    ros2 launch turtlebot3_bringup robot.launch.py &
    python3 scripts/server_agent.py
    python3 scripts/server_agent.py --dry-run   # don't publish /cmd_vel, just log

This polls the server for "navigate_to" commands, estimates the current shelf
and heading from visible local AprilTags, turns toward the target shelf, follows
the black tape rectangle with the front camera, stops when the target coil's
three-tag grid condition is satisfied, acks the command, and periodically
reports status (current node, battery, alignment) back.

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
import sys
import time
from math import radians
from pathlib import Path

import cv2
import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from wpt_adjustment_turtlebot.controller_math import (
    TagObservation,
    TargetPoseInImage,
    VelocityCommand,
    alignment_state_for_report,
    compute_alignment_cmd,
    compute_pair_alignment_error,
    compute_pair_observation,
    grid_cell,
    is_aligned,
)
from wpt_adjustment_turtlebot.line_tracing import compute_line_trace_cmd, detect_black_line
from wpt_adjustment_turtlebot.server_client import DEFAULT_BASE_URL, DEFAULT_ROBOT_ID, ServerClient
from wpt_adjustment_turtlebot.shelf_layout import NODE_TO_SHELF, SHELF_TO_NODE, distance_and_heading, shelf_position
from wpt_adjustment_turtlebot.tag_layout import four_coil_pair_ids, four_coil_tag_id
from wpt_adjustment_turtlebot.tag_navigation import NavigationTag, estimate_pose_from_tags, rectilinear_shelf_path, turn_delta_deg
from wpt_adjustment_turtlebot.wpt_alignment_node import AprilTagDetector

DEFAULT_LINEAR_SPEED_M_S = 0.05
DEFAULT_ANGULAR_SPEED_DEG_S = 20.0
LOOP_HZ = 10.0
REPORT_INTERVAL_S = 1.0  # matches the server's default event_polling_interval_ms
STABLE_FRAMES_FOR_LOCK = 10
ALIGN_TIMEOUT_S = 30.0
LOCALIZE_TIMEOUT_S = 2.0
LINE_TRACE_TIMEOUT_S = 20.0


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

    def publish_velocity(self, cmd: VelocityCommand) -> None:
        if self.dry_run:
            print(f"[dry-run] would publish linear.x={cmd.linear_x:.4f} angular.z={cmd.angular_z:.4f}")
            return
        msg = Twist()
        msg.linear.x = float(cmd.linear_x)
        msg.angular.z = float(cmd.angular_z)
        self.cmd_pub.publish(msg)

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

    def read_all_tags(self) -> list[TagObservation]:
        rclpy.spin_once(self, timeout_sec=0.01)
        observations: list[TagObservation] = []
        for camera_name, cap in self.cameras.items():
            ok, frame = cap.read()
            if not ok:
                continue
            for det in self.detector.detect(frame):
                observations.append(
                    TagObservation(
                        tag_id=det.tag_id,
                        center_x=det.center[0],
                        center_y=det.center[1],
                        angle_deg=det.angle_deg,
                        area_px=det.area_px,
                        camera_name=camera_name,
                    )
                )
        return observations

    def localize_from_tags(self, timeout_s: float) -> tuple[int | None, float | None, list[str], str]:
        end = time.monotonic() + timeout_s
        best_pose = None
        while time.monotonic() < end:
            observations = self.read_all_tags()
            nav_tags = [NavigationTag(o.tag_id, o.camera_name, o.area_px, o.angle_deg) for o in observations]
            pose = estimate_pose_from_tags(nav_tags)
            if pose is not None:
                best_pose = pose
                if pose.heading_deg is not None:
                    break
            time.sleep(1.0 / LOOP_HZ)
        if best_pose is None:
            return None, None, [], ""
        return (
            best_pose.shelf,
            best_pose.heading_deg,
            [str(tag_id) for tag_id in best_pose.detected_tag_ids],
            best_pose.heading_source,
        )

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
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, 10)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def drive_to_node(
    node: RobotAgentNode,
    from_shelf: int,
    to_shelf: int,
    current_heading_deg: float,
    linear_speed: float,
    angular_speed_deg: float,
) -> float:
    start = shelf_position(from_shelf)
    target = shelf_position(to_shelf)
    distance_m, heading_deg = distance_and_heading(start, target)
    turn_deg = turn_delta_deg(current_heading_deg, heading_deg)
    print(
        f"driving shelf {from_shelf} -> shelf {to_shelf}: distance={distance_m:.3f}m "
        f"current_heading={current_heading_deg:.1f}deg target_heading={heading_deg:.1f}deg turn={turn_deg:.1f}deg"
    )
    node.turn_to(turn_deg, angular_speed_deg)
    node.drive_forward(distance_m, linear_speed)
    return heading_deg


def line_trace_to_node(
    node: RobotAgentNode,
    *,
    from_shelf: int,
    to_shelf: int,
    current_heading_deg: float,
    width: int,
    height: int,
    grid_size: int,
    line_target_col: int,
    line_linear_speed: float,
    angular_speed_deg: float,
    line_k_angular: float,
    line_max_angular: float,
    line_max_value: int,
    line_min_area_px: float,
    line_timeout_s: float,
    target_front_position: str,
    front_target_cell: tuple[int, int],
    side_target_cell: tuple[int, int],
    target_stable_frames: int,
) -> tuple[float, str]:
    _, target_heading_deg = distance_and_heading(shelf_position(from_shelf), shelf_position(to_shelf))
    turn_deg = turn_delta_deg(current_heading_deg, target_heading_deg)
    print(
        f"line tracing shelf {from_shelf} -> shelf {to_shelf}: "
        f"current_heading={current_heading_deg:.1f}deg target_heading={target_heading_deg:.1f}deg turn={turn_deg:.1f}deg"
    )
    node.turn_to(turn_deg, angular_speed_deg)

    target_coil = f"coil_{to_shelf}"
    stable_count = 0
    any_target_seen = False
    last_line_seen = time.monotonic()
    end = time.monotonic() + line_timeout_s
    period_s = 1.0 / LOOP_HZ

    while time.monotonic() < end:
        front_cap = node.cameras["front"]
        ok, frame = front_cap.read()
        if not ok:
            node.stop()
            return target_heading_deg, "Failed"

        line = detect_black_line(
            frame,
            grid_size=grid_size,
            max_value=line_max_value,
            min_area_px=line_min_area_px,
        )
        if line.found:
            last_line_seen = time.monotonic()
            cmd = compute_line_trace_cmd(
                line,
                linear_speed=line_linear_speed,
                k_angular=line_k_angular,
                max_angular=line_max_angular,
                target_col=line_target_col,
            )
            node.publish_velocity(cmd)
        else:
            node.publish_velocity(VelocityCommand())

        obs = node.read_all_tags()
        target_ok, seen, detail = target_three_tag_alignment(
            obs,
            target_coil=target_coil,
            front_position=target_front_position,
            front_cell=front_target_cell,
            side_cell=side_target_cell,
            width=width,
            height=height,
            grid_size=grid_size,
        )
        any_target_seen = any_target_seen or seen
        stable_count = stable_count + 1 if target_ok else 0
        line_state = f"line_cell={line.cell} line_error={line.error_x:.3f}" if line.found else "line=None"
        print(
            f"[line_trace] target={target_coil} {line_state} "
            f"line_center_col_ok={line.cell is not None and line.cell[1] == line_target_col} "
            f"target_stable={stable_count}/{target_stable_frames} {detail}",
            flush=True,
        )

        if stable_count >= target_stable_frames:
            node.stop()
            return target_heading_deg, "Locked"

        if time.monotonic() - last_line_seen > 1.0:
            print("[line_trace] line lost for more than 1.0s; stopping", flush=True)
            node.stop()
            return target_heading_deg, "Searching" if any_target_seen else "Failed"

        time.sleep(period_s)

    node.stop()
    return target_heading_deg, "Searching" if any_target_seen else "None"


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


def find_observation(obs: list[TagObservation], tag_id: int, camera_name: str) -> TagObservation | None:
    candidates = [o for o in obs if o.tag_id == tag_id and o.camera_name == camera_name]
    return max(candidates, key=lambda o: o.area_px, default=None)


def find_target_pair(
    obs: list[TagObservation],
    target_coil: str,
    pair_name: str,
    preferred_camera: str | None = None,
):
    first_id, second_id = four_coil_pair_ids(target_coil, pair_name)
    camera_names = sorted({o.camera_name for o in obs if o.camera_name != "front"})
    if preferred_camera:
        camera_names = [preferred_camera]
    best_pair = None
    best_area = -1.0
    for camera_name in camera_names:
        first = find_observation(obs, first_id, camera_name)
        second = find_observation(obs, second_id, camera_name)
        if first is None or second is None:
            continue
        area = first.area_px + second.area_px
        if area > best_area:
            best_pair = compute_pair_observation(first, second)
            best_area = area
    return best_pair


def has_any_pair_marker(obs: list[TagObservation], target_coil: str, pair_name: str) -> bool:
    required = set(four_coil_pair_ids(target_coil, pair_name))
    return any(o.tag_id in required for o in obs)


def observed_cell_by_tag(obs: list[TagObservation], tag_id: int, camera_name: str, width: int, height: int, grid_size: int):
    candidates = [o for o in obs if o.tag_id == tag_id and o.camera_name == camera_name]
    best = max(candidates, key=lambda o: o.area_px, default=None)
    if best is None:
        return None
    return grid_cell(best.center_x, best.center_y, width, height, grid_size)


def target_three_tag_alignment(
    obs: list[TagObservation],
    *,
    target_coil: str,
    front_position: str,
    front_cell: tuple[int, int],
    side_cell: tuple[int, int],
    width: int,
    height: int,
    grid_size: int,
) -> tuple[bool, bool, str]:
    west_id = four_coil_tag_id(target_coil, "west")
    east_id = four_coil_tag_id(target_coil, "east")
    front_ids = (
        [four_coil_tag_id(target_coil, position) for position in ("north", "south", "west", "east")]
        if front_position == "any"
        else [four_coil_tag_id(target_coil, front_position)]
    )

    west_cell = None
    east_cell = None
    for camera_name in ("right_bottom", "left_bottom"):
        west_cell = west_cell or observed_cell_by_tag(obs, west_id, camera_name, width, height, grid_size)
        east_cell = east_cell or observed_cell_by_tag(obs, east_id, camera_name, width, height, grid_size)
    front_seen = None
    for front_id in front_ids:
        front_cell_candidate = observed_cell_by_tag(obs, front_id, "front", width, height, grid_size)
        if front_cell_candidate is not None:
            front_seen = (front_id, front_cell_candidate)
            break
    front_seen_cell = front_seen[1] if front_seen is not None else None

    any_target_seen = any(cell is not None for cell in (west_cell, east_cell, front_seen_cell))
    ok = west_cell == side_cell and east_cell == side_cell and front_seen_cell == front_cell
    detail = (
        f"target={target_coil} west_id={west_id}@{west_cell} east_id={east_id}@{east_cell} "
        f"front={front_seen} side_cell={side_cell} front_cell={front_cell}"
    )
    return ok, any_target_seen, detail


def target_for_pair(config: dict, target_coil: str, camera_name: str, pair_name: str) -> TargetPoseInImage:
    coil_cfg = config["coils"][target_coil]
    target_cfg = coil_cfg["targets"][camera_name].get(pair_name, coil_cfg["targets"][camera_name]["default"])
    return TargetPoseInImage(
        x=float(target_cfg["x"]),
        y=float(target_cfg["y"]),
        angle_deg=float(target_cfg["angle_deg"]),
    )


def coil_thresholds(config: dict) -> dict[str, float]:
    cfg = config["alignment"]["coil"]
    return {
        "threshold_x_px": float(cfg["threshold_x_px"]),
        "threshold_y_px": float(cfg["threshold_y_px"]),
        "threshold_angle_deg": float(cfg["threshold_angle_deg"]),
    }


def coil_control_params(config: dict) -> dict:
    c = config["control"]["coil"]
    s = config["speed"]
    return {
        "k_y_to_linear": float(c["k_y_to_linear"]),
        "k_x_to_angular": float(c["k_x_to_angular"]),
        "k_angle_to_angular": float(c["k_angle_to_angular"]),
        "max_linear": float(s["coil_max_linear"]),
        "max_angular": float(s["coil_max_angular"]),
        "min_linear": float(s.get("coil_min_linear", 0.0)),
        "min_angular": float(s.get("coil_min_angular", 0.0)),
        "x_deadband_px": float(c.get("x_deadband_px", 0.0)),
        "y_deadband_px": float(c.get("y_deadband_px", 0.0)),
        "angle_deadband_deg": float(c.get("angle_deadband_deg", 0.0)),
        "invert_linear": bool(c.get("invert_linear", False)),
        "invert_angular": bool(c.get("invert_angular", False)),
    }


def fine_align_to_coil(node: RobotAgentNode, config: dict, target_coil: str, timeout_s: float) -> str:
    """Closed-loop final alignment using the same pair midpoint/angle math as wpt_alignment_node."""
    pair_name = str(config["alignment"].get("final_pair", "west_east"))
    stable_required = int(config["alignment"].get("stable_frames_required", STABLE_FRAMES_FOR_LOCK))
    search_cfg = config["alignment"].get("search", {})
    search_linear = float(search_cfg.get("burst_linear", config["speed"].get("shelf_entry_linear", 0.025)))
    stable_count = 0
    active_camera: str | None = None
    any_detected = False
    end = time.monotonic() + timeout_s
    period_s = 1.0 / LOOP_HZ

    while time.monotonic() < end:
        obs = node.read_all_tags()
        any_detected = any_detected or bool(obs)
        pair = find_target_pair(obs, target_coil, pair_name, active_camera)
        if pair is None and active_camera is not None:
            pair = find_target_pair(obs, target_coil, pair_name)

        if pair is None:
            stable_count = 0
            if has_any_pair_marker(obs, target_coil, pair_name):
                node.publish_velocity(VelocityCommand())
                print(f"[fine_align] target={target_coil} pair={pair_name} partial marker visible; holding")
            else:
                node.publish_velocity(VelocityCommand(linear_x=search_linear))
                print(f"[fine_align] target={target_coil} pair={pair_name} not visible; creeping forward")
            time.sleep(period_s)
            continue

        active_camera = pair.camera_name
        err = compute_pair_alignment_error(pair, target_for_pair(config, target_coil, pair.camera_name, pair_name))
        aligned = is_aligned(err, **coil_thresholds(config))
        stable_count = stable_count + 1 if aligned else 0
        print(
            f"[fine_align] target={target_coil} camera={pair.camera_name} "
            f"x_error={err.x:.2f} y_error={err.y:.2f} angle_error={err.angle_deg:.2f} "
            f"stable={stable_count}/{stable_required}"
        )
        if stable_count >= stable_required:
            node.stop()
            return "Locked"
        node.publish_velocity(compute_alignment_cmd(err, **coil_control_params(config)))
        time.sleep(period_s)

    node.stop()
    return "Searching" if any_detected else "None"


def load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def navigate_between_shelves(
    node: RobotAgentNode,
    config: dict,
    args: argparse.Namespace,
    *,
    current_shelf: int,
    current_heading_deg: float,
    target_shelf: int,
) -> tuple[int, float, str]:
    observed_shelf, observed_heading, detected_tag_ids, heading_source = node.localize_from_tags(args.localize_timeout)
    if observed_shelf is not None:
        current_shelf = observed_shelf
    if observed_heading is not None:
        current_heading_deg = observed_heading
    print(
        f"localization before move: shelf={observed_shelf} heading={observed_heading} "
        f"heading_source={heading_source or 'fallback'} detected_tags={detected_tag_ids} "
        f"using_shelf={current_shelf} using_heading={current_heading_deg:.1f}"
    )

    target_cell = (args.target_row, args.target_col)
    if args.skip_line_tracing:
        current_heading_deg = drive_to_node(
            node,
            current_shelf,
            target_shelf,
            current_heading_deg,
            args.linear_speed,
            args.angular_speed_deg,
        )
        if args.skip_fine_align:
            state = wait_for_lock(node, target_cell, args.grid_size, args.fine_align_timeout)
        else:
            state = fine_align_to_coil(node, config, f"coil_{target_shelf}", args.fine_align_timeout)
        return target_shelf, current_heading_deg, state

    path = rectilinear_shelf_path(current_shelf, target_shelf)
    print(f"line path: {path}")
    state = "Locked"
    for segment_target in path[1:]:
        current_heading_deg, state = line_trace_to_node(
            node,
            from_shelf=current_shelf,
            to_shelf=segment_target,
            current_heading_deg=current_heading_deg,
            width=args.width,
            height=args.height,
            grid_size=args.grid_size,
            line_target_col=args.line_target_col,
            line_linear_speed=args.line_linear_speed,
            angular_speed_deg=args.angular_speed_deg,
            line_k_angular=args.line_k_angular,
            line_max_angular=args.line_max_angular,
            line_max_value=args.line_max_value,
            line_min_area_px=args.line_min_area_px,
            line_timeout_s=args.line_timeout,
            target_front_position=args.target_front_position,
            front_target_cell=(args.front_target_row, args.front_target_col),
            side_target_cell=(args.side_target_row, args.side_target_col),
            target_stable_frames=args.target_stable_frames,
        )
        if state != "Locked":
            return current_shelf, current_heading_deg, state
        current_shelf = segment_target
    return target_shelf, current_heading_deg, state


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--server-url", default=DEFAULT_BASE_URL)
    parser.add_argument("--robot-id", default=DEFAULT_ROBOT_ID)
    parser.add_argument("--start-shelf", type=int, default=1, choices=[1, 2, 3, 4], help="shelf the robot starts at")
    parser.add_argument("--start-heading-deg", type=float, default=0.0, help="fallback heading if front-camera tags cannot estimate it")
    parser.add_argument("--config-file", default="config/wpt_alignment.yaml")
    parser.add_argument("--front-device", type=int, default=0)
    parser.add_argument("--right-device", type=int, default=2)
    parser.add_argument("--left-device", type=int, default=4)
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=240)
    parser.add_argument("--family", default="tag36h11")
    parser.add_argument("--grid-size", type=int, default=3)
    parser.add_argument("--target-row", type=int, default=2)
    parser.add_argument("--target-col", type=int, default=2)
    parser.add_argument("--linear-speed", type=float, default=DEFAULT_LINEAR_SPEED_M_S)
    parser.add_argument("--angular-speed-deg", type=float, default=DEFAULT_ANGULAR_SPEED_DEG_S)
    parser.add_argument("--report-interval", type=float, default=REPORT_INTERVAL_S)
    parser.add_argument("--localize-timeout", type=float, default=LOCALIZE_TIMEOUT_S)
    parser.add_argument("--fine-align-timeout", type=float, default=ALIGN_TIMEOUT_S)
    parser.add_argument("--skip-line-tracing", action="store_true", help="use older time-based straight drive instead of front-camera tape following")
    parser.add_argument("--line-timeout", type=float, default=LINE_TRACE_TIMEOUT_S)
    parser.add_argument("--line-linear-speed", type=float, default=0.035)
    parser.add_argument("--line-k-angular", type=float, default=0.7)
    parser.add_argument("--line-max-angular", type=float, default=0.18)
    parser.add_argument("--line-max-value", type=int, default=85, help="HSV V threshold for black tape detection")
    parser.add_argument("--line-min-area-px", type=float, default=80.0)
    parser.add_argument("--line-target-col", type=int, default=2)
    parser.add_argument("--front-target-row", type=int, default=1)
    parser.add_argument("--front-target-col", type=int, default=2)
    parser.add_argument("--side-target-row", type=int, default=3)
    parser.add_argument("--side-target-col", type=int, default=2)
    parser.add_argument("--target-front-position", default="any", choices=["any", "north", "south", "west", "east"])
    parser.add_argument("--target-stable-frames", type=int, default=2)
    parser.add_argument("--once-to-shelf", type=int, choices=[1, 2, 3, 4], help="run one navigation locally without polling the server")
    parser.add_argument("--skip-fine-align", action="store_true", help="fall back to the older 3x3 grid lock check")
    parser.add_argument("--dry-run", action="store_true", help="don't publish /cmd_vel, just log planned moves")
    args = parser.parse_args()

    target_cell = (args.target_row, args.target_col)
    config = load_config(args.config_file)
    client = ServerClient(base_url=args.server_url, robot_id=args.robot_id)

    cameras = {
        "front": open_camera(args.front_device, args.width, args.height),
        "right_bottom": open_camera(args.right_device, args.width, args.height),
        "left_bottom": open_camera(args.left_device, args.width, args.height),
    }
    detector = AprilTagDetector(args.family)
    print(f"AprilTag backend: {detector.backend}")

    rclpy.init()
    node = RobotAgentNode(cameras, detector, args.dry_run)
    current_shelf = args.start_shelf
    current_heading_deg = args.start_heading_deg

    try:
        if args.once_to_shelf is not None:
            current_shelf, current_heading_deg, state = navigate_between_shelves(
                node,
                config,
                args,
                current_shelf=current_shelf,
                current_heading_deg=current_heading_deg,
                target_shelf=args.once_to_shelf,
            )
            print(
                f"once navigation complete: shelf={current_shelf} "
                f"heading={current_heading_deg:.1f} state={state}",
                flush=True,
            )
            return

        while rclpy.ok():
            command = None
            try:
                command = client.next_command()
            except Exception as exc:  # noqa: BLE001 - network hiccups shouldn't crash the agent
                print(f"failed to fetch next command: {exc}")

            if command and command.get("command") == "navigate_to":
                target_node_id = command.get("target_node_id")
                target_shelf = NODE_TO_SHELF.get(target_node_id)
                command_id = command.get("id") or command.get("command_id")
                if target_shelf is None:
                    print(f"unknown target_node_id '{target_node_id}', not in {list(NODE_TO_SHELF)}; failing command")
                    if command_id:
                        client.ack_command(command_id, status="failed", message=f"unknown node {target_node_id}")
                else:
                    try:
                        current_shelf, current_heading_deg, state = navigate_between_shelves(
                            node,
                            config,
                            args,
                            current_shelf=current_shelf,
                            current_heading_deg=current_heading_deg,
                            target_shelf=target_shelf,
                        )
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
                            client.ack_command(command_id, status="acked" if state not in {"None", "Failed"} else "failed")
                    except Exception as exc:  # noqa: BLE001
                        print(f"navigate_to failed: {exc}")
                        if command_id:
                            client.ack_command(command_id, status="failed", message=str(exc))
                continue  # check for another queued command right away

            # idle: just report current status
            observed_shelf, observed_heading, _, _heading_source = node.localize_from_tags(0.2)
            if observed_shelf is not None:
                current_shelf = observed_shelf
            if observed_heading is not None:
                current_heading_deg = observed_heading
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
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as exc:  # noqa: BLE001
            print(f"warning: rclpy shutdown skipped: {exc}")


if __name__ == "__main__":
    main()
