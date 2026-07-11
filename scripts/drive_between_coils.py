#!/usr/bin/env python3
"""Line-following transit between WPT coils with marker-triggered stops.

All four coils share one global compass (see coil_transit.py):

        north
    coil_1 | coil_2
    -------+-------
    coil_3 | coil_4
        south

Per leg (e.g. coil_1 -> coil_3, direction=south, measured distance 25.5 cm;
horizontal legs are 45.3 cm):
1. CRUISE: drive forward along the black tape for transit.cruise_fraction of
   the measured coil-center distance. Both side cameras (left_bottom /
   right_bottom) detect the tape; their observations are fused through a
   Kalman filter and steer angular.z so heading drift is corrected
   continuously instead of accumulating.
2. APPROACH: slow to transit.approach_linear and stop THE INSTANT the target
   coil's head marker in the travel direction (coil_3 south = id 32) is seen
   by the front camera WHILE both of the target's flanking side markers
   (coil_3 west/east = 33/34 for north-south travel) are visible -- the
   combination that gave the best alignment odds in the physical
   experiments. Fine pair alignment can then follow separately.

Run on the robot (needs ROS2 for /cmd_vel unless --dry-run):

    source /opt/ros/humble/setup.bash
    ros2 launch turtlebot3_bringup robot.launch.py &
    python3 scripts/drive_between_coils.py --from-coil coil_1 --to-coil coil_3

    # camera/steering check without ROS2 or motors:
    python3 scripts/drive_between_coils.py --from-coil coil_1 --to-coil coil_3 --dry-run
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from wpt_adjustment_turtlebot.coil_transit import compute_line_follow_cmd, plan_transit_legs, stop_condition_met  # noqa: E402
from wpt_adjustment_turtlebot.controller_math import VelocityCommand  # noqa: E402
from wpt_adjustment_turtlebot.line_detection import LineDetector, LineObservation  # noqa: E402
from wpt_adjustment_turtlebot.sensor_fusion import ErrorKalmanFilter  # noqa: E402
from wpt_adjustment_turtlebot.wpt_alignment_node import AprilTagDetector  # noqa: E402

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node as RosNode
except Exception:  # pragma: no cover - dry-run works without ROS2
    rclpy = None
    Twist = None
    RosNode = object


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default=str(REPO_ROOT / "config" / "wpt_alignment.yaml"))
    parser.add_argument("--from-coil", required=True, help="departure coil (coil_1..coil_4, or 1..4)")
    parser.add_argument("--to-coil", required=True, help="target coil (coil_1..coil_4, or 1..4)")
    parser.add_argument("--dry-run", action="store_true", help="log commands instead of publishing /cmd_vel")
    parser.add_argument("--max-leg-sec", type=float, default=60.0, help="abort a leg after this long without the stop marker")
    return parser.parse_args()


def load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def open_camera(name: str, cfg: dict) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(int(cfg["device"]), cv2.CAP_V4L2)
    if cfg.get("fourcc"):
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*str(cfg["fourcc"])[:4]))
    if cfg.get("width"):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cfg["width"]))
    if cfg.get("height"):
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cfg["height"]))
    if cfg.get("fps"):
        cap.set(cv2.CAP_PROP_FPS, float(cfg["fps"]))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        raise RuntimeError(f"camera {name}: cannot open /dev/video{cfg['device']}")
    return cap


def read_frames(cameras: dict[str, cv2.VideoCapture]) -> dict[str, object]:
    grabbed = {name: cap.grab() for name, cap in cameras.items()}
    frames: dict[str, object] = {}
    for name, cap in cameras.items():
        if not grabbed.get(name, False):
            continue
        ok, frame = cap.retrieve()
        if ok:
            frames[name] = frame
    return frames


def fuse_side_lines(left: LineObservation | None, right: LineObservation | None) -> tuple[float, float, float] | None:
    """Confidence-weighted (x_error, angle_error_deg, confidence) from the side cameras."""
    observations = [obs for obs in (left, right) if obs is not None]
    if not observations:
        return None
    total = sum(max(0.01, obs.confidence) for obs in observations)
    x = sum(obs.x_error * max(0.01, obs.confidence) for obs in observations) / total
    angle = sum(obs.angle_error_deg * max(0.01, obs.confidence) for obs in observations) / total
    return x, angle, min(1.0, total / len(observations))


class CmdPublisher:
    """Publishes /cmd_vel through ROS2, or just logs in dry-run."""

    def __init__(self, dry_run: bool) -> None:
        self.dry_run = dry_run
        self.node = None
        if not dry_run:
            if rclpy is None:
                raise RuntimeError("ROS2 rclpy is unavailable; use --dry-run or run inside a ROS2 environment")
            rclpy.init()
            self.node = rclpy.create_node("drive_between_coils")
            self.pub = self.node.create_publisher(Twist, "/cmd_vel", 10)

    def publish(self, cmd: VelocityCommand) -> None:
        if self.dry_run:
            return
        msg = Twist()
        msg.linear.x = float(cmd.linear_x)
        msg.angular.z = float(cmd.angular_z)
        self.pub.publish(msg)

    def stop(self) -> None:
        self.publish(VelocityCommand())

    def shutdown(self) -> None:
        if self.node is not None:
            self.stop()
            self.node.destroy_node()
            rclpy.shutdown()


def drive_leg(
    leg,
    cameras: dict[str, cv2.VideoCapture],
    tag_detector: AprilTagDetector,
    line_detector: LineDetector,
    kalman: ErrorKalmanFilter,
    publisher: CmdPublisher,
    config: dict,
    max_leg_sec: float,
) -> bool:
    """One straight leg, in two phases keyed to the measured leg distance.

    CRUISE   -- Kalman line following at cruise speed for transit.cruise_fraction
                of the measured coil-center distance (dead-reckoned from the
                commanded linear speed).
    APPROACH -- slow down and hunt for the arrival condition: the target coil's
                BOTH flanking side markers visible (any camera, with a small
                freshness window) AND its head/travel-direction marker seen by
                the front camera -> stop that instant. This combination gave
                the highest alignment success in the physical experiments.

    The arrival condition is checked in both phases (marker IDs are unique per
    coil, so an early sighting can only be the real target).
    """
    follow = config.get("line_follow", {})
    transit = config.get("transit", {})
    loop_hz = float(follow.get("loop_hz", 10.0))
    line_lost_timeout = float(follow.get("line_lost_timeout_sec", 1.0))
    cruise_fraction = float(transit.get("cruise_fraction", 0.6))
    approach_linear = float(transit.get("approach_linear", 0.02))
    side_freshness = float(transit.get("side_marker_freshness_sec", 0.7))
    cruise_linear = float(follow.get("cruise_linear", 0.05))
    steer_params = {
        "k_x_to_angular": float(follow.get("k_x_to_angular", -0.003)),
        "k_angle_to_angular": float(follow.get("k_angle_to_angular", -0.010)),
        "max_angular": float(follow.get("max_angular", 0.3)),
        "x_deadband_px": float(follow.get("x_deadband_px", 3.0)),
        "angle_deadband_deg": float(follow.get("angle_deadband_deg", 1.0)),
        "invert_angular": bool(follow.get("invert_angular", False)),
    }

    cruise_distance = leg.distance_m * cruise_fraction
    print(
        f"[leg] {leg.from_coil} -> {leg.to_coil} direction={leg.direction} distance={leg.distance_m:.3f}m "
        f"(cruise {cruise_distance:.3f}m then approach) departure_head={leg.head_marker_id} "
        f"arrival_head={leg.stop_marker_id} side_markers={leg.side_marker_ids}"
    )
    started = time.monotonic()
    last_tick = started
    last_line_seen = started
    distance_driven = 0.0
    side_last_seen: dict[int, float] = {}

    while True:
        now = time.monotonic()
        if now - started > max_leg_sec:
            publisher.stop()
            print(f"[leg] TIMEOUT after {max_leg_sec:.0f}s without arrival condition; stopped")
            return False

        frames = read_frames(cameras)

        # 1) Arrival check. Side markers may appear on any camera (physically
        #    the side-bottom ones); the head marker must be on the front camera.
        head_visible = False
        for camera_name, frame in frames.items():
            for det in tag_detector.detect(frame):
                if det.tag_id in leg.side_marker_ids:
                    side_last_seen[det.tag_id] = now
                if det.tag_id == leg.stop_marker_id and camera_name == "front":
                    head_visible = True
                elif det.tag_id == leg.head_marker_id and camera_name == "front":
                    print(f"[leg] passing departure head marker {leg.head_marker_id}")
        if stop_condition_met(now, head_visible, side_last_seen, leg.side_marker_ids, side_freshness):
            publisher.stop()
            print(
                f"[leg] arrival: head marker {leg.stop_marker_id} + side markers {leg.side_marker_ids} "
                f"-> STOP over {leg.to_coil} (driven ~{distance_driven:.3f}m of {leg.distance_m:.3f}m)"
            )
            return True
        if head_visible:
            print(f"[leg] head marker {leg.stop_marker_id} visible but side markers not fresh; keep approaching")

        # 2) Phase by dead-reckoned distance: full speed for the known straight
        #    stretch, then slow so the stop lands as close to center as possible.
        phase = "cruise" if distance_driven < cruise_distance else "approach"
        linear_speed = cruise_linear if phase == "cruise" else approach_linear

        # 3) Line following on the side cameras, Kalman-smoothed, so heading
        #    drift is corrected continuously instead of accumulating.
        left_obs = line_detector.detect(frames["left_bottom"]) if "left_bottom" in frames else None
        right_obs = line_detector.detect(frames["right_bottom"]) if "right_bottom" in frames else None
        fused = fuse_side_lines(left_obs, right_obs)
        if fused is not None:
            x_error, angle_error, confidence = fused
            estimate = kalman.update(x_error=x_error, y_error=0.0, angle_error_deg=angle_error, confidence=confidence)
            last_line_seen = now
        else:
            estimate = kalman.predict()
            if now - last_line_seen > line_lost_timeout:
                publisher.stop()
                print(f"[leg] line lost for >{line_lost_timeout:.1f}s; stopped for safety")
                return False

        cmd = compute_line_follow_cmd(estimate.x_error, estimate.angle_error_deg, cruise_linear=linear_speed, **steer_params)
        publisher.publish(cmd)
        distance_driven += cmd.linear_x * (now - last_tick)
        last_tick = now
        cameras_seen = ("L" if left_obs else "-") + ("R" if right_obs else "-")
        sides_fresh = sum(1 for m in leg.side_marker_ids if now - side_last_seen.get(m, float("-inf")) <= side_freshness)
        print(
            f"t={now - started:6.2f}s phase={phase} dist={distance_driven:.3f}m x_err={estimate.x_error:7.2f} "
            f"angle_err={estimate.angle_error_deg:6.2f} line_cams={cameras_seen} sides_fresh={sides_fresh}/2 "
            f"cmd linear={cmd.linear_x:.3f} angular={cmd.angular_z:.3f}"
        )
        time.sleep(1.0 / loop_hz)


def main() -> int:
    args = parse_args()
    config = load_config(args.config)

    legs = plan_transit_legs(args.from_coil, args.to_coil)
    if not legs:
        print("already at the target coil; nothing to do")
        return 0
    print(f"transit plan: {' -> '.join([legs[0].from_coil] + [leg.to_coil for leg in legs])}")

    detector_cfg = config.get("line_follow", {}).get("detector", {})
    line_detector = LineDetector(
        threshold=int(detector_cfg.get("threshold", 80)),
        min_area_px=float(detector_cfg.get("min_area_px", 120.0)),
        roi_top_ratio=float(detector_cfg.get("roi_top_ratio", 0.0)),
        roi_bottom_ratio=float(detector_cfg.get("roi_bottom_ratio", 1.0)),
    )
    kalman_cfg = config.get("line_follow", {}).get("kalman", {})
    tag_detector = AprilTagDetector(config["apriltag"].get("family", "tag36h11"))
    print(f"AprilTag backend: {tag_detector.backend}")

    cameras = {name: open_camera(name, cfg) for name, cfg in config["cameras"].items()}
    publisher = CmdPublisher(dry_run=args.dry_run)
    if args.dry_run:
        print("dry_run=True: /cmd_vel will NOT be published")

    try:
        for index, leg in enumerate(legs):
            # Fresh filter per leg: heading errors from the previous leg's
            # 90-degree turn shouldn't leak into the new straight.
            kalman = ErrorKalmanFilter(
                process_variance=float(kalman_cfg.get("process_variance", 1.0)),
                measurement_variance=float(kalman_cfg.get("measurement_variance", 4.0)),
            )
            ok = drive_leg(leg, cameras, tag_detector, line_detector, kalman, publisher, config, args.max_leg_sec)
            if not ok:
                return 1
            if index < len(legs) - 1:
                # Diagonal transits need a 90-degree turn between legs. That
                # turn is not automated yet (rotating blind can occlude the
                # tape/markers), so stop here and let the operator re-run for
                # the second leg after turning.
                next_leg = legs[index + 1]
                print(
                    f"[transit] stopped at {leg.to_coil}. Rotate to face {next_leg.direction}, then run: "
                    f"python3 scripts/drive_between_coils.py --from-coil {next_leg.from_coil} --to-coil {next_leg.to_coil}"
                )
                return 0
        print("[transit] arrived; run fine pair alignment next (wpt_alignment_node or check_camera_alignment.py)")
        return 0
    finally:
        publisher.stop()
        for cap in cameras.values():
            cap.release()
        publisher.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
