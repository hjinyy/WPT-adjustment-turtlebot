"""ROS2 node for AprilTag based WPT coil alignment."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from pathlib import Path
from typing import Any

import cv2
import yaml

from .controller_math import (
    AlignmentState,
    TagObservation,
    TagPairObservation,
    TargetPoseInImage,
    VelocityCommand,
    compute_alignment_cmd,
    compute_pair_alignment_error,
    compute_pair_observation,
    is_aligned,
)
from .tag_layout import coil_pair_ids

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
except Exception:  # pragma: no cover
    rclpy = None
    Node = object
    Twist = None

try:
    from pupil_apriltags import Detector as PupilAprilTagDetector
except Exception:  # pragma: no cover
    PupilAprilTagDetector = None


@dataclass
class Detection:
    tag_id: int
    center: tuple[float, float]
    corners: list[tuple[float, float]]
    angle_deg: float
    area_px: float


class AprilTagDetector:
    def __init__(self, family: str = "tag36h11") -> None:
        self.backend = "none"
        self.detector = None
        if PupilAprilTagDetector is not None:
            self.detector = PupilAprilTagDetector(families=family)
            self.backend = "pupil_apriltags"
        elif hasattr(cv2, "aruco") and hasattr(cv2.aruco, "ArucoDetector"):
            dictionary_id = getattr(cv2.aruco, "DICT_APRILTAG_36h11", None)
            if dictionary_id is not None:
                dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
                params = cv2.aruco.DetectorParameters()
                self.detector = cv2.aruco.ArucoDetector(dictionary, params)
                self.backend = "opencv_aruco"

    def detect(self, frame_bgr) -> list[Detection]:
        if self.detector is None:
            raise RuntimeError("Install pupil-apriltags or OpenCV aruco AprilTag support.")
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        if self.backend == "pupil_apriltags":
            return [self._from_pupil(d) for d in self.detector.detect(gray)]
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return []
        return [self._from_corners(int(i[0]), [(float(x), float(y)) for x, y in c.reshape(-1, 2)]) for c, i in zip(corners, ids)]

    @staticmethod
    def _from_pupil(d) -> Detection:
        corners = [(float(x), float(y)) for x, y in d.corners]
        return Detection(int(d.tag_id), (float(d.center[0]), float(d.center[1])), corners, _angle_from_corners(corners), _polygon_area(corners))

    @staticmethod
    def _from_corners(tag_id: int, corners: list[tuple[float, float]]) -> Detection:
        cx = sum(p[0] for p in corners) / 4.0
        cy = sum(p[1] for p in corners) / 4.0
        return Detection(tag_id, (cx, cy), corners, _angle_from_corners(corners), _polygon_area(corners))


def _angle_from_corners(corners: list[tuple[float, float]]) -> float:
    dx = corners[1][0] - corners[0][0]
    dy = corners[1][1] - corners[0][1]
    return math.degrees(math.atan2(dy, dx))


def _polygon_area(corners: list[tuple[float, float]]) -> float:
    area = 0.0
    for i, (x1, y1) in enumerate(corners):
        x2, y2 = corners[(i + 1) % len(corners)]
        area += x1 * y2 - x2 * y1
    return abs(area) * 0.5


class WptAlignmentNode(Node):
    def __init__(self) -> None:
        super().__init__("wpt_alignment_node")
        self.declare_parameter("config_file", "")
        self.declare_parameter("target_shelf", 1)
        self.declare_parameter("dry_run", None)
        config_file = self.get_parameter("config_file").get_parameter_value().string_value
        if not config_file:
            config_file = str(Path.cwd() / "config" / "wpt_alignment.yaml")
        self.config = _load_yaml(config_file)
        override_dry = self.get_parameter("dry_run").value
        if override_dry is not None:
            self.config["safety"]["dry_run"] = bool(override_dry)
        self.target_shelf = int(self.get_parameter("target_shelf").value)
        self.state = AlignmentState.ENTER_SHELF
        self.stable_count = 0
        self.last_seen_time = time.monotonic()
        self.active_coil_camera = None
        self.detector = AprilTagDetector(self.config["apriltag"].get("family", "tag36h11"))
        self.get_logger().info(f"AprilTag backend: {self.detector.backend}")
        self.cameras = self._open_cameras()
        self.cmd_pub = self.create_publisher(Twist, self.config["ros"].get("cmd_vel_topic", "/cmd_vel"), 10)
        self.timer = self.create_timer(1.0 / float(self.config["control"].get("loop_hz", 10.0)), self.step)

    def _open_cameras(self) -> dict[str, cv2.VideoCapture]:
        cameras = {}
        for name, cfg in self.config["cameras"].items():
            cap = cv2.VideoCapture(cfg["device"])
            if cfg.get("width"):
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cfg["width"]))
            if cfg.get("height"):
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cfg["height"]))
            if not cap.isOpened():
                self.get_logger().warning(f"Camera {name} device {cfg['device']} is not open")
            cameras[name] = cap
        return cameras

    def step(self) -> None:
        try:
            observations = self.read_all_tags()
            cmd = self.update_state_and_compute_cmd(observations)
            self.publish_cmd(cmd)
        except Exception as exc:
            self.get_logger().error(f"controller error: {exc}")
            self.publish_cmd(VelocityCommand())
            self.state = AlignmentState.ERROR

    def read_all_tags(self) -> list[TagObservation]:
        observations: list[TagObservation] = []
        for cam_name, cap in self.cameras.items():
            ok, frame = cap.read()
            if not ok:
                continue
            for det in self.detector.detect(frame):
                observations.append(TagObservation(det.tag_id, det.center[0], det.center[1], det.angle_deg, det.area_px, cam_name))
        if observations:
            self.last_seen_time = time.monotonic()
        return observations

    def update_state_and_compute_cmd(self, obs: list[TagObservation]) -> VelocityCommand:
        if self._tag_lost_too_long():
            self.get_logger().warning("AprilTag lost timeout; stopping")
            return VelocityCommand()
        if self.state == AlignmentState.ENTER_SHELF:
            pair_name = self._final_pair_name()
            pair = self._find_target_pair(obs, pair_name)
            if pair:
                self.active_coil_camera = pair.camera_name
                self.state = AlignmentState.ALIGN_COIL
                return VelocityCommand()
            if self._has_any_pair_marker(obs, pair_name):
                return VelocityCommand()
            return VelocityCommand(linear_x=float(self.config["speed"]["corridor_linear"]))
        if self.state == AlignmentState.ALIGN_COIL:
            pair_name = self._final_pair_name()
            pair = self._find_target_pair(obs, pair_name, self.active_coil_camera)
            if not pair:
                self.stable_count = 0
                return VelocityCommand()
            err = compute_pair_alignment_error(pair, self._target_for(pair.camera_name, pair_name))
            if is_aligned(err, **self._thresholds("coil")):
                self.stable_count += 1
                if self.stable_count >= int(self.config["alignment"]["stable_frames_required"]):
                    self.state = AlignmentState.FINAL_STOP
                    return VelocityCommand()
            else:
                self.stable_count = 0
            return compute_alignment_cmd(err, **self._control_params("coil"))
        if self.state == AlignmentState.FINAL_STOP:
            self.state = AlignmentState.CHARGING
            self.get_logger().info("WPT charging trigger should be enabled here")
            return VelocityCommand()
        return VelocityCommand()

    def publish_cmd(self, cmd: VelocityCommand) -> None:
        if self.config["safety"].get("dry_run", True):
            self.get_logger().info(f"dry_run state={self.state.value} cmd linear.x={cmd.linear_x:.4f}, angular.z={cmd.angular_z:.4f}")
            return
        msg = Twist()
        msg.linear.x = float(cmd.linear_x)
        msg.angular.z = float(cmd.angular_z)
        self.cmd_pub.publish(msg)

    def _find(self, obs: list[TagObservation], tag_id: int, camera: str | None = None) -> TagObservation | None:
        candidates = [o for o in obs if o.tag_id == tag_id and (camera is None or o.camera_name == camera)]
        return max(candidates, key=lambda o: o.area_px, default=None)

    def _find_target_pair(
        self, obs: list[TagObservation], pair_name: str, preferred_camera: str | None = None
    ) -> TagPairObservation | None:
        first_id, second_id = self._pair_ids(pair_name)
        camera_names = sorted({o.camera_name for o in obs if o.camera_name})
        if preferred_camera:
            camera_names = [preferred_camera]
        best_pair = None
        best_area = -1.0
        for camera_name in camera_names:
            first = self._find(obs, first_id, camera=camera_name)
            second = self._find(obs, second_id, camera=camera_name)
            if first is None or second is None:
                continue
            area = first.area_px + second.area_px
            if area > best_area:
                best_pair = compute_pair_observation(first, second)
                best_area = area
        return best_pair

    def _has_any_pair_marker(self, obs: list[TagObservation], pair_name: str) -> bool:
        first_id, second_id = self._pair_ids(pair_name)
        return any(o.tag_id in {first_id, second_id} for o in obs)

    def _final_pair_name(self) -> str:
        return str(self.config["alignment"].get("final_pair", "west_east"))

    def _pair_ids(self, pair_name: str) -> tuple[int, int]:
        return coil_pair_ids(self.target_shelf, pair_name)

    def _target_for(self, camera: str, tag_key: str) -> TargetPoseInImage:
        shelf_cfg = self.config["shelves"][str(self.target_shelf)]
        raw = shelf_cfg["targets"][camera].get(tag_key, shelf_cfg["targets"][camera]["default"])
        return TargetPoseInImage(float(raw["x"]), float(raw["y"]), float(raw["angle_deg"]))

    def _thresholds(self, kind: str) -> dict[str, float]:
        cfg = self.config["alignment"][kind]
        return {"threshold_x_px": float(cfg["threshold_x_px"]), "threshold_y_px": float(cfg["threshold_y_px"]), "threshold_angle_deg": float(cfg["threshold_angle_deg"])}

    def _control_params(self, kind: str) -> dict[str, Any]:
        c = self.config["control"][kind]
        s = self.config["speed"]
        return {
            "k_y_to_linear": float(c["k_y_to_linear"]),
            "k_x_to_angular": float(c["k_x_to_angular"]),
            "k_angle_to_angular": float(c["k_angle_to_angular"]),
            "max_linear": float(s[f"{kind}_max_linear"]),
            "max_angular": float(s[f"{kind}_max_angular"]),
            "min_linear": float(s.get(f"{kind}_min_linear", 0.0)),
            "min_angular": float(s.get(f"{kind}_min_angular", 0.0)),
            "x_deadband_px": float(c.get("x_deadband_px", 0.0)),
            "y_deadband_px": float(c.get("y_deadband_px", 0.0)),
            "angle_deadband_deg": float(c.get("angle_deadband_deg", 0.0)),
            "invert_linear": bool(c.get("invert_linear", False)),
            "invert_angular": bool(c.get("invert_angular", False)),
        }

    def _tag_lost_too_long(self) -> bool:
        timeout = float(self.config["safety"].get("tag_lost_timeout_sec", 2.0))
        return (time.monotonic() - self.last_seen_time) > timeout and self.state not in {AlignmentState.ENTER_SHELF, AlignmentState.CHARGING}


def _load_yaml(path: str) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def main(args=None) -> None:
    if rclpy is None:
        raise RuntimeError("ROS2 rclpy is not available. Run this node inside a ROS2 environment.")
    rclpy.init(args=args)
    node = WptAlignmentNode()
    try:
        rclpy.spin(node)
    finally:
        node.publish_cmd(VelocityCommand())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
