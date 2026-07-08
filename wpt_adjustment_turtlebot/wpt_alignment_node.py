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
    TagPairObservation,
    TagObservation,
    TargetPoseInImage,
    VelocityCommand,
    compute_alignment_cmd,
    compute_alignment_error,
    compute_pair_alignment_error,
    compute_pair_observation,
    is_aligned,
)
from .tag_layout import coil_pair_ids, four_coil_pair_ids, head_tag_id, station_pair_ids

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
        elif hasattr(cv2, "aruco") and hasattr(cv2.aruco, "detectMarkers"):
            dictionary_id = getattr(cv2.aruco, "DICT_APRILTAG_36h11", None)
            if dictionary_id is not None:
                self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
                if hasattr(cv2.aruco, "DetectorParameters_create"):
                    self.params = cv2.aruco.DetectorParameters_create()
                else:
                    self.params = cv2.aruco.DetectorParameters()
                self.detector = cv2.aruco
                self.backend = "opencv_aruco_legacy"

    def detect(self, frame_bgr) -> list[Detection]:
        if self.detector is None:
            raise RuntimeError("Install pupil-apriltags or OpenCV aruco AprilTag support.")
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        if self.backend == "pupil_apriltags":
            return [self._from_pupil(d) for d in self.detector.detect(gray)]
        if self.backend == "opencv_aruco_legacy":
            corners, ids, _ = self.detector.detectMarkers(gray, self.dictionary, parameters=self.params)
        else:
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
        self.declare_parameter("target_station", "")
        self.declare_parameter("target_coil", "")
        self.declare_parameter("dry_run", None)
        config_file = self.get_parameter("config_file").get_parameter_value().string_value
        if not config_file:
            config_file = str(Path.cwd() / "config" / "wpt_alignment.yaml")
        self.config = _load_yaml(config_file)
        override_dry = self.get_parameter("dry_run").value
        if override_dry is not None:
            self.config["safety"]["dry_run"] = bool(override_dry)
        override_station = self.get_parameter("target_station").get_parameter_value().string_value
        if override_station:
            self.config["target_station"] = override_station
            self.config["layout_mode"] = "station_map"
        override_coil = self.get_parameter("target_coil").get_parameter_value().string_value
        if override_coil:
            self.config["target_coil"] = override_coil
            self.config["layout_mode"] = "four_coil_map"
        self.target_shelf = int(self.get_parameter("target_shelf").value)
        self.layout_mode = str(self.config.get("layout_mode", "four_coil_map"))
        self.target_station = str(self.config.get("target_station", "A02")).upper()
        self.target_coil = str(self.config.get("target_coil", "coil_1")).lower()
        self.state = AlignmentState.ALIGN_COIL if self._starts_at_alignment() else AlignmentState.SEARCH_HEAD_TAG
        self.stable_count = 0
        self.last_seen_time = time.monotonic()
        self.active_coil_camera = None
        self.detector = AprilTagDetector(self.config["apriltag"].get("family", "tag36h11"))
        self.get_logger().info(f"AprilTag backend: {self.detector.backend}")
        self.get_logger().info(f"layout_mode={self.layout_mode} target={self._target_name_for_log()}")
        self.cameras = self._open_cameras()
        self.cmd_pub = self.create_publisher(Twist, self.config["ros"].get("cmd_vel_topic", "/cmd_vel"), 10)
        self.timer = self.create_timer(1.0 / float(self.config["control"].get("loop_hz", 10.0)), self.step)

    def _open_cameras(self) -> dict[str, cv2.VideoCapture]:
        cameras = {}
        for name, cfg in self.config["cameras"].items():
            cap = cv2.VideoCapture(cfg["device"], cv2.CAP_V4L2)
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
        grabbed = {cam_name: cap.grab() for cam_name, cap in self.cameras.items()}
        for cam_name, cap in self.cameras.items():
            if not grabbed.get(cam_name, False):
                self.get_logger().warning(f"Camera {cam_name} grab failed")
                continue
            ok, frame = cap.retrieve()
            if not ok:
                self.get_logger().warning(f"Camera {cam_name} retrieve failed")
                continue
            for det in self.detector.detect(frame):
                observations.append(TagObservation(det.tag_id, det.center[0], det.center[1], det.angle_deg, det.area_px, cam_name))
        if observations:
            self.last_seen_time = time.monotonic()
            self._log_detected_tags(observations)
        return observations

    def update_state_and_compute_cmd(self, obs: list[TagObservation]) -> VelocityCommand:
        if self._tag_lost_too_long():
            self.get_logger().warning("AprilTag lost timeout; stopping")
            return VelocityCommand()
        if self.state == AlignmentState.SEARCH_HEAD_TAG:
            head_obs = self._find(obs, head_tag_id(self.target_shelf), camera="front")
            if head_obs:
                self.state = AlignmentState.APPROACH_SHELF
                self.get_logger().info("target head tag found")
                return VelocityCommand()
            return VelocityCommand(linear_x=float(self.config["speed"]["corridor_linear"]))
        if self.state == AlignmentState.APPROACH_SHELF:
            head_obs = self._find(obs, head_tag_id(self.target_shelf), camera="front")
            if not head_obs:
                return VelocityCommand()
            err = compute_alignment_error(head_obs, self._target_for("front", "head"))
            if is_aligned(err, **self._thresholds("head")):
                self.state = AlignmentState.ENTER_SHELF
                return VelocityCommand()
            return compute_alignment_cmd(err, **self._control_params("head"))
        if self.state == AlignmentState.ENTER_SHELF:
            pair_name = self._final_pair_name()
            pair = self._find_target_pair(obs, pair_name)
            if pair:
                self._log_pair_alignment(pair_name, pair, None)
                self.active_coil_camera = pair.camera_name
                self.state = AlignmentState.ALIGN_COIL
                return VelocityCommand()
            if self._has_any_pair_marker(obs, pair_name):
                self._log_pair_missing(pair_name, obs, self.active_coil_camera)
                return VelocityCommand()
            return VelocityCommand(linear_x=float(self.config["speed"]["shelf_entry_linear"]))
        if self.state == AlignmentState.ALIGN_COIL:
            pair_name = self._final_pair_name()
            pair = self._find_target_pair(obs, pair_name, self.active_coil_camera)
            if not pair:
                self.stable_count = 0
                self._log_pair_missing(pair_name, obs, self.active_coil_camera)
                return VelocityCommand()
            err = compute_pair_alignment_error(pair, self._target_for(pair.camera_name, pair_name))
            if is_aligned(err, **self._thresholds("coil")):
                self.stable_count += 1
                if self.stable_count >= int(self.config["alignment"]["stable_frames_required"]):
                    self._log_pair_alignment(pair_name, pair, err)
                    self.state = AlignmentState.FINAL_STOP
                    return VelocityCommand()
            else:
                self.stable_count = 0
            self._log_pair_alignment(pair_name, pair, err)
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

    def _find_any_target_coil_tag(self, obs: list[TagObservation], preferred_camera: str | None = None) -> TagObservation | None:
        target_ids = set(self._pair_ids("north_south") + self._pair_ids("west_east"))
        candidates = [o for o in obs if o.tag_id in target_ids]
        if preferred_camera:
            preferred = [o for o in candidates if o.camera_name == preferred_camera]
            if preferred:
                candidates = preferred
        return max(candidates, key=lambda o: o.area_px, default=None)

    def _find_target_pair(self, obs: list[TagObservation], pair_name: str, preferred_camera: str | None = None):
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

    def _target_for(self, camera: str, tag_key: str) -> TargetPoseInImage:
        if self._is_four_coil_map():
            coils = self.config["coils"]
            target_cfg = coils.get(self.target_coil, coils.get("default", coils["coil_1"]))
        elif self._is_station_map():
            stations = self.config["stations"]
            target_cfg = stations.get(self.target_station, stations.get("default", stations["A02"]))
        else:
            target_cfg = self.config["shelves"][str(self.target_shelf)]
        raw = target_cfg["targets"][camera].get(tag_key, target_cfg["targets"][camera]["default"])
        return TargetPoseInImage(float(raw["x"]), float(raw["y"]), float(raw["angle_deg"]))

    def _final_pair_name(self) -> str:
        return str(self.config["alignment"].get("final_pair", "west_east"))

    def _is_station_map(self) -> bool:
        return self.layout_mode == "station_map"

    def _is_four_coil_map(self) -> bool:
        return self.layout_mode == "four_coil_map"

    def _starts_at_alignment(self) -> bool:
        return self._is_station_map() or self._is_four_coil_map()

    def _target_name_for_log(self) -> str:
        if self._is_four_coil_map():
            return self.target_coil
        if self._is_station_map():
            return self.target_station
        return f"shelf_{self.target_shelf}"

    def _pair_ids(self, pair_name: str) -> tuple[int, int]:
        if self._is_four_coil_map():
            return four_coil_pair_ids(self.target_coil, pair_name)
        if self._is_station_map():
            return station_pair_ids(self.target_station, pair_name)
        return coil_pair_ids(self.target_shelf, pair_name)

    def _log_detected_tags(self, obs: list[TagObservation]) -> None:
        tags = ", ".join(
            f"id={o.tag_id} cam={o.camera_name} center=({o.center_x:.1f},{o.center_y:.1f}) angle={o.angle_deg:.1f}"
            for o in obs
        )
        self.get_logger().info(
            f"calib layout_mode={self.layout_mode} target={self._target_name_for_log()} "
            f"state={self.state.value} detected_tags=[{tags}]"
        )

    def _log_pair_missing(self, pair_name: str, obs: list[TagObservation], camera: str | None = None) -> None:
        first_id, second_id = self._pair_ids(pair_name)
        visible = [o for o in obs if o.tag_id in {first_id, second_id} and (camera is None or o.camera_name == camera)]
        visible_text = ", ".join(
            f"id={o.tag_id} cam={o.camera_name} center=({o.center_x:.1f},{o.center_y:.1f})" for o in visible
        )
        self.get_logger().info(
            f"calib layout_mode={self.layout_mode} target={self._target_name_for_log()} state={self.state.value} "
            f"selected_pair={pair_name} required_ids=({first_id},{second_id}) "
            f"pair_missing visible_marker_ids=[{visible_text}] stable_count={self.stable_count}"
        )

    def _log_pair_alignment(self, pair_name: str, pair: TagPairObservation, err) -> None:
        first_id, second_id = self._pair_ids(pair_name)
        err_text = "x_error=NA y_error=NA angle_error=NA"
        if err is not None:
            err_text = f"x_error={err.x:.2f} y_error={err.y:.2f} angle_error={err.angle_deg:.2f}"
        self.get_logger().info(
            f"calib layout_mode={self.layout_mode} target={self._target_name_for_log()} state={self.state.value} "
            f"selected_pair={pair_name} required_ids=({first_id},{second_id}) camera={pair.camera_name} "
            f"marker_a=id={pair.first.tag_id} center=({pair.first.center_x:.1f},{pair.first.center_y:.1f}) "
            f"marker_b=id={pair.second.tag_id} center=({pair.second.center_x:.1f},{pair.second.center_y:.1f}) "
            f"pair_mid_x={pair.midpoint_x:.2f} pair_mid_y={pair.midpoint_y:.2f} "
            f"pair_angle_deg={pair.pair_angle_deg:.2f} {err_text} stable_count={self.stable_count}"
        )

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
        return (time.monotonic() - self.last_seen_time) > timeout and self.state not in {AlignmentState.SEARCH_HEAD_TAG, AlignmentState.CHARGING}


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
