"""전역 AprilTag 지도, 라인 추종, 최종 코일 정합 ROS2 노드."""
from __future__ import annotations

import time
from pathlib import Path

import cv2
import yaml

from .controller_math import (
    AlignmentError,
    TagObservation,
    TargetPoseInImage,
    VelocityCommand,
    blend_marker_and_line_cmd,
    block_reverse_linear_cmd,
    compute_alignment_cmd,
    compute_pair_alignment_error,
    compute_pair_observation,
    is_undershoot_aligned,
)
from .global_map import TAG_SIZE_M, build_tag_world_poses, nearest_coil, plan_axis_aligned_route
from .global_route_control import RouteFollower
from .line_detection import LineDetector
from .map_localization import MapPose, MapPoseEKF, estimate_map_pose_from_tag_corners
from .run_logging import RunLogger
from .sensor_fusion import ErrorKalmanFilter
from .tag_layout import four_coil_pair_ids
from .wpt_alignment_node import AprilTagDetector

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_srvs.srv import Trigger
except ImportError:  # pragma: no cover
    rclpy = None
    Node = object


class LegacyGlobalMapNavigator(Node):
    def __init__(self) -> None:
        super().__init__("global_map_navigation")
        self.declare_parameter("config_file", "")
        self.declare_parameter("target_coil", "coil_3")
        self.declare_parameter("dry_run", True)
        self.declare_parameter("log_root", "")

        config_path = self.get_parameter("config_file").value or str(Path.cwd() / "config" / "wpt_alignment.yaml")
        self.cfg = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))
        self.map_cfg = self.cfg["map_localization"]
        self.target_coil = str(self.get_parameter("target_coil").value).lower()
        self.dry_run = bool(self.get_parameter("dry_run").value)
        if self.target_coil not in self.cfg["coils"]:
            raise ValueError(f"지원하지 않는 목표 코일: {self.target_coil}")
        log_root = self.get_parameter("log_root").value or str(Path(config_path).resolve().parent.parent / "logs")
        self.run_log = RunLogger(log_root, target_coil=self.target_coil)

        self.state = "IDLE"
        self.started = False
        self.last_pose_time = 0.0
        self.last_step_time = time.monotonic()
        self.last_cmd = VelocityCommand()
        self.route: RouteFollower | None = None
        self.stable_count = 0
        self.align_coil_missed_frames = 0
        self.latest_detections = []
        self.latest_raw_pose = None
        self.latest_filtered_pose = None
        self.latest_line = None
        self.odom_linear_x = None
        self.odom_angular_z = None
        self._logged_state = None
        self.align_perp_started_at = 0.0
        self.target_coil_seen = False
        self._line_seen: bool | None = None
        self._alignment_markers_visible: bool | None = None
        self._start_coil: str | None = None

        fusion = self.cfg.get("fusion", {})
        self.pose_filter = MapPoseEKF(
            process_variance=float(self.map_cfg.get("process_variance", 0.002)),
            measurement_variance=float(self.map_cfg.get("measurement_variance", 0.01)),
            outlier_distance_m=float(self.map_cfg.get("outlier_distance_m", 0.20)),
        )
        self.line_filter = ErrorKalmanFilter(
            process_variance=float(fusion.get("line_process_variance", 1.0)),
            measurement_variance=float(fusion.get("line_measurement_variance", 8.0)),
        )
        self.marker_filter = ErrorKalmanFilter(
            process_variance=float(fusion.get("marker_process_variance", 1.0)),
            measurement_variance=float(fusion.get("marker_measurement_variance", 4.0)),
        )
        line_cfg = self.cfg["line_tracking"]
        self.line_detector = LineDetector(
            threshold=int(line_cfg.get("threshold", 80)),
            min_area_px=float(line_cfg.get("min_area_px", 120)),
            roi_top_ratio=float(line_cfg.get("roi_top_ratio", 0.0)),
            roi_bottom_ratio=float(line_cfg.get("roi_bottom_ratio", 1.0)),
            blur_kernel=int(line_cfg.get("blur_kernel", 5)),
        )
        self.detector = AprilTagDetector(self.cfg["apriltag"].get("family", "tag36h11"))
        self.cameras = {}
        for name, cfg in self.cfg["cameras"].items():
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
            self.cameras[name] = cap
        self.cap = self.cameras[self.map_cfg.get("camera", "front")]
        self.tag_world_poses = build_tag_world_poses()

        self.pub = self.create_publisher(Twist, self.cfg["ros"].get("cmd_vel_topic", "/cmd_vel"), 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_service(Trigger, "/wpt_alignment/start", self.start)
        self.create_service(Trigger, "/wpt_alignment/stop", self.stop)
        self.timer = self.create_timer(1.0 / float(self.cfg["control"].get("loop_hz", 10.0)), self.step)
        message = f"대기 중: target={self.target_coil}, Enter/start 서비스 전에는 이동하지 않습니다"
        self.get_logger().info(message)
        self.run_log.event(f"노드 시작 target={self.target_coil} dry_run={self.dry_run}")
        self.run_log.event(f"로그 디렉터리 {self.run_log.run_dir}")

    def odom_callback(self, message) -> None:
        self.odom_linear_x = float(message.twist.twist.linear.x)
        self.odom_angular_z = float(message.twist.twist.angular.z)

    def calibrated(self) -> bool:
        return bool(self.map_cfg.get("enabled") and abs(float(self.map_cfg.get("tag_size_m", 0.0)) - TAG_SIZE_M) < 1e-9)

    def start(self, _request, response):
        if not self.calibrated():
            response.success, response.message = False, "전역 맵 또는 태그 크기 설정이 올바르지 않습니다"
            return response
        self.started = True
        self.state = "WAIT_FOR_START_MARKER"
        self.route = None
        self.stable_count = 0
        self._line_seen = None
        self._alignment_markers_visible = None
        self._start_coil = None
        self.last_pose_time = time.monotonic()
        self.run_log.event(f"주행 시작 target={self.target_coil}")
        response.success, response.message = True, f"현재 위치에서 {self.target_coil} 이동을 시작합니다"
        return response

    def stop(self, _request, response):
        self.started = False
        self.state = "IDLE"
        self.run_log.event("정지 서비스 요청")
        self.publish(VelocityCommand())
        response.success, response.message = True, "정지했습니다"
        return response

    def capture_observation(self) -> tuple[MapPose | None, float]:
        ok, frame = self.cap.read()
        if not ok:
            self.latest_detections = []
            self.latest_raw_pose = None
            self.latest_filtered_pose = None
            self.latest_line = None
            return None, 0.0
        tag_started = time.perf_counter()
        self.latest_detections = self.detector.detect(frame)
        self.tag_detect_ms = (time.perf_counter() - tag_started) * 1000.0
        corners = {d.tag_id: d.corners for d in self.latest_detections if d.tag_id in self.tag_world_poses}
        measured = estimate_map_pose_from_tag_corners(
            corners,
            frame_size=(frame.shape[1], frame.shape[0]),
            tag_world_poses=self.tag_world_poses,
            tag_size_m=float(self.map_cfg["tag_size_m"]),
        )
        self.latest_raw_pose = measured
        pose = self.pose_filter.update([measured]) if measured is not None else None
        self.latest_filtered_pose = pose
        line = self.line_detector.detect(frame) if self.cfg["line_tracking"].get("enabled", True) else None
        self.latest_line = line
        if line is None or line.confidence < float(self.cfg["line_tracking"].get("min_confidence", 0.1)):
            return pose, 0.0
        estimate = self.line_filter.update(
            x_error=line.x_error,
            y_error=0.0,
            angle_error_deg=line.angle_error_deg,
            confidence=line.confidence,
        )
        control = self.cfg["control"]["line"]
        line_angular = float(control["k_x_to_angular"]) * estimate.x_error + float(control["k_angle_to_angular"]) * estimate.angle_error_deg
        limit = float(self.cfg["speed"]["line_max_angular"])
        return pose, max(-limit, min(limit, line_angular))

    def _visible_start_coil(self) -> tuple[str | None, list[int]]:
        tag_ids = [int(obs.tag_id) for obs in getattr(self, "all_observations", [])]
        if not tag_ids:
            tag_ids = [int(det.tag_id) for det in self.latest_detections]
        counts: dict[int, int] = {}
        for tag_id in tag_ids:
            coil_number = tag_id // 10
            if 1 <= coil_number <= 4:
                counts[coil_number] = counts.get(coil_number, 0) + 1
        if not counts:
            return None, tag_ids
        coil_number = max(counts, key=counts.get)
        return f"coil_{coil_number}", tag_ids

    def _report_line_visibility(self, line_seen: bool) -> None:
        if self._line_seen is line_seen:
            return
        self._line_seen = line_seen
        message = "front line acquired; following line" if line_seen else "front line lost; robot stopped"
        self.get_logger().info(f"[wpt] {message}")
        self.run_log.event(message)

    def _report_alignment_marker_visibility(self, visible: bool) -> None:
        if self._alignment_markers_visible is visible:
            return
        self._alignment_markers_visible = visible
        message = "alignment markers ready" if visible else "alignment markers incomplete; robot stopped"
        self.get_logger().info(f"[wpt] {message}")
        self.run_log.event(message)

    def step(self) -> None:
        self._control_cycle_started = time.perf_counter()
        now = time.monotonic()
        self.last_step_time = now
        if not self.started:
            self.publish(VelocityCommand())
            return

        _measured, line_angular, line_seen = self.capture_observation()
        self._report_line_visibility(line_seen)
        if self.state != "ALIGN_COIL" and not line_seen:
            self.state = "LINE_LOST"
            self.publish(VelocityCommand())
            return

        if self.route is None:
            from .global_map import expected_route_marker_ids
            from .global_route_control import MarkerRouteGuide

            start_coil, visible_tag_ids = self._visible_start_coil()
            if start_coil is None:
                self.state = "WAIT_FOR_START_MARKER"
                self.publish(VelocityCommand())
                return
            if start_coil == self.target_coil:
                self.state = "ALIGN_COIL"
                self.get_logger().info(f"[wpt] start marker {visible_tag_ids}: already at {start_coil}")
                self.run_log.event(f"start coil={start_coil} markers={visible_tag_ids}")
                self.publish(self.final_alignment_command(0.0))
                return

            departure_id, goal_id = expected_route_marker_ids(start_coil, self.target_coil)
            self.route = RouteFollower(
                plan_axis_aligned_route(start_coil, self.target_coil),
                linear_speed=float(self.map_cfg.get("route_linear", 0.03)),
                max_angular=float(self.map_cfg.get("route_max_angular", 0.18)),
                turn_threshold_deg=float(self.map_cfg.get("heading_threshold_deg", 8.0)),
                waypoint_tolerance_m=float(self.map_cfg.get("preapproach_radius_m", 0.06)),
            )
            self.route_guide = MarkerRouteGuide(
                departure_id,
                goal_id,
                image_width=int(self.cfg["cameras"][self.map_cfg.get("camera", "front")].get("width", 320)),
                center_tolerance_px=float(self.map_cfg.get("route_marker_center_tolerance_px", 45)),
            )
            self._start_coil = start_coil
            self.route_started_at = now
            self.last_route_line_angular = 0.0
            self.state = "ACQUIRE_ROUTE"
            message = f"start coil={start_coil} markers={visible_tag_ids}; departure={departure_id}; goal={goal_id}"
            self.get_logger().info(f"[wpt] {message}")
            self.run_log.event(message)

        tag_ids = [det.tag_id for det in self.latest_detections]
        tag_centers = [(det.tag_id, det.center[0]) for det in self.latest_detections]
        self.route_guide.update_turn_direction(tag_ids)

        if self.state == "ACQUIRE_ROUTE":
            if self.route_guide.update_departure(tag_centers):
                self.state = "FOLLOW_LINE"
                message = "departure direction confirmed; line tracing started"
                self.get_logger().info(f"[wpt] {message}")
                self.run_log.event(message)
            elif now - self.route_started_at > float(self.map_cfg.get("acquire_timeout_sec", 12.0)):
                self.state = "ERROR"
                self.publish(VelocityCommand())
                return
            else:
                self.publish(VelocityCommand(angular_z=self.route_guide.rotation_sign * float(self.map_cfg.get("route_max_angular", 0.18))))
                return

        if self.state != "ALIGN_COIL" and self.route_guide.goal_visible(tag_ids):
            self.state = "ALIGN_COIL"
            message = "front goal marker detected; marker alignment started"
            self.get_logger().info(f"[wpt] {message}")
            self.run_log.event(message)

        if self.state == "ALIGN_COIL":
            self.publish(self.final_alignment_command(0.0))
            return

        if line_seen:
            self.last_route_line_angular = line_angular
        cmd = VelocityCommand(
            linear_x=float(self.map_cfg.get("route_linear", 0.03)),
            angular_z=self.last_route_line_angular,
        )
        self.state = "FOLLOW_LINE"
        self.publish(cmd)
        self._control_cycle_started = time.perf_counter()
        now = time.monotonic()
        dt = max(1e-3, now - self.last_step_time)
        self.last_step_time = now
        if not self.started:
            self.publish(VelocityCommand())
            return
        pose, line_angular = self.capture_observation()
        if pose is not None:
            self.last_pose_time = now
        elif self.pose_filter.pose is not None and now - self.last_pose_time <= float(self.map_cfg.get("prediction_timeout_sec", 0.4)):
            pose = self.pose_filter.predict(linear_m_s=self.last_cmd.linear_x, angular_rad_s=self.last_cmd.angular_z, dt_s=dt)
            self.latest_filtered_pose = pose
        if pose is None:
            if now - self.last_pose_time > float(self.map_cfg.get("lost_timeout_sec", 2.0)):
                self.state = "ERROR"
                self.publish(VelocityCommand())
            else:
                self.state = "LOCALIZE"
                self.publish(VelocityCommand(angular_z=float(self.map_cfg.get("scan_angular", 0.10))))
            return
        if self.route is None:
            start_coil = nearest_coil(pose.x_m, pose.y_m)
            self.route = RouteFollower(
                plan_axis_aligned_route(start_coil, self.target_coil),
                linear_speed=float(self.map_cfg.get("route_linear", 0.03)),
                max_angular=float(self.map_cfg.get("route_max_angular", 0.18)),
                turn_threshold_deg=float(self.map_cfg.get("heading_threshold_deg", 8.0)),
                waypoint_tolerance_m=float(self.map_cfg.get("preapproach_radius_m", 0.06)),
            )
            message = f"경로 생성: {start_coil} -> {self.target_coil}"
            self.get_logger().info(message)
            self.run_log.event(f"{message} waypoints={self.route.waypoints}")
        if self.state == "ALIGN_COIL":
            self.publish(self.final_alignment_command(line_angular))
            return
        cmd, route_complete = self.route.compute(pose, line_angular_z=line_angular)
        if route_complete:
            self.state = "ALIGN_COIL"
            self.publish(VelocityCommand())
            return
        self.state = "ROTATE_TO_ROUTE" if cmd.linear_x == 0.0 else "FOLLOW_ROUTE"
        self.publish(cmd)

    def expected_tags_for_cameras(self) -> dict[str, int]:
        if not self.route or len(self.route.waypoints) < 2:
            return {}
        # Use the LAST leg direction, not overall start-to-end direction.
        # For 3-waypoint diagonal routes (e.g. coil_4->coil_1 via coil_3),
        # the final alignment direction is determined by the last segment.
        start_x, start_y = self.route.waypoints[-2]
        target_x, target_y = self.route.waypoints[-1]
        
        c = int(self.target_coil[-1])
        north_id = c * 10 + 1
        south_id = c * 10 + 2
        west_id  = c * 10 + 3
        east_id  = c * 10 + 4
        
        is_horizontal = abs(target_x - start_x) >= abs(target_y - start_y)
        if is_horizontal:
            if target_x > start_x:  # Eastbound
                return {"front": east_id, "left_bottom": north_id, "right_bottom": south_id}
            else:  # Westbound
                return {"front": west_id, "left_bottom": south_id, "right_bottom": north_id}
        else:
            if target_y > start_y:  # Northbound
                return {"front": north_id, "left_bottom": west_id, "right_bottom": east_id}
            else:  # Southbound
                return {"front": south_id, "left_bottom": east_id, "right_bottom": west_id}

    def final_alignment_command(self, line_angular: float) -> VelocityCommand:
        expected = self.expected_tags_for_cameras()
        if not expected:
            self._report_alignment_marker_visibility(False)
            return VelocityCommand()

        all_obs = getattr(self, "all_observations", [])
        
        front_tag = expected["front"]
        left_tag = expected["left_bottom"]
        right_tag = expected["right_bottom"]
        
        front_obs = next((o for o in all_obs if o.tag_id == front_tag and o.camera_name == "front"), None)
        left_obs = next((o for o in all_obs if o.tag_id == left_tag and o.camera_name == "left_bottom"), None)
        right_obs = next((o for o in all_obs if o.tag_id == right_tag and o.camera_name == "right_bottom"), None)
        
        # Persistence: allow short tag loss before initiating scan
        if left_obs is None or right_obs is None or front_obs is None:
            self._report_alignment_marker_visibility(False)
            return VelocityCommand()
        self._report_alignment_marker_visibility(True)
        self.align_coil_missed_frames = 0
            
        target_left_x = 160.0
        target_right_x = 160.0
        target_front_x = 160.0
        
        err_forward = ( (left_obs.center_x - target_left_x) - (right_obs.center_x - target_right_x) ) / 2.0
        err_yaw = ( (left_obs.center_x - target_left_x) + (right_obs.center_x - target_right_x) ) / 2.0
        err_lateral = front_obs.center_x - target_front_x
        
        filtered = AlignmentError(err_lateral, err_forward, err_yaw)
        
        stop = self.cfg["alignment"]["undershoot_stop"]
        if is_undershoot_aligned(
            filtered,
            threshold_x_px=float(stop["threshold_x_px"]),
            undershoot_y_px=float(stop["undershoot_y_px"]),
            overshoot_y_px=float(stop["overshoot_y_px"]),
            threshold_angle_deg=float(stop["threshold_angle_deg"]),
            approach_y_sign=int(stop["approach_y_sign"]),
        ):
            self.stable_count += 1
            if self.stable_count >= int(self.cfg["alignment"]["stable_frames_required"]):
                self.state = "COMPLETE"
                self.started = False
                self.get_logger().info(f"{self.target_coil} 정합 완료")
                self.run_log.event(f"{self.target_coil} 정합 완료")
                return VelocityCommand()
        else:
            self.stable_count = 0
            
        control = self.cfg["control"]["coil"]
        speed = self.cfg["speed"]
        
        marker_cmd = compute_alignment_cmd(
            filtered,
            k_y_to_linear=float(control["k_y_to_linear"]),
            k_x_to_angular=float(control["k_x_to_angular"]),
            k_angle_to_angular=float(control["k_angle_to_angular"]),
            max_linear=float(speed["coil_max_linear"]),
            max_angular=float(speed["coil_max_angular"]),
            min_linear=float(speed["coil_min_linear"]),
            min_angular=float(speed["coil_min_angular"]),
            x_deadband_px=float(control["x_deadband_px"]),
            y_deadband_px=float(control["y_deadband_px"]),
            angle_deadband_deg=float(control["angle_deadband_deg"]),
        )
        
        cmd = blend_marker_and_line_cmd(
            marker_cmd,
            VelocityCommand(angular_z=line_angular),
            line_weight=float(self.cfg["line_tracking"].get("align_line_weight", 0.25)),
            max_linear=float(speed["coil_max_linear"]),
            max_angular=float(speed["coil_max_angular"]),
        )
        return block_reverse_linear_cmd(cmd, forward_linear_sign=int(stop.get("forward_linear_sign", 1)))

    def publish(self, cmd: VelocityCommand) -> None:
        if rclpy is not None and not rclpy.ok():
            return
        cycle_started = getattr(self, "_control_cycle_started", None)
        self.control_cycle_ms = None if cycle_started is None else (time.perf_counter() - cycle_started) * 1000.0
        period_ms = 1000.0 / float(self.cfg["control"].get("loop_hz", 10.0))
        self.control_overrun = self.control_cycle_ms is not None and self.control_cycle_ms > period_ms
        self.last_cmd = cmd
        if self.state != self._logged_state:
            self.run_log.event(f"상태 전환 {self._logged_state} -> {self.state}")
            self._logged_state = self.state
        waypoint = None
        if self.route is not None and self.route.index < len(self.route.waypoints):
            waypoint = self.route.waypoints[self.route.index]
        raw = self.latest_raw_pose
        filtered = self.latest_filtered_pose or self.pose_filter.pose
        line = self.latest_line
        self.run_log.telemetry(
            state=self.state,
            reason=self.state.lower(),
            tag_ids=";".join(str(d.tag_id) for d in self.latest_detections),
            raw_x_m=None if raw is None else raw.x_m,
            raw_y_m=None if raw is None else raw.y_m,
            raw_yaw_rad=None if raw is None else raw.yaw_rad,
            filtered_x_m=None if filtered is None else filtered.x_m,
            filtered_y_m=None if filtered is None else filtered.y_m,
            filtered_yaw_rad=None if filtered is None else filtered.yaw_rad,
            line_x_error_px=None if line is None else line.x_error,
            line_angle_error_deg=None if line is None else line.angle_error_deg,
            line_confidence=None if line is None else line.confidence,
            waypoint_x_m=None if waypoint is None else waypoint[0],
            waypoint_y_m=None if waypoint is None else waypoint[1],
            cmd_linear_x=cmd.linear_x,
            cmd_angular_z=cmd.angular_z,
            odom_linear_x=self.odom_linear_x,
            odom_angular_z=self.odom_angular_z,
            frame_capture_ms=getattr(self, "frame_capture_ms", None),
            tag_detect_ms=getattr(self, "tag_detect_ms", None),
            line_detect_ms=getattr(self, "line_detect_ms", None),
            control_cycle_ms=self.control_cycle_ms,
            control_overrun=self.control_overrun,
        )
        if self.dry_run:
            return
        message = Twist()
        message.linear.x = float(cmd.linear_x)
        message.angular.z = float(cmd.angular_z)
        self.pub.publish(message)


def main(args=None) -> None:
    if rclpy is None:
        raise RuntimeError("ROS2 환경에서 실행하세요")
    rclpy.init(args=args)
    node = GlobalMapNavigator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish(VelocityCommand())
        node.run_log.close()
        node.cap.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class GlobalMapNavigator(LegacyGlobalMapNavigator):
    def _route_line_detector(self) -> LineDetector:
        detector = getattr(self, "_route_line_detector_instance", None)
        if detector is not None:
            return detector
        cfg = self.cfg["line_tracking"]
        detector = LineDetector(
            threshold=int(cfg.get("threshold", 80)),
            min_area_px=float(cfg.get("min_area_px", 120)),
            roi_top_ratio=float(cfg.get("route_roi_top_ratio", 0.0)),
            roi_bottom_ratio=float(cfg.get("route_roi_bottom_ratio", 0.55)),
            roi_left_ratio=float(cfg.get("route_roi_left_ratio", 0.20)),
            roi_right_ratio=float(cfg.get("route_roi_right_ratio", 0.80)),
            max_abs_angle_deg=float(cfg.get("route_max_abs_angle_deg", 45.0)),
            min_vertical_span_ratio=float(cfg.get("route_min_vertical_span_ratio", 0.20)),
            min_vertical_aspect_ratio=float(cfg.get("route_min_vertical_aspect_ratio", 1.2)),
            blur_kernel=int(cfg.get("blur_kernel", 5)),
        )
        self._route_line_detector_instance = detector
        return detector

    def capture_observation(self) -> tuple[MapPose | None, float, bool]:
        """Read the front camera for route control; read side cameras only during alignment."""
        capture_started = time.perf_counter()
        ok, frame = self.cap.read()
        self.frame_capture_ms = (time.perf_counter() - capture_started) * 1000.0
        if not ok:
            self.latest_detections = []
            self.all_observations = []
            self.latest_raw_pose = None
            self.latest_line = None
            return None, 0.0, False

        tag_started = time.perf_counter()
        self.latest_detections = self.detector.detect(frame)
        self.tag_detect_ms = (time.perf_counter() - tag_started) * 1000.0
        self.all_observations = [
            TagObservation(det.tag_id, det.center[0], det.center[1], det.angle_deg, det.area_px, "front")
            for det in self.latest_detections
        ]

        # Side cameras are exclusively for the stationary final-marker alignment.
        # Keeping them out of the route loop prevents one unavailable side stream
        # from blocking line following on the front camera.
        if self.state == "ALIGN_COIL":
            for cam_name in ("left_bottom", "right_bottom"):
                cap = self.cameras.get(cam_name)
                if cap is None:
                    continue
                ok_side, side_frame = cap.read()
                if not ok_side:
                    continue
                for det in self.detector.detect(side_frame):
                    self.all_observations.append(
                        TagObservation(det.tag_id, det.center[0], det.center[1], det.angle_deg, det.area_px, cam_name)
                    )

        corners = {det.tag_id: det.corners for det in self.latest_detections if det.tag_id in self.tag_world_poses}
        measured = estimate_map_pose_from_tag_corners(
            corners,
            frame_size=(frame.shape[1], frame.shape[0]),
            tag_world_poses=self.tag_world_poses,
            tag_size_m=float(self.map_cfg["tag_size_m"]),
        )
        self.latest_raw_pose = measured
        if measured is not None:
            self.latest_filtered_pose = self.pose_filter.update([measured])

        line_started = time.perf_counter()
        line = self._route_line_detector().detect(frame) if self.cfg["line_tracking"].get("enabled", True) else None
        self.line_detect_ms = (time.perf_counter() - line_started) * 1000.0
        self.latest_line = line
        if line is None or line.confidence < float(self.cfg["line_tracking"].get("min_confidence", 0.1)):
            return measured, 0.0, False
        control = self.cfg["control"]["line"]
        x_error = 0.0 if abs(line.x_error) <= float(control.get("x_deadband_px", 0.0)) else line.x_error
        angle_error = 0.0 if abs(line.angle_error_deg) <= float(control.get("angle_deadband_deg", 0.0)) else line.angle_error_deg
        angular = float(control["k_x_to_angular"]) * x_error + float(control["k_angle_to_angular"]) * angle_error
        if bool(control.get("invert_angular", False)):
            angular = -angular
        limit = float(self.cfg["speed"]["line_max_angular"])
        return measured, max(-limit, min(limit, angular)), True

    def compute_perpendicular_error(self) -> float | None:
        if not self.route or len(self.route.waypoints) < 2:
            return None
        start_x, start_y = self.route.waypoints[-2]
        target_x, target_y = self.route.waypoints[-1]
        is_horizontal = abs(target_x - start_x) >= abs(target_y - start_y)
        
        target_coil_num = int(self.target_coil[-1])
        if is_horizontal:
            perp1, perp2 = target_coil_num * 10 + 1, target_coil_num * 10 + 2
            direction_sign = 1.0 if target_x > start_x else -1.0
        else:
            perp1, perp2 = target_coil_num * 10 + 3, target_coil_num * 10 + 4
            direction_sign = 1.0 if target_y > start_y else -1.0
            
        left_obs = next((o for o in self.all_observations if o.tag_id in (perp1, perp2) and o.camera_name == "left_bottom"), None)
        right_obs = next((o for o in self.all_observations if o.tag_id in (perp1, perp2) and o.camera_name == "right_bottom"), None)
        
        offset_left = (left_obs.center_x - 160.0) if left_obs else None
        offset_right = (160.0 - right_obs.center_x) if right_obs else None
        
        if offset_left is not None and offset_right is not None:
            err = (offset_left + offset_right) / 2.0
        elif offset_left is not None:
            err = offset_left
        elif offset_right is not None:
            err = offset_right
        else:
            return None
            
        return err * direction_sign

    def step(self) -> None:
        self._control_cycle_started = time.perf_counter()
        now = time.monotonic()
        self.last_step_time = now
        if not self.started:
            self.publish(VelocityCommand())
            return

        _measured, line_angular, line_seen = self.capture_observation()
        self._report_line_visibility(line_seen)

        # A missing front line is a hard safety stop.  Do not fall through to
        # ACQUIRE_ROUTE/FOLLOW_LINE, which would otherwise publish the previous
        # linear command after logging that the robot stopped.
        if not line_seen and self.state != "ALIGN_COIL":
            self.state = "LINE_LOST"
            self.publish(VelocityCommand())
            return
        if self.route is None:
            from .global_map import expected_route_marker_ids
            from .global_route_control import MarkerRouteGuide

            start_coil, visible_tag_ids = self._visible_start_coil()
            if start_coil is None:
                self.state = "WAIT_FOR_START_MARKER"
                self.publish(VelocityCommand())
                return
            if start_coil == self.target_coil:
                self.state = "ALIGN_COIL"
                self.get_logger().info(f"[wpt] start marker {visible_tag_ids}: already at {start_coil}")
                self.run_log.event(f"start coil={start_coil} markers={visible_tag_ids}")
                self.publish(self.final_alignment_command(0.0))
                return

            departure_id, goal_id = expected_route_marker_ids(start_coil, self.target_coil)
            self.route = RouteFollower(
                plan_axis_aligned_route(start_coil, self.target_coil),
                linear_speed=float(self.map_cfg.get("route_linear", 0.03)),
                max_angular=float(self.map_cfg.get("route_max_angular", 0.18)),
                turn_threshold_deg=float(self.map_cfg.get("heading_threshold_deg", 8.0)),
                waypoint_tolerance_m=float(self.map_cfg.get("preapproach_radius_m", 0.06)),
            )
            self.route_guide = MarkerRouteGuide(
                departure_id,
                goal_id,
                image_width=int(self.cfg["cameras"][self.map_cfg.get("camera", "front")].get("width", 320)),
                center_tolerance_px=float(self.map_cfg.get("route_marker_center_tolerance_px", 45)),
            )
            self._start_coil = start_coil
            self.route_started_at = now
            self.last_route_line_angular = 0.0
            self.state = "ACQUIRE_ROUTE"
            message = f"start coil={start_coil} markers={visible_tag_ids}; departure={departure_id}; goal={goal_id}"
            self.get_logger().info(f"[wpt] {message}")
            self.run_log.event(message)

        tag_ids = [det.tag_id for det in self.latest_detections]
        tag_centers = [(det.tag_id, det.center[0]) for det in self.latest_detections]
        self.route_guide.update_turn_direction(tag_ids)

        if self.state == "ACQUIRE_ROUTE":
            if self.route_guide.update_departure(tag_centers):
                self.state = "FOLLOW_LINE"
                message = "departure direction confirmed; line tracing started"
                self.get_logger().info(f"[wpt] {message}")
                self.run_log.event(message)
            elif now - self.route_started_at > float(self.map_cfg.get("acquire_timeout_sec", 12.0)):
                self.state = "ERROR"
                self.publish(VelocityCommand())
                return
            else:
                self.publish(VelocityCommand(angular_z=self.route_guide.rotation_sign * float(self.map_cfg.get("route_max_angular", 0.18))))
                return

        if self.state != "ALIGN_COIL" and self.route_guide.goal_visible(tag_ids):
            self.state = "ALIGN_COIL"
            message = "front goal marker detected; marker alignment started"
            self.get_logger().info(f"[wpt] {message}")
            self.run_log.event(message)

        if self.state == "ALIGN_COIL":
            self.publish(self.final_alignment_command(0.0))
            return

        if line_seen:
            self.last_route_line_angular = line_angular
        cmd = VelocityCommand(
            linear_x=float(self.map_cfg.get("route_linear", 0.03)),
            angular_z=self.last_route_line_angular,
        )
        self.state = "FOLLOW_LINE"
        self.publish(cmd)
