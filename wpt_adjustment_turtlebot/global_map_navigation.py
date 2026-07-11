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


class GlobalMapNavigator(Node):
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
        self.latest_detections = []
        self.latest_raw_pose = None
        self.latest_filtered_pose = None
        self.latest_line = None
        self.odom_linear_x = None
        self.odom_angular_z = None
        self._logged_state = None

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
        camera = self.cfg["cameras"][self.map_cfg.get("camera", "front")]
        self.cap = cv2.VideoCapture(int(camera["device"]), cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(camera.get("width", 320)))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(camera.get("height", 240)))
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
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
        self.state = "LOCALIZE"
        self.route = None
        self.stable_count = 0
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
        self.latest_detections = self.detector.detect(frame)
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

    def step(self) -> None:
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

    def final_alignment_command(self, line_angular: float) -> VelocityCommand:
        west_id, east_id = four_coil_pair_ids(self.target_coil, "west_east")
        by_id = {d.tag_id: d for d in self.latest_detections}
        if west_id not in by_id or east_id not in by_id:
            return VelocityCommand(angular_z=float(self.map_cfg.get("final_scan_angular", 0.05)))

        def observation(d):
            return TagObservation(d.tag_id, d.center[0], d.center[1], d.angle_deg, d.area_px, "front")

        pair = compute_pair_observation(observation(by_id[west_id]), observation(by_id[east_id]))
        target_cfg = self.cfg["coils"][self.target_coil]["targets"]["front"]["default"]
        error = compute_pair_alignment_error(pair, TargetPoseInImage(**target_cfg))
        estimate = self.marker_filter.update(x_error=error.x, y_error=error.y, angle_error_deg=error.angle_deg)
        filtered = AlignmentError(estimate.x_error, estimate.y_error, estimate.angle_error_deg)
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
        )
        if self.dry_run:
            self.get_logger().info(f"state={self.state} cmd=({cmd.linear_x:.3f}, {cmd.angular_z:.3f})")
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