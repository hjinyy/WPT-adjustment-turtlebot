"""전역 웨이포인트와 라인 보조를 결합한 차동구동 제어."""

from __future__ import annotations

import math

from .controller_math import VelocityCommand, clamp
from .map_localization import MapPose, wrap_angle_rad


class RouteFollower:
    def __init__(
        self,
        waypoints: list[tuple[float, float]],
        *,
        linear_speed: float = 0.03,
        max_angular: float = 0.18,
        heading_kp: float = 1.0,
        turn_threshold_deg: float = 8.0,
        waypoint_tolerance_m: float = 0.04,
    ) -> None:
        if len(waypoints) < 2:
            raise ValueError("at least two waypoints are required")
        self.waypoints = waypoints
        self.index = 1
        self.linear_speed = float(linear_speed)
        self.max_angular = float(max_angular)
        self.heading_kp = float(heading_kp)
        self.turn_threshold = math.radians(float(turn_threshold_deg))
        self.waypoint_tolerance_m = float(waypoint_tolerance_m)

    def compute(self, pose: MapPose, *, line_angular_z: float) -> tuple[VelocityCommand, bool]:
        target_x, target_y = self.waypoints[self.index]
        dx, dy = target_x - pose.x_m, target_y - pose.y_m
        distance = math.hypot(dx, dy)
        if distance <= self.waypoint_tolerance_m:
            if self.index == len(self.waypoints) - 1:
                return VelocityCommand(), True
            self.index += 1
            target_x, target_y = self.waypoints[self.index]
            dx, dy = target_x - pose.x_m, target_y - pose.y_m
        heading_error = wrap_angle_rad(math.atan2(dy, dx) - pose.yaw_rad)
        angular = clamp(self.heading_kp * heading_error, self.max_angular)
        if abs(heading_error) > self.turn_threshold:
            return VelocityCommand(angular_z=angular), False
        angular = clamp(angular + float(line_angular_z), self.max_angular)
        return VelocityCommand(linear_x=max(0.0, self.linear_speed), angular_z=angular), False


TAG_SUFFIX_YAW_RAD = {
    1: math.pi / 2.0,
    2: -math.pi / 2.0,
    3: math.pi,
    4: 0.0,
}


class MarkerRouteGuide:
    """Gate route departure and preserve a deterministic search turn direction."""

    def __init__(
        self,
        departure_marker_id: int,
        goal_marker_id: int,
        *,
        image_width: int,
        center_tolerance_px: float,
    ) -> None:
        self.departure_marker_id = int(departure_marker_id)
        self.goal_marker_id = int(goal_marker_id)
        self.image_width = int(image_width)
        self.center_tolerance_px = float(center_tolerance_px)
        self.departure_acquired = False
        self.rotation_sign = 1.0

    def update_turn_direction(self, tag_ids: list[int]) -> float:
        source_coil = self.departure_marker_id // 10
        source_tag = next((tag_id for tag_id in tag_ids if tag_id // 10 == source_coil), None)
        if source_tag is None:
            return self.rotation_sign
        current_yaw = TAG_SUFFIX_YAW_RAD.get(source_tag % 10)
        target_yaw = TAG_SUFFIX_YAW_RAD.get(self.departure_marker_id % 10)
        if current_yaw is None or target_yaw is None:
            return self.rotation_sign
        error = wrap_angle_rad(target_yaw - current_yaw)
        if abs(error) >= math.pi - 1e-6:
            self.rotation_sign = 1.0
        elif abs(error) > 1e-6:
            self.rotation_sign = 1.0 if error > 0.0 else -1.0
        return self.rotation_sign

    def update_departure(self, tag_centers: list[tuple[int, float]]) -> bool:
        self.update_turn_direction([tag_id for tag_id, _center_x in tag_centers])
        image_center = self.image_width / 2.0
        for tag_id, center_x in tag_centers:
            if tag_id == self.departure_marker_id and abs(center_x - image_center) <= self.center_tolerance_px:
                self.departure_acquired = True
                break
        return self.departure_acquired

    def goal_visible(self, tag_ids: list[int]) -> bool:
        return self.goal_marker_id in tag_ids


class NavigationRecoveryPolicy:
    """Allow a bounded forward grace period after both route signals disappear."""

    def __init__(self, grace_sec: float = 0.5) -> None:
        self.grace_sec = float(grace_sec)
        self.last_seen_time: float | None = None

    def observe(self, *, now: float, line_seen: bool, marker_seen: bool) -> None:
        if line_seen or marker_seen:
            self.last_seen_time = float(now)

    def should_reacquire(self, *, now: float) -> bool:
        return self.last_seen_time is not None and float(now) - self.last_seen_time > self.grace_sec
