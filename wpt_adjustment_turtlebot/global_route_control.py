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
