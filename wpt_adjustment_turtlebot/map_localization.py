"""AprilTag 전역 자세의 평면 호모그래피와 EKF."""

from dataclasses import dataclass
from math import atan2, cos, pi, sin


def wrap_angle_rad(angle: float) -> float:
    return (float(angle) + pi) % (2.0 * pi) - pi


@dataclass(frozen=True)
class MapPose:
    x_m: float
    y_m: float
    yaw_rad: float


class MapPoseEKF:
    def __init__(self, *, process_variance: float, measurement_variance: float, outlier_distance_m: float) -> None:
        self.pose: MapPose | None = None
        self.process_variance = float(process_variance)
        self.measurement_variance = float(measurement_variance)
        self.outlier_distance_m = float(outlier_distance_m)
        self._variance = [1.0, 1.0, 1.0]

    def predict(self, *, linear_m_s: float, angular_rad_s: float, dt_s: float) -> MapPose:
        if self.pose is None:
            raise ValueError("pose observation is required before prediction")
        dt = max(0.0, float(dt_s))
        yaw = wrap_angle_rad(self.pose.yaw_rad + float(angular_rad_s) * dt)
        self.pose = MapPose(
            self.pose.x_m + float(linear_m_s) * cos(self.pose.yaw_rad) * dt,
            self.pose.y_m + float(linear_m_s) * sin(self.pose.yaw_rad) * dt,
            yaw,
        )
        self._variance = [v + self.process_variance * max(dt, 1e-3) for v in self._variance]
        return self.pose

    def update(self, observations: list[MapPose]) -> MapPose:
        if not observations:
            if self.pose is None:
                raise ValueError("at least one map pose observation is required")
            return self.pose
        xs = sorted(p.x_m for p in observations)
        ys = sorted(p.y_m for p in observations)
        median_x, median_y = xs[len(xs) // 2], ys[len(ys) // 2]
        valid = [p for p in observations if (p.x_m - median_x) ** 2 + (p.y_m - median_y) ** 2 <= self.outlier_distance_m ** 2]
        if not valid:
            valid = [observations[0]]
        measured = MapPose(
            sum(p.x_m for p in valid) / len(valid),
            sum(p.y_m for p in valid) / len(valid),
            atan2(sum(sin(p.yaw_rad) for p in valid), sum(cos(p.yaw_rad) for p in valid)),
        )
        if self.pose is None:
            self.pose = MapPose(measured.x_m, measured.y_m, wrap_angle_rad(measured.yaw_rad))
            self._variance = [self.measurement_variance] * 3
            return self.pose
        gains = [v / (v + self.measurement_variance) for v in self._variance]
        self.pose = MapPose(
            self.pose.x_m + gains[0] * (measured.x_m - self.pose.x_m),
            self.pose.y_m + gains[1] * (measured.y_m - self.pose.y_m),
            wrap_angle_rad(self.pose.yaw_rad + gains[2] * wrap_angle_rad(measured.yaw_rad - self.pose.yaw_rad)),
        )
        self._variance = [(1.0 - gain) * variance for gain, variance in zip(gains, self._variance)]
        return self.pose


def estimate_map_pose_from_tag_corners(tag_corners, *, frame_size, tag_world_poses, tag_size_m: float) -> MapPose | None:
    """태그 모서리로 영상 평면을 전역 바닥 평면에 투영한다."""
    import cv2
    import numpy as np

    image_points, world_points = [], []
    half = float(tag_size_m) / 2.0
    for tag_id, corners in tag_corners.items():
        if int(tag_id) not in tag_world_poses:
            continue
        center_x, center_y = tag_world_poses[int(tag_id)]
        image_points.extend(np.asarray(corners, dtype=np.float32).reshape(4, 2))
        world_points.extend([
            (center_x - half, center_y + half),
            (center_x + half, center_y + half),
            (center_x + half, center_y - half),
            (center_x - half, center_y - half),
        ])
    if len(image_points) < 4:
        return None
    homography, _mask = cv2.findHomography(np.asarray(image_points, dtype=np.float32), np.asarray(world_points, dtype=np.float32), cv2.RANSAC)
    if homography is None:
        return None
    width, height = frame_size
    query = np.asarray([[[width / 2.0, height / 2.0], [width / 2.0, 0.0]]], dtype=np.float32)
    center, forward = cv2.perspectiveTransform(query, homography)[0]
    yaw = atan2(float(forward[1] - center[1]), float(forward[0] - center[0]))
    return MapPose(float(center[0]), float(center[1]), wrap_angle_rad(yaw))
