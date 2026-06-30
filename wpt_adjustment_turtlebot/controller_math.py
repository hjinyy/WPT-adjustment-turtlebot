"""Pure control math for AprilTag based TurtleBot alignment."""

from dataclasses import dataclass
from enum import Enum
from math import copysign


class AlignmentState(str, Enum):
    IDLE = "IDLE"
    SEARCH_HEAD_TAG = "SEARCH_HEAD_TAG"
    APPROACH_SHELF = "APPROACH_SHELF"
    ENTER_SHELF = "ENTER_SHELF"
    ALIGN_COIL = "ALIGN_COIL"
    FINAL_STOP = "FINAL_STOP"
    CHARGING = "CHARGING"
    ERROR = "ERROR"


@dataclass
class TagObservation:
    tag_id: int
    center_x: float
    center_y: float
    angle_deg: float
    area_px: float = 0.0
    camera_name: str = ""


@dataclass
class TargetPoseInImage:
    x: float
    y: float
    angle_deg: float


@dataclass
class AlignmentError:
    x: float
    y: float
    angle_deg: float


@dataclass
class VelocityCommand:
    linear_x: float = 0.0
    angular_z: float = 0.0


def clamp(value: float, limit: float) -> float:
    limit = abs(limit)
    return max(-limit, min(limit, value))


def deadband(value: float, threshold: float) -> float:
    return 0.0 if abs(value) < abs(threshold) else value


def angle_error_deg(current: float, target: float) -> float:
    return (current - target + 180.0) % 360.0 - 180.0


def compute_alignment_error(obs: TagObservation, target: TargetPoseInImage) -> AlignmentError:
    return AlignmentError(
        x=obs.center_x - target.x,
        y=obs.center_y - target.y,
        angle_deg=angle_error_deg(obs.angle_deg, target.angle_deg),
    )


def is_aligned(err: AlignmentError, threshold_x_px: float, threshold_y_px: float, threshold_angle_deg: float) -> bool:
    return abs(err.x) <= threshold_x_px and abs(err.y) <= threshold_y_px and abs(err.angle_deg) <= threshold_angle_deg


def compute_alignment_cmd(
    err: AlignmentError,
    *,
    k_y_to_linear: float,
    k_x_to_angular: float,
    k_angle_to_angular: float,
    max_linear: float,
    max_angular: float,
    min_linear: float = 0.0,
    min_angular: float = 0.0,
    x_deadband_px: float = 0.0,
    y_deadband_px: float = 0.0,
    angle_deadband_deg: float = 0.0,
    invert_linear: bool = False,
    invert_angular: bool = False,
) -> VelocityCommand:
    """Map image error to differential-drive TurtleBot velocity.

    Typical first tuning:
    linear.x  = k_y * y_error
    angular.z = k_x * x_error + k_angle * angle_error
    Signs may need inversion depending on camera mounting.
    """
    y = deadband(err.y, y_deadband_px)
    x = deadband(err.x, x_deadband_px)
    a = deadband(err.angle_deg, angle_deadband_deg)
    linear = k_y_to_linear * y
    angular = k_x_to_angular * x + k_angle_to_angular * a
    if invert_linear:
        linear *= -1.0
    if invert_angular:
        angular *= -1.0
    linear = clamp(linear, max_linear)
    angular = clamp(angular, max_angular)
    if linear and abs(linear) < min_linear:
        linear = copysign(min_linear, linear)
    if angular and abs(angular) < min_angular:
        angular = copysign(min_angular, angular)
    return VelocityCommand(linear_x=linear, angular_z=angular)
