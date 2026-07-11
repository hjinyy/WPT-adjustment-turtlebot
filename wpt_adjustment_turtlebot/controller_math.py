"""Pure control math for AprilTag based TurtleBot alignment."""

from dataclasses import dataclass
from enum import Enum
from math import atan2, copysign, degrees


class AlignmentState(str, Enum):
    IDLE = "IDLE"
    SEARCH_HEAD_TAG = "SEARCH_HEAD_TAG"
    APPROACH_SHELF = "APPROACH_SHELF"
    ENTER_SHELF = "ENTER_SHELF"
    SEARCH_COIL = "SEARCH_COIL"
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
class TagPairObservation:
    first: TagObservation
    second: TagObservation
    midpoint_x: float
    midpoint_y: float
    pair_angle_deg: float
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


def compute_pair_observation(first: TagObservation, second: TagObservation) -> TagPairObservation:
    midpoint_x = (first.center_x + second.center_x) / 2.0
    midpoint_y = (first.center_y + second.center_y) / 2.0
    pair_angle_deg = degrees(atan2(second.center_y - first.center_y, second.center_x - first.center_x))
    camera_name = first.camera_name if first.camera_name == second.camera_name else ""
    return TagPairObservation(first, second, midpoint_x, midpoint_y, pair_angle_deg, camera_name)


def compute_alignment_error(obs: TagObservation, target: TargetPoseInImage) -> AlignmentError:
    return AlignmentError(
        x=obs.center_x - target.x,
        y=obs.center_y - target.y,
        angle_deg=angle_error_deg(obs.angle_deg, target.angle_deg),
    )


def compute_pair_alignment_error(pair: TagPairObservation, target: TargetPoseInImage) -> AlignmentError:
    return AlignmentError(
        x=pair.midpoint_x - target.x,
        y=pair.midpoint_y - target.y,
        angle_deg=angle_error_deg(pair.pair_angle_deg, target.angle_deg),
    )


def is_aligned(err: AlignmentError, threshold_x_px: float, threshold_y_px: float, threshold_angle_deg: float) -> bool:
    return abs(err.x) <= threshold_x_px and abs(err.y) <= threshold_y_px and abs(err.angle_deg) <= threshold_angle_deg


def grid_cell(x: float, y: float, width: float, height: float, grid_size: int = 3) -> tuple[int, int]:
    """1-indexed (row, col) of the grid_size x grid_size cell containing (x, y) in a width x height frame."""
    col = min(grid_size, max(1, int(x / width * grid_size) + 1))
    row = min(grid_size, max(1, int(y / height * grid_size) + 1))
    return row, col


def alignment_state_for_report(any_tag_detected: bool, all_cameras_aligned: bool, locked: bool) -> str:
    """Map a local 3x3-grid alignment check to the charging-control server's alignment_state enum."""
    if locked:
        return "Locked"
    if all_cameras_aligned:
        return "Aligned"
    if any_tag_detected:
        return "Searching"
    return "None"


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


def blend_marker_and_line_cmd(
    marker_cmd: VelocityCommand | None,
    line_cmd: VelocityCommand | None,
    *,
    line_weight: float,
    max_linear: float,
    max_angular: float,
) -> VelocityCommand:
    """Blend marker-centered alignment with auxiliary line following.

    When marker_cmd exists, it owns linear.x so the robot keeps prioritizing the
    marker-pair midpoint. The line can only add a bounded angular correction.
    When marker_cmd is missing, line_cmd can drive conservative line following.
    """
    if marker_cmd is None and line_cmd is None:
        return VelocityCommand()
    if marker_cmd is None:
        return VelocityCommand(
            linear_x=clamp(line_cmd.linear_x, max_linear),
            angular_z=clamp(line_cmd.angular_z, max_angular),
        )
    angular = marker_cmd.angular_z
    if line_cmd is not None:
        angular += float(line_weight) * line_cmd.angular_z
    return VelocityCommand(
        linear_x=clamp(marker_cmd.linear_x, max_linear),
        angular_z=clamp(angular, max_angular),
    )


def is_undershoot_aligned(
    err: AlignmentError,
    *,
    threshold_x_px: float,
    undershoot_y_px: float,
    overshoot_y_px: float,
    threshold_angle_deg: float,
    approach_y_sign: int,
) -> bool:
    """Return true only when the robot is close while still on the approach side.

    approach_y_sign defines which sign of y_error means "not yet past the
    target". For the current top-down setup, -1 means negative y_error is the
    preferred undershoot side and positive y_error is overshoot.
    """
    sign = -1 if int(approach_y_sign) < 0 else 1
    y_on_approach_axis = err.y * sign
    return (
        abs(err.x) <= abs(threshold_x_px)
        and -abs(overshoot_y_px) <= y_on_approach_axis <= abs(undershoot_y_px)
        and abs(err.angle_deg) <= abs(threshold_angle_deg)
    )


def block_reverse_linear_cmd(cmd: VelocityCommand, *, forward_linear_sign: int) -> VelocityCommand:
    """Suppress reverse linear corrections to avoid front/back oscillation."""
    sign = -1 if int(forward_linear_sign) < 0 else 1
    if cmd.linear_x * sign < 0:
        return VelocityCommand(linear_x=0.0, angular_z=cmd.angular_z)
    return cmd
