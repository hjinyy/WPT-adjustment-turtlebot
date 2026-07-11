"""Coil-to-coil transit planning on the 2x2 stage with a GLOBAL compass.

Every coil's markers are physically aligned to ONE shared compass (not
per-coil "north faces the shelf" anymore):

        north
    coil_1 | coil_2
    -------+-------      west  <->  east
    coil_3 | coil_4
        south

So coil_1's south marker (id 12) and coil_3's south marker (id 32) point the
same physical direction.

Transit rule (example: coil_1 -> coil_3, travel direction = south):
- The departure coil's marker in the travel direction is the "head" marker
  the robot drives out over (coil_1 south = id 12).
- The robot line-follows the black tape between coils, and STOPS the moment
  the front camera sees the TARGET coil's marker in the same travel
  direction (coil_3 south = id 32). Seeing that far-side marker from the
  front camera means the receive coil is over the transmit coil.
"""

from __future__ import annotations

from dataclasses import dataclass

from .controller_math import VelocityCommand, clamp, deadband
from .tag_layout import four_coil_tag_id

# (col, row) on the stage; row grows southward, col grows eastward.
COIL_GRID_POS: dict[str, tuple[int, int]] = {
    "coil_1": (0, 0),
    "coil_2": (1, 0),
    "coil_3": (0, 1),
    "coil_4": (1, 1),
}

_STEP_TO_DIRECTION = {
    (0, -1): "north",
    (0, 1): "south",
    (-1, 0): "west",
    (1, 0): "east",
}


def normalize_coil_name(coil: str | int) -> str:
    if isinstance(coil, int):
        coil = f"coil_{coil}"
    coil = str(coil).lower()
    if coil in {"1", "2", "3", "4"}:
        coil = f"coil_{coil}"
    if coil not in COIL_GRID_POS:
        raise ValueError(f"unknown coil {coil!r}; expected one of {sorted(COIL_GRID_POS)}")
    return coil


def travel_direction(from_coil: str | int, to_coil: str | int) -> str | None:
    """Compass direction for one grid step, or None if not adjacent."""
    a = COIL_GRID_POS[normalize_coil_name(from_coil)]
    b = COIL_GRID_POS[normalize_coil_name(to_coil)]
    return _STEP_TO_DIRECTION.get((b[0] - a[0], b[1] - a[1]))


@dataclass(frozen=True)
class TransitLeg:
    """One straight segment between two adjacent coils."""

    from_coil: str
    to_coil: str
    direction: str
    head_marker_id: int  # departure coil's marker in the travel direction
    stop_marker_id: int  # target coil's marker in the travel direction -> stop on sight


def make_leg(from_coil: str | int, to_coil: str | int) -> TransitLeg:
    from_name = normalize_coil_name(from_coil)
    to_name = normalize_coil_name(to_coil)
    direction = travel_direction(from_name, to_name)
    if direction is None:
        raise ValueError(f"{from_name} -> {to_name} is not a single straight segment")
    return TransitLeg(
        from_coil=from_name,
        to_coil=to_name,
        direction=direction,
        head_marker_id=four_coil_tag_id(from_name, direction),
        stop_marker_id=four_coil_tag_id(to_name, direction),
    )


def plan_transit_legs(from_coil: str | int, to_coil: str | int) -> list[TransitLeg]:
    """Straight legs from one coil to another.

    Adjacent coils give a single leg. Diagonal moves (e.g. coil_1 -> coil_4)
    are split into two straight legs, vertical first (the taped paths run
    along the grid axes, so the robot never drives a diagonal).
    """
    from_name = normalize_coil_name(from_coil)
    to_name = normalize_coil_name(to_coil)
    if from_name == to_name:
        return []
    direction = travel_direction(from_name, to_name)
    if direction is not None:
        return [make_leg(from_name, to_name)]

    # Diagonal: go vertical first, then horizontal.
    from_col, _from_row = COIL_GRID_POS[from_name]
    _to_col, to_row = COIL_GRID_POS[to_name]
    corner = next(name for name, pos in COIL_GRID_POS.items() if pos == (from_col, to_row))
    return [make_leg(from_name, corner), make_leg(corner, to_name)]


def compute_line_follow_cmd(
    x_error: float,
    angle_error_deg: float,
    *,
    cruise_linear: float,
    k_x_to_angular: float,
    k_angle_to_angular: float,
    max_angular: float,
    x_deadband_px: float = 0.0,
    angle_deadband_deg: float = 0.0,
    invert_angular: bool = False,
) -> VelocityCommand:
    """Steer back onto the tape while cruising forward.

    x_error / angle_error_deg follow LineObservation conventions (positive
    x_error = line is right of image center). Gains map that to angular.z;
    signs depend on camera mounting, so flip with invert_angular (or negative
    gains) after a dry-run check, same as the coil alignment controller.
    """
    x = deadband(x_error, x_deadband_px)
    a = deadband(angle_error_deg, angle_deadband_deg)
    angular = k_x_to_angular * x + k_angle_to_angular * a
    if invert_angular:
        angular *= -1.0
    return VelocityCommand(linear_x=float(cruise_linear), angular_z=clamp(angular, max_angular))
