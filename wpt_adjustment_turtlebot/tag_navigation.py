"""Helpers for estimating robot position and heading from the 4-coil tag map."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, degrees, hypot

from .shelf_layout import SHELF_COL, SHELF_ROW
from .tag_layout import decode_four_coil_tag


POSITION_HEADING_TO_CENTER_DEG = {
    "west": 0.0,
    "south": -90.0,
    "east": 180.0,
    "north": 90.0,
}

# Stage-frame yaw of the printed tag's image x-axis/top edge.
# 0=east/right, 90=south/down, -90=north/up, 180=west/left.
#
# The top-row north tags face the upper shelf side, while the bottom-row north
# tags face the lower shelf side. This is why coil_1/2 north and coil_3/4 north
# intentionally have opposite yaw values even though they share the "north"
# logical position name.
DEFAULT_TAG_STAGE_YAW_DEG = {
    11: -90.0,
    12: 90.0,
    13: 180.0,
    14: 0.0,
    21: -90.0,
    22: 90.0,
    23: 180.0,
    24: 0.0,
    32: 90.0,
    31: -90.0,
    34: 180.0,
    33: 0.0,
    42: 90.0,
    41: -90.0,
    44: 180.0,
    43: 0.0,
}

# For the front top-down camera, image +x points to the robot's right side.
# In the stage heading convention that is robot_heading + 90 degrees.
DEFAULT_CAMERA_X_AXIS_OFFSET_DEG = {"front": 90.0}


@dataclass(frozen=True)
class NavigationTag:
    tag_id: int
    camera_name: str
    area_px: float = 1.0
    angle_deg: float | None = None


@dataclass(frozen=True)
class LocalizedPose:
    shelf: int
    coil_name: str
    heading_deg: float | None
    detected_tag_ids: tuple[int, ...]
    heading_source: str = ""


def normalize_angle_deg(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


def turn_delta_deg(current_heading_deg: float, target_heading_deg: float) -> float:
    """Shortest signed turn from current heading to target heading."""
    return normalize_angle_deg(target_heading_deg - current_heading_deg)


def heading_between_shelves(from_shelf: int, to_shelf: int) -> float:
    """Heading in the shelf-layout frame; 0=east/right, 90=south/down."""
    dx = SHELF_COL[to_shelf] - SHELF_COL[from_shelf]
    dy = SHELF_ROW[to_shelf] - SHELF_ROW[from_shelf]
    return normalize_angle_deg(degrees(atan2(dy, dx)))


def distance_in_grid_steps(from_shelf: int, to_shelf: int) -> float:
    dx = SHELF_COL[to_shelf] - SHELF_COL[from_shelf]
    dy = SHELF_ROW[to_shelf] - SHELF_ROW[from_shelf]
    return hypot(dx, dy)


def heading_from_tag_orientation(
    *,
    tag_stage_yaw_deg: float,
    observed_image_angle_deg: float,
    camera_x_axis_offset_deg: float = 90.0,
) -> float:
    """Estimate robot heading from a known physical tag yaw and observed image angle.

    observed_image_angle_deg is measured in image coordinates from +x/right.
    camera_x_axis_offset_deg is the stage-frame direction of image +x relative
    to robot forward. For a normal top-down front camera, image +x is robot
    right, so this offset is +90 degrees.
    """
    return normalize_angle_deg(tag_stage_yaw_deg - observed_image_angle_deg - camera_x_axis_offset_deg)


def rectilinear_shelf_path(from_shelf: int, to_shelf: int) -> list[int]:
    """Path along the rectangle edges connecting coil centers.

    Diagonal moves are decomposed into two edge moves. Horizontal-first is used
    as the deterministic default because the current 2x2 layout has equal path
    alternatives for diagonal moves.
    """
    if from_shelf == to_shelf:
        return [from_shelf]

    path = [from_shelf]
    current = from_shelf
    target_col = SHELF_COL[to_shelf]
    target_row = SHELF_ROW[to_shelf]

    if SHELF_COL[current] != target_col:
        current = next(shelf for shelf, col in SHELF_COL.items() if col == target_col and SHELF_ROW[shelf] == SHELF_ROW[current])
        path.append(current)
    if SHELF_ROW[current] != target_row:
        current = next(shelf for shelf, row in SHELF_ROW.items() if row == target_row and SHELF_COL[shelf] == SHELF_COL[current])
        path.append(current)
    return path


def estimate_pose_from_tags(
    tags: list[NavigationTag],
    *,
    tag_stage_yaw_deg: dict[int, float] | None = None,
    camera_x_axis_offset_deg: dict[str, float] | None = None,
) -> LocalizedPose | None:
    """Estimate current coil/shelf and heading from visible four-coil tags.

    Current shelf is selected by weighted visible tag area. Heading prefers a
    physical tag-orientation estimate when the detection includes angle_deg,
    then falls back to logical front-camera tag position.
    """
    tag_stage_yaw_deg = tag_stage_yaw_deg or DEFAULT_TAG_STAGE_YAW_DEG
    camera_x_axis_offset_deg = camera_x_axis_offset_deg or DEFAULT_CAMERA_X_AXIS_OFFSET_DEG
    coil_scores: dict[str, float] = {}
    orientation_heading_scores: dict[float, float] = {}
    position_heading_scores: dict[float, float] = {}
    detected_ids: list[int] = []

    for tag in tags:
        decoded = decode_four_coil_tag(tag.tag_id)
        if decoded is None:
            continue
        coil_name, position = decoded
        detected_ids.append(tag.tag_id)
        weight = max(float(tag.area_px), 1.0)
        coil_scores[coil_name] = coil_scores.get(coil_name, 0.0) + weight
        camera_offset = camera_x_axis_offset_deg.get(tag.camera_name)
        physical_yaw = tag_stage_yaw_deg.get(tag.tag_id)
        if camera_offset is not None and physical_yaw is not None and tag.angle_deg is not None:
            heading = heading_from_tag_orientation(
                tag_stage_yaw_deg=physical_yaw,
                observed_image_angle_deg=tag.angle_deg,
                camera_x_axis_offset_deg=camera_offset,
            )
            orientation_heading_scores[heading] = orientation_heading_scores.get(heading, 0.0) + weight
        elif tag.camera_name == "front":
            heading = POSITION_HEADING_TO_CENTER_DEG[position]
            position_heading_scores[heading] = position_heading_scores.get(heading, 0.0) + weight

    if not coil_scores:
        return None

    coil_name = max(coil_scores, key=coil_scores.get)
    shelf = int(coil_name.split("_", 1)[1])
    if orientation_heading_scores:
        heading = max(orientation_heading_scores, key=orientation_heading_scores.get)
        heading_source = "tag_orientation"
    elif position_heading_scores:
        heading = max(position_heading_scores, key=position_heading_scores.get)
        heading_source = "tag_position"
    else:
        heading = None
        heading_source = ""
    return LocalizedPose(
        shelf=shelf,
        coil_name=coil_name,
        heading_deg=heading,
        detected_tag_ids=tuple(sorted(set(detected_ids))),
        heading_source=heading_source,
    )
