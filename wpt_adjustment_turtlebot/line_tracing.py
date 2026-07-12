"""Front-camera black tape detection for line tracing."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .controller_math import VelocityCommand, clamp, grid_cell


@dataclass(frozen=True)
class LineObservation:
    found: bool
    center_x: float | None
    center_y: float | None
    error_x: float
    cell: tuple[int, int] | None
    area_px: float


def detect_black_line(
    frame_bgr,
    *,
    grid_size: int = 3,
    roi_top_ratio: float = 0.45,
    max_value: int = 85,
    min_area_px: float = 80.0,
) -> LineObservation:
    """Detect black electrical tape in the lower front-camera region.

    The returned error_x is normalized to roughly [-1, 1], where positive means
    the line center is to the right of the image center.
    """
    height, width = frame_bgr.shape[:2]
    roi_y0 = int(height * roi_top_ratio)
    roi = frame_bgr[roi_y0:height, :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([0, 0, 0], dtype=np.uint8), np.array([180, 255, max_value], dtype=np.uint8))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return LineObservation(False, None, None, 0.0, None, 0.0)

    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    if area < min_area_px:
        return LineObservation(False, None, None, 0.0, None, area)

    moments = cv2.moments(contour)
    if moments["m00"] == 0:
        return LineObservation(False, None, None, 0.0, None, area)

    center_x = float(moments["m10"] / moments["m00"])
    center_y = float(roi_y0 + moments["m01"] / moments["m00"])
    error_x = (center_x - (width / 2.0)) / (width / 2.0)
    return LineObservation(
        found=True,
        center_x=center_x,
        center_y=center_y,
        error_x=error_x,
        cell=grid_cell(center_x, center_y, width, height, grid_size),
        area_px=area,
    )


def compute_line_trace_cmd(
    obs: LineObservation,
    *,
    linear_speed: float,
    k_angular: float,
    max_angular: float,
    target_col: int = 2,
) -> VelocityCommand:
    if not obs.found:
        return VelocityCommand()
    angular = clamp(-k_angular * obs.error_x, max_angular)
    if obs.cell is not None and obs.cell[1] == target_col:
        angular *= 0.5
    return VelocityCommand(linear_x=linear_speed, angular_z=angular)
