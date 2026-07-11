"""Black tape line detection for top-down TurtleBot cameras."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, degrees

import cv2


@dataclass(frozen=True)
class LineObservation:
    center_x: float
    center_y: float
    x_error: float
    angle_error_deg: float
    confidence: float
    area_px: float


class LineDetector:
    """Detect a dark floor line and express it as image-space control errors.

    The forward direction is the top of the image. A perfectly aligned line is
    vertical, centered at frame_width / 2, with angle_error_deg close to 0.
    """

    def __init__(
        self,
        *,
        threshold: int = 80,
        min_area_px: float = 120.0,
        roi_top_ratio: float = 0.0,
        roi_bottom_ratio: float = 1.0,
        blur_kernel: int = 5,
    ) -> None:
        self.threshold = int(threshold)
        self.min_area_px = float(min_area_px)
        self.roi_top_ratio = float(roi_top_ratio)
        self.roi_bottom_ratio = float(roi_bottom_ratio)
        self.blur_kernel = int(blur_kernel) if int(blur_kernel) % 2 == 1 else int(blur_kernel) + 1

    def detect(self, frame_bgr) -> LineObservation | None:
        height, width = frame_bgr.shape[:2]
        y0 = max(0, min(height - 1, int(round(height * self.roi_top_ratio))))
        y1 = max(y0 + 1, min(height, int(round(height * self.roi_bottom_ratio))))
        roi = frame_bgr[y0:y1, :]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        if self.blur_kernel > 1:
            gray = cv2.GaussianBlur(gray, (self.blur_kernel, self.blur_kernel), 0)
        _unused, mask = cv2.threshold(gray, self.threshold, 255, cv2.THRESH_BINARY_INV)
        contours, _hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        if area < self.min_area_px:
            return None

        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            return None
        center_x = float(moments["m10"] / moments["m00"])
        center_y = float(moments["m01"] / moments["m00"]) + y0

        vx, vy, _x, _y = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
        if vy < 0:
            vx, vy = -vx, -vy
        angle_error = -degrees(atan2(float(vx), float(vy)))
        confidence = max(0.0, min(1.0, area / max(1.0, width * (y1 - y0) * 0.05)))
        return LineObservation(
            center_x=center_x,
            center_y=center_y,
            x_error=center_x - (width / 2.0),
            angle_error_deg=angle_error,
            confidence=confidence,
            area_px=area,
        )
