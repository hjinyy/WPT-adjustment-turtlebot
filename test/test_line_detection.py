import cv2
import numpy as np

from wpt_adjustment_turtlebot.line_detection import LineDetector


def _blank_frame(width=320, height=240):
    return np.full((height, width, 3), 255, dtype=np.uint8)


def test_detects_centered_vertical_black_line():
    frame = _blank_frame()
    cv2.line(frame, (160, 30), (160, 239), (0, 0, 0), 12)

    detector = LineDetector(threshold=80, min_area_px=100, roi_top_ratio=0.0, roi_bottom_ratio=1.0)
    obs = detector.detect(frame)

    assert obs is not None
    assert abs(obs.center_x - 160) <= 1
    assert abs(obs.x_error) <= 1
    assert abs(obs.angle_error_deg) <= 2
    assert obs.confidence > 0.5


def test_detects_slanted_line_angle_relative_to_forward_axis():
    frame = _blank_frame()
    cv2.line(frame, (135, 239), (180, 20), (0, 0, 0), 10)

    detector = LineDetector(threshold=80, min_area_px=100, roi_top_ratio=0.0, roi_bottom_ratio=1.0)
    obs = detector.detect(frame)

    assert obs is not None
    assert obs.angle_error_deg > 5


def test_missing_line_returns_none_instead_of_false_zero():
    detector = LineDetector(threshold=80, min_area_px=100, roi_top_ratio=0.0, roi_bottom_ratio=1.0)

    assert detector.detect(_blank_frame()) is None


def test_rejects_large_horizontal_robot_wheel_as_line():
    frame = _blank_frame()
    cv2.rectangle(frame, (10, 150), (310, 235), (0, 0, 0), -1)

    detector = LineDetector(
        threshold=80,
        min_area_px=100,
        roi_top_ratio=0.0,
        roi_bottom_ratio=0.55,
        roi_left_ratio=0.20,
        roi_right_ratio=0.80,
        max_abs_angle_deg=45.0,
        min_vertical_span_ratio=0.20,
    )

    assert detector.detect(frame) is None
