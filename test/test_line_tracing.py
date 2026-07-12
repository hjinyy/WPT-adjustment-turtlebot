import cv2
import numpy as np

from wpt_adjustment_turtlebot.line_tracing import compute_line_trace_cmd, detect_black_line


def test_detect_black_line_center_cell():
    frame = np.full((240, 320, 3), 220, dtype=np.uint8)
    cv2.rectangle(frame, (150, 120), (170, 239), (0, 0, 0), -1)

    obs = detect_black_line(frame, grid_size=3, roi_top_ratio=0.4)

    assert obs.found
    assert obs.cell == (3, 2)
    assert abs(obs.error_x) < 0.05


def test_line_trace_command_turns_toward_line():
    frame = np.full((240, 320, 3), 220, dtype=np.uint8)
    cv2.rectangle(frame, (220, 120), (245, 239), (0, 0, 0), -1)
    obs = detect_black_line(frame, grid_size=3, roi_top_ratio=0.4)

    cmd = compute_line_trace_cmd(obs, linear_speed=0.03, k_angular=0.7, max_angular=0.18)

    assert cmd.linear_x == 0.03
    assert cmd.angular_z < 0.0
