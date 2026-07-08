#!/usr/bin/env python3
"""Standalone experiment: check whether AprilTags sit inside a target cell of a
3x3 grid on all three cameras (front/right/left) at once.

Run directly on the robot (not through ROS2/colcon):
    python3 scripts/camera_grid_alignment.py
    python3 scripts/camera_grid_alignment.py --show
    python3 scripts/camera_grid_alignment.py --target-row 2 --target-col 2

The target cell defaults to the center (2, 2) but is only a starting guess —
expect to adjust --target-row/--target-col once this has been tried on the
real hardware.
"""

from __future__ import annotations

import argparse
import time

import cv2

from wpt_adjustment_turtlebot.controller_math import grid_cell
from wpt_adjustment_turtlebot.wpt_alignment_node import AprilTagDetector


def open_camera(device: int, width: int, height: int) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap


def draw_grid(frame, target_cell: tuple[int, int], grid_size: int) -> None:
    h, w = frame.shape[:2]
    for i in range(1, grid_size):
        x = round(w * i / grid_size)
        y = round(h * i / grid_size)
        cv2.line(frame, (x, 0), (x, h), (80, 80, 80), 1)
        cv2.line(frame, (0, y), (w, y), (80, 80, 80), 1)
    row, col = target_cell
    x0, x1 = round(w * (col - 1) / grid_size), round(w * col / grid_size)
    y0, y1 = round(h * (row - 1) / grid_size), round(h * row / grid_size)
    cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 200, 0), 2)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--front-device", type=int, default=0)
    parser.add_argument("--right-device", type=int, default=2)
    parser.add_argument("--left-device", type=int, default=4)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--family", default="tag36h11")
    parser.add_argument("--grid-size", type=int, default=3)
    parser.add_argument("--target-row", type=int, default=2)
    parser.add_argument("--target-col", type=int, default=2)
    parser.add_argument("--show", action="store_true", help="open OpenCV preview windows (needs a display)")
    args = parser.parse_args()

    target_cell = (args.target_row, args.target_col)
    cameras = {
        "front": open_camera(args.front_device, args.width, args.height),
        "right": open_camera(args.right_device, args.width, args.height),
        "left": open_camera(args.left_device, args.width, args.height),
    }
    devices = {"front": args.front_device, "right": args.right_device, "left": args.left_device}
    for name, cap in cameras.items():
        if not cap.isOpened():
            print(f"warning: camera '{name}' (device {devices[name]}) did not open")

    detector = AprilTagDetector(args.family)
    print(f"AprilTag backend: {detector.backend}")
    if detector.backend == "none":
        print("no AprilTag backend available; install pupil-apriltags or use an OpenCV build with cv2.aruco")
        return

    show_enabled = args.show
    try:
        while True:
            per_camera = {}
            for name, cap in cameras.items():
                ok, frame = cap.read()
                if not ok:
                    per_camera[name] = (False, None, None)
                    continue
                h, w = frame.shape[:2]
                detections = detector.detect(frame)
                cell = None
                tag_id = None
                if detections:
                    best = max(detections, key=lambda d: d.area_px)
                    cell = grid_cell(best.center[0], best.center[1], w, h, args.grid_size)
                    tag_id = best.tag_id
                per_camera[name] = (cell == target_cell, tag_id, cell)

                if show_enabled:
                    draw_grid(frame, target_cell, args.grid_size)
                    for d in detections:
                        x, y = int(d.center[0]), int(d.center[1])
                        cv2.circle(frame, (x, y), 6, (0, 0, 255), -1)
                    try:
                        cv2.imshow(name, frame)
                    except cv2.error as exc:
                        print(f"cv2.imshow failed ({exc}); re-run without --show, or set up a display (local desktop or ssh -X)")
                        show_enabled = False

            all_aligned = all(v[0] for v in per_camera.values())
            status = " | ".join(
                f"{name}: id={tag_id} cell={cell} {'ALIGNED' if aligned else '-'}"
                for name, (aligned, tag_id, cell) in per_camera.items()
            )
            print(f"target={target_cell} {status} -> ALL_ALIGNED={all_aligned}")

            if show_enabled:
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            else:
                time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        for cap in cameras.values():
            cap.release()
        if show_enabled:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
