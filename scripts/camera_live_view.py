#!/usr/bin/env python3
"""Show a raw live feed from one or more cameras (no tag detection).

Quick way to confirm a camera is wired to the port you expect before running
the AprilTag scripts.

    python3 scripts/camera_live_view.py --device 0
    python3 scripts/camera_live_view.py --all   # front=0, right=2, left=4

Needs a display: a monitor + desktop on the Pi itself, or `ssh -X`/`-Y` X11
forwarding from your laptop (see README).
"""

from __future__ import annotations

import argparse

import cv2

DEFAULT_DEVICES = {"front": 0, "right": 2, "left": 4}


def open_camera(device: int, width: int, height: int) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap


def run(cameras: dict[str, cv2.VideoCapture]) -> None:
    print("press q to quit")
    try:
        while True:
            for name, cap in cameras.items():
                ok, frame = cap.read()
                if ok:
                    cv2.imshow(name, frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        for cap in cameras.values():
            cap.release()
        cv2.destroyAllWindows()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", type=int, help="single camera device index to view (e.g. 0, 2, 4)")
    parser.add_argument("--all", action="store_true", help="view front(0)/right(2)/left(4) at once")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()

    if args.all:
        devices = DEFAULT_DEVICES
    elif args.device is not None:
        devices = {f"camera{args.device}": args.device}
    else:
        parser.error("specify --device N or --all")
        return

    cameras = {name: open_camera(device, args.width, args.height) for name, device in devices.items()}
    for name, cap in cameras.items():
        if not cap.isOpened():
            print(f"warning: camera '{name}' (device {devices[name]}) did not open")

    run(cameras)


if __name__ == "__main__":
    main()
