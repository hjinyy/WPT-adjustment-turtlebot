#!/usr/bin/env python3
"""Capture all configured cameras and report AprilTag alignment status.

This is a dry-run diagnostic script. It does not use ROS and never publishes
/cmd_vel. Run it on the machine that has /dev/video0, /dev/video2, /dev/video4.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

import cv2
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from wpt_adjustment_turtlebot.controller_math import (  # noqa: E402
    TagObservation,
    TargetPoseInImage,
    compute_pair_alignment_error,
    compute_pair_observation,
    is_aligned,
)
from wpt_adjustment_turtlebot.tag_layout import (  # noqa: E402
    decode_four_coil_tag,
    four_coil_pair_ids,
)
from wpt_adjustment_turtlebot.wpt_alignment_node import AprilTagDetector  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Dry-run AprilTag camera alignment check.")
    parser.add_argument("--config", default=str(REPO_ROOT / "config" / "wpt_alignment.yaml"))
    parser.add_argument("--target-coil", default=None, choices=["coil_1", "coil_2", "coil_3", "coil_4"])
    parser.add_argument("--pair", default=None, choices=["west_east", "east_west", "north_south", "south_north"])
    parser.add_argument("--frames", type=int, default=1, help="Number of diagnostic frames to capture.")
    parser.add_argument("--warmup", type=int, default=10, help="Frames to discard before checking.")
    parser.add_argument("--delay", type=float, default=0.2, help="Delay between diagnostic frames.")
    parser.add_argument("--read-retries", type=int, default=3, help="Retries when a camera frame read times out.")
    parser.add_argument("--cross-camera-only", action="store_true", help="Only print cross-camera pair status.")
    parser.add_argument("--log-file", default="", help="Optional CSV file for cross-camera pair status.")
    parser.add_argument("--output-dir", default="camera_alignment_check", help="Annotated image output directory.")
    parser.add_argument("--no-save", action="store_true", help="Do not save annotated images.")
    return parser.parse_args()


def load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def open_camera(name: str, cfg: dict) -> cv2.VideoCapture:
    device = int(cfg["device"])
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if cfg.get("fourcc"):
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*str(cfg["fourcc"])[:4]))
    if cfg.get("width"):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cfg["width"]))
    if cfg.get("height"):
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cfg["height"]))
    if cfg.get("fps"):
        cap.set(cv2.CAP_PROP_FPS, float(cfg["fps"]))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        raise RuntimeError(f"{name}: cannot open /dev/video{device}")
    print(
        f"opened {name}: /dev/video{device} "
        f"{int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} "
        f"fps={cap.get(cv2.CAP_PROP_FPS):.1f} fourcc={fourcc_to_str(cap.get(cv2.CAP_PROP_FOURCC))}"
    )
    return cap


def fourcc_to_str(value: float) -> str:
    code = int(value)
    return "".join(chr((code >> 8 * i) & 0xFF) for i in range(4)).strip()


def read_all_camera_frames(cameras: dict[str, cv2.VideoCapture], retries: int) -> dict[str, tuple[bool, object | None]]:
    frames: dict[str, tuple[bool, object | None]] = {camera_name: (False, None) for camera_name in cameras}
    for _ in range(max(1, retries)):
        grabbed = {
            camera_name: cap.grab()
            for camera_name, cap in cameras.items()
            if not frames[camera_name][0]
        }
        for camera_name, cap in cameras.items():
            if frames[camera_name][0] or not grabbed.get(camera_name, False):
                continue
            ok, frame = cap.retrieve()
            if ok:
                frames[camera_name] = (True, frame)
        if all(ok for ok, _frame in frames.values()):
            break
        time.sleep(0.03)
    return frames


def detect_frame(detector: AprilTagDetector, camera_name: str, frame) -> list[TagObservation]:
    observations = []
    for det in detector.detect(frame):
        observations.append(
            TagObservation(
                tag_id=det.tag_id,
                center_x=det.center[0],
                center_y=det.center[1],
                angle_deg=det.angle_deg,
                area_px=det.area_px,
                camera_name=camera_name,
            )
        )
    return observations


def target_pose(config: dict, target_coil: str, camera_name: str, pair_name: str) -> TargetPoseInImage:
    coil_cfg = config["coils"][target_coil]
    camera_targets = coil_cfg["targets"][camera_name]
    raw = camera_targets.get(pair_name, camera_targets["default"])
    return TargetPoseInImage(float(raw["x"]), float(raw["y"]), float(raw["angle_deg"]))


def best_observation(observations: list[TagObservation], tag_id: int) -> TagObservation | None:
    candidates = [o for o in observations if o.tag_id == tag_id]
    return max(candidates, key=lambda o: o.area_px, default=None)


def describe_tag(tag_id: int) -> str:
    decoded = decode_four_coil_tag(tag_id)
    if decoded is None:
        return "not four-coil-layout tag"
    coil_name, position = decoded
    return f"{coil_name} {position}"


def annotate(frame, observations: list[TagObservation], required_ids: tuple[int, int], aligned: bool | None):
    color_required = (0, 255, 255)
    color_other = (255, 180, 0)
    for obs in observations:
        color = color_required if obs.tag_id in required_ids else color_other
        x = int(round(obs.center_x))
        y = int(round(obs.center_y))
        cv2.circle(frame, (x, y), 5, color, -1)
        cv2.putText(
            frame,
            f"id {obs.tag_id}",
            (x + 8, y - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )
    if aligned is not None:
        status = "ALIGNED" if aligned else "NOT ALIGNED"
        color = (0, 200, 0) if aligned else (0, 0, 255)
        cv2.putText(frame, status, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2, cv2.LINE_AA)
    return frame


def report_camera(
    config: dict,
    detector: AprilTagDetector,
    camera_name: str,
    frame,
    target_coil: str,
    pair_name: str,
):
    observations = detect_frame(detector, camera_name, frame)
    first_id, second_id = four_coil_pair_ids(target_coil, pair_name)
    print(f"\n[{camera_name}] visible_tags={len(observations)} required_ids=({first_id},{second_id})")
    for obs in sorted(observations, key=lambda o: o.tag_id):
        print(
            f"  tag_id={obs.tag_id} meaning='{describe_tag(obs.tag_id)}' "
            f"center=({obs.center_x:.1f},{obs.center_y:.1f}) angle={obs.angle_deg:.1f} area={obs.area_px:.1f}"
        )

    first = best_observation(observations, first_id)
    second = best_observation(observations, second_id)
    if first is None or second is None:
        visible_pair_ids = [obs.tag_id for obs in observations if obs.tag_id in {first_id, second_id}]
        print(f"  camera_pair_view=partial_or_none visible_pair_ids={visible_pair_ids}")
        return observations, None, False

    pair = compute_pair_observation(first, second)
    target = target_pose(config, target_coil, camera_name, pair_name)
    err = compute_pair_alignment_error(pair, target)
    thresholds = config["alignment"]["coil"]
    aligned = is_aligned(
        err,
        threshold_x_px=float(thresholds["threshold_x_px"]),
        threshold_y_px=float(thresholds["threshold_y_px"]),
        threshold_angle_deg=float(thresholds["threshold_angle_deg"]),
    )
    print(
        f"  pair_status=found selected_pair={pair_name} "
        f"pair_mid_x={pair.midpoint_x:.2f} pair_mid_y={pair.midpoint_y:.2f} "
        f"pair_angle_deg={pair.pair_angle_deg:.2f}"
    )
    print(
        f"  target_x={target.x:.2f} target_y={target.y:.2f} target_angle_deg={target.angle_deg:.2f} "
        f"x_error={err.x:.2f} y_error={err.y:.2f} angle_error={err.angle_deg:.2f} aligned={aligned}"
    )
    return observations, pair, aligned


def cross_camera_pair_status(observations: list[TagObservation], target_coil: str, pair_name: str) -> dict[str, object]:
    first_id, second_id = four_coil_pair_ids(target_coil, pair_name)
    first = best_observation(observations, first_id)
    second = best_observation(observations, second_id)
    visible_ids = sorted({obs.tag_id for obs in observations})
    status: dict[str, object] = {
        "pair": pair_name,
        "first_id": first_id,
        "second_id": second_id,
        "visible_ids": visible_ids,
        "pair_presence": first is not None and second is not None,
        "aligned": first is not None and second is not None,
        "first_camera": "" if first is None else first.camera_name,
        "first_x": "" if first is None else first.center_x,
        "first_y": "" if first is None else first.center_y,
        "second_camera": "" if second is None else second.camera_name,
        "second_x": "" if second is None else second.center_x,
        "second_y": "" if second is None else second.center_y,
        "missing_ids": [tag_id for tag_id, obs in ((first_id, first), (second_id, second)) if obs is None],
    }
    return status


def report_cross_camera_pair(observations: list[TagObservation], target_coil: str, pair_name: str) -> dict[str, object]:
    status = cross_camera_pair_status(observations, target_coil, pair_name)
    first_id = int(status["first_id"])
    second_id = int(status["second_id"])
    visible_ids = status["visible_ids"]
    if not status["pair_presence"]:
        print(
            f"\n[cross_camera] selected_pair={pair_name} required_ids=({first_id},{second_id}) "
            f"visible_ids={visible_ids} pair_presence=False aligned=False "
            f"missing_ids=({','.join(str(x) for x in status['missing_ids'])})"
        )
        return status

    print(
        f"\n[cross_camera] selected_pair={pair_name} required_ids=({first_id},{second_id}) "
        f"visible_ids={visible_ids} pair_presence=True aligned=True"
    )
    print(
        f"  marker_a=id={first_id} camera={status['first_camera']} "
        f"center=({float(status['first_x']):.1f},{float(status['first_y']):.1f})"
    )
    print(
        f"  marker_b=id={second_id} camera={status['second_camera']} "
        f"center=({float(status['second_x']):.1f},{float(status['second_y']):.1f})"
    )
    return status


def print_cross_camera_line(frame_index: int, elapsed_sec: float, status: dict[str, object]) -> None:
    print(
        f"t={elapsed_sec:.2f}s frame={frame_index} "
        f"pair_presence={status['pair_presence']} aligned={status['aligned']} "
        f"id{status['first_id']}_camera={status['first_camera'] or 'None'} "
        f"id{status['second_id']}_camera={status['second_camera'] or 'None'} "
        f"missing={status['missing_ids']}"
    )


def write_cross_camera_row(writer, frame_index: int, elapsed_sec: float, status: dict[str, object]) -> None:
    writer.writerow(
        {
            "elapsed_sec": f"{elapsed_sec:.3f}",
            "frame": frame_index,
            "pair": status["pair"],
            "pair_presence": status["pair_presence"],
            "aligned": status["aligned"],
            "first_id": status["first_id"],
            "first_camera": status["first_camera"],
            "first_x": "" if status["first_x"] == "" else f"{float(status['first_x']):.2f}",
            "first_y": "" if status["first_y"] == "" else f"{float(status['first_y']):.2f}",
            "second_id": status["second_id"],
            "second_camera": status["second_camera"],
            "second_x": "" if status["second_x"] == "" else f"{float(status['second_x']):.2f}",
            "second_y": "" if status["second_y"] == "" else f"{float(status['second_y']):.2f}",
            "visible_ids": " ".join(str(tag_id) for tag_id in status["visible_ids"]),
            "missing_ids": " ".join(str(tag_id) for tag_id in status["missing_ids"]),
        }
    )


def main() -> int:
    args = parse_args()
    config = load_config(args.config)
    target_coil = args.target_coil or str(config.get("target_coil", "coil_1")).lower()
    pair_name = args.pair or str(config["alignment"].get("final_pair", "west_east"))
    required_ids = four_coil_pair_ids(target_coil, pair_name)
    detector = AprilTagDetector(config["apriltag"].get("family", "tag36h11"))

    print("dry_run=True")
    print(f"apriltag_backend={detector.backend}")
    print(f"layout_mode={config.get('layout_mode')} target_coil={target_coil} selected_pair={pair_name}")
    print(f"required_ids={required_ids}")
    print("cmd_vel_publish=False")

    output_dir = Path(args.output_dir)
    save_images = not args.no_save and not args.cross_camera_only
    if save_images:
        output_dir.mkdir(parents=True, exist_ok=True)

    cameras = {}
    log_file = None
    csv_writer = None
    try:
        if args.log_file:
            log_file = open(args.log_file, "w", newline="", encoding="utf-8")
            csv_writer = csv.DictWriter(
                log_file,
                fieldnames=[
                    "elapsed_sec",
                    "frame",
                    "pair",
                    "pair_presence",
                    "aligned",
                    "first_id",
                    "first_camera",
                    "first_x",
                    "first_y",
                    "second_id",
                    "second_camera",
                    "second_x",
                    "second_y",
                    "visible_ids",
                    "missing_ids",
                ],
            )
            csv_writer.writeheader()

        for camera_name, camera_cfg in config["cameras"].items():
            cameras[camera_name] = open_camera(camera_name, camera_cfg)

        for _ in range(max(0, args.warmup)):
            read_all_camera_frames(cameras, args.read_retries)
            time.sleep(0.03)

        start_time = time.monotonic()
        for frame_index in range(args.frames):
            if not args.cross_camera_only:
                print(f"\n=== frame {frame_index} ===")
            frames = read_all_camera_frames(cameras, args.read_retries)
            frame_observations = []
            for camera_name, (ok, frame) in frames.items():
                if not ok:
                    if not args.cross_camera_only:
                        print(f"\n[{camera_name}] read=False aligned=False")
                    continue
                if args.cross_camera_only:
                    observations = detect_frame(detector, camera_name, frame)
                    aligned = False
                else:
                    observations, _pair, aligned = report_camera(config, detector, camera_name, frame, target_coil, pair_name)
                frame_observations.extend(observations)
                if save_images:
                    annotated = annotate(frame.copy(), observations, required_ids, aligned)
                    out = output_dir / f"{camera_name}_frame_{frame_index}.jpg"
                    cv2.imwrite(str(out), annotated)
                    print(f"  saved={out}")
            elapsed_sec = time.monotonic() - start_time
            if args.cross_camera_only:
                status = cross_camera_pair_status(frame_observations, target_coil, pair_name)
                print_cross_camera_line(frame_index, elapsed_sec, status)
            else:
                status = report_cross_camera_pair(frame_observations, target_coil, pair_name)
            if csv_writer is not None:
                write_cross_camera_row(csv_writer, frame_index, elapsed_sec, status)
                log_file.flush()
            if frame_index + 1 < args.frames:
                time.sleep(args.delay)
    finally:
        if log_file is not None:
            log_file.close()
        for cap in cameras.values():
            cap.release()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
