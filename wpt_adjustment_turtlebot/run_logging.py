"""실행별 이벤트와 제어 텔레메트리 기록."""

from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path
import time


TELEMETRY_FIELDS = [
    "timestamp_iso",
    "elapsed_s",
    "target_coil",
    "state",
    "reason",
    "tag_ids",
    "raw_x_m",
    "raw_y_m",
    "raw_yaw_rad",
    "filtered_x_m",
    "filtered_y_m",
    "filtered_yaw_rad",
    "line_x_error_px",
    "line_angle_error_deg",
    "line_confidence",
    "waypoint_x_m",
    "waypoint_y_m",
    "cmd_linear_x",
    "cmd_angular_z",
    "odom_linear_x",
    "odom_angular_z",
]


class RunLogger:
    def __init__(self, root, *, target_coil: str, run_name: str | None = None) -> None:
        stamp = run_name or datetime.now().strftime("%Y%m%d_%H%M%S") + f"_{target_coil}"
        self.run_dir = Path(root).expanduser() / stamp
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self.target_coil = target_coil
        self.started_at = time.monotonic()
        self._csv_file = (self.run_dir / "telemetry.csv").open("w", encoding="utf-8", newline="")
        self._event_file = (self.run_dir / "events.log").open("w", encoding="utf-8")
        self._writer = csv.DictWriter(self._csv_file, fieldnames=TELEMETRY_FIELDS)
        self._writer.writeheader()
        self._csv_file.flush()

    def event(self, message: str) -> None:
        self._event_file.write(f"{datetime.now().isoformat(timespec='milliseconds')} {message}\n")
        self._event_file.flush()

    def telemetry(self, **values) -> None:
        row = {field: "" for field in TELEMETRY_FIELDS}
        row.update(values)
        row["timestamp_iso"] = datetime.now().isoformat(timespec="milliseconds")
        row["elapsed_s"] = f"{time.monotonic() - self.started_at:.3f}"
        row["target_coil"] = self.target_coil
        self._writer.writerow(row)
        self._csv_file.flush()

    def close(self) -> None:
        if not self._csv_file.closed:
            self._csv_file.close()
        if not self._event_file.closed:
            self._event_file.close()
