import csv

from wpt_adjustment_turtlebot.run_logging import RunLogger


def test_run_logger_writes_control_timing_columns(tmp_path):
    logger = RunLogger(tmp_path, target_coil="coil_3")
    logger.telemetry(
        state="FOLLOW_LINE",
        reason="line",
        frame_capture_ms=8.0,
        tag_detect_ms=32.0,
        line_detect_ms=2.0,
        control_cycle_ms=45.0,
        control_overrun=False,
    )
    logger.close()

    with (logger.run_dir / "telemetry.csv").open(newline="", encoding="utf-8") as stream:
        row = next(csv.DictReader(stream))
    assert row["frame_capture_ms"] == "8.0"
    assert row["control_cycle_ms"] == "45.0"
    assert row["control_overrun"] == "False"
