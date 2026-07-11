import csv

from wpt_adjustment_turtlebot.run_logging import RunLogger


def test_run_logger_creates_per_run_csv_and_event_log(tmp_path):
    logger = RunLogger(tmp_path, target_coil="coil_3", run_name="test_run")
    logger.event("주행 시작")
    logger.telemetry(
        state="ROTATE_TO_ROUTE",
        tag_ids="41;42",
        filtered_x_m=0.2,
        filtered_y_m=-0.1,
        filtered_yaw_rad=1.2,
        cmd_linear_x=0.0,
        cmd_angular_z=-0.18,
        odom_linear_x=0.0,
        odom_angular_z=-0.02,
        reason="heading_error",
    )
    logger.close()

    run_dir = tmp_path / "test_run"
    assert "주행 시작" in (run_dir / "events.log").read_text(encoding="utf-8")
    rows = list(csv.DictReader((run_dir / "telemetry.csv").open(encoding="utf-8")))
    assert rows[0]["target_coil"] == "coil_3"
    assert rows[0]["cmd_angular_z"] == "-0.18"
    assert rows[0]["odom_angular_z"] == "-0.02"


def test_run_logger_writes_empty_values_for_unavailable_sensors(tmp_path):
    logger = RunLogger(tmp_path, target_coil="coil_1", run_name="missing")
    logger.telemetry(state="LOCALIZE", reason="no_tags")
    logger.close()

    row = next(csv.DictReader((tmp_path / "missing" / "telemetry.csv").open(encoding="utf-8")))
    assert row["filtered_x_m"] == ""
    assert row["odom_linear_x"] == ""
