from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_alignment_cleanup_is_bounded_and_non_reentrant():
    text = (ROOT / "start_alignment.sh").read_text(encoding="utf-8")

    assert "trap - INT TERM EXIT" in text
    assert "timeout 2 ros2 service call /wpt_alignment/stop" in text
    assert "timeout 2 ros2 topic pub --once /cmd_vel" in text


def test_node_handles_external_shutdown_without_invalid_publish():
    text = (ROOT / "wpt_adjustment_turtlebot" / "global_map_navigation.py").read_text(encoding="utf-8")

    assert "ExternalShutdownException" in text
    assert "except (KeyboardInterrupt, ExternalShutdownException):" in text
    assert "if rclpy.ok():" in text
