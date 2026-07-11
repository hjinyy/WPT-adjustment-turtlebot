from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_navigation_node_records_commands_and_odom_feedback():
    text = (ROOT / "wpt_adjustment_turtlebot" / "global_map_navigation.py").read_text(encoding="utf-8")

    assert "from nav_msgs.msg import Odometry" in text
    assert "self.create_subscription(Odometry, \"/odom\"" in text
    assert "self.run_log.telemetry(" in text
    assert "self.run_log.event(" in text
    assert "node.run_log.close()" in text


def test_package_declares_nav_msgs_dependency():
    assert "<depend>nav_msgs</depend>" in (ROOT / "package.xml").read_text(encoding="utf-8")
