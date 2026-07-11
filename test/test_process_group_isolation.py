from pathlib import Path


def test_navigation_process_is_isolated_from_terminal_sigint():
    root = Path(__file__).resolve().parents[1]
    text = (root / "start_alignment.sh").read_text(encoding="utf-8")

    assert "setsid ros2 run wpt_adjustment_turtlebot global_map_navigation" in text
