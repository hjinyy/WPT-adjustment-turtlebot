from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_alignment_script_accepts_target_coil_and_waits_for_enter():
    text = (ROOT / "start_alignment.sh").read_text(encoding="utf-8")

    assert 'TARGET_COIL="${1:-coil_3}"' in text
    assert '-p target_coil:="$TARGET_COIL"' in text
    assert "read -r" in text
    assert "/wpt_alignment/start" in text


def test_scripts_source_ros_before_enabling_strict_shell_options():
    for name in ("start_bringup.sh", "start_alignment.sh"):
        lines = (ROOT / name).read_text(encoding="utf-8").splitlines()
        source_index = next(i for i, line in enumerate(lines) if line.startswith("source /opt/ros/"))
        assert all("set -u" not in line and "set -euo" not in line for line in lines[: source_index + 1])
