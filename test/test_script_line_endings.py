from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_shell_scripts_use_lf_only():
    for name in ("start_bringup.sh", "start_alignment.sh"):
        assert b"\r" not in (ROOT / name).read_bytes()
