from pathlib import Path


def test_cleanup_terminates_the_isolated_process_group():
    root = Path(__file__).resolve().parents[1]
    text = (root / "start_alignment.sh").read_text(encoding="utf-8")

    assert 'kill -TERM -- "-$PID"' in text
    assert 'kill -KILL -- "-$PID"' in text
