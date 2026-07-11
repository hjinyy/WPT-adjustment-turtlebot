from pathlib import Path


def test_alignment_script_supports_safe_dry_run_override():
    root = Path(__file__).resolve().parents[1]
    text = (root / "start_alignment.sh").read_text(encoding="utf-8")

    assert 'DRY_RUN="${WPT_DRY_RUN:-false}"' in text
    assert '-p dry_run:="$DRY_RUN"' in text
