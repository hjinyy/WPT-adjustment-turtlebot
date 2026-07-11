from pathlib import Path

import yaml

from wpt_adjustment_turtlebot.tag_layout import FOUR_COIL_TAGS


ROOT = Path(__file__).resolve().parents[1]


def test_canonical_coil_3_and_4_marker_layout_matches_physical_map():
    assert FOUR_COIL_TAGS["coil_3"] == {"north": 31, "south": 32, "west": 33, "east": 34}
    assert FOUR_COIL_TAGS["coil_4"] == {"north": 41, "south": 42, "west": 43, "east": 44}


def test_yaml_uses_canonical_coil_3_and_4_marker_layout():
    config = yaml.safe_load((ROOT / "config" / "wpt_alignment.yaml").read_text(encoding="utf-8"))

    assert config["coils"]["coil_3"]["markers"] == {"north": 31, "south": 32, "west": 33, "east": 34}
    assert config["coils"]["coil_4"]["markers"] == {"north": 41, "south": 42, "west": 43, "east": 44}
