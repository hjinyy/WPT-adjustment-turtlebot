import pytest

from wpt_adjustment_turtlebot.macs_link import LegPlan, heading_between, parse_node


def test_parse_node():
    assert parse_node("A01") == (0, 0)
    assert parse_node("B03") == (1, 2)
    assert parse_node("H06") == (7, 5)
    assert parse_node("AA10") == (26, 9)
    with pytest.raises(ValueError):
        parse_node("??")


def test_heading_matches_web_map():
    # north = up = decreasing row; east = increasing col.
    assert heading_between("A03", "A02") == "north"
    assert heading_between("A02", "A03") == "south"
    assert heading_between("A02", "B02") == "east"
    assert heading_between("B02", "A02") == "west"
    assert heading_between("A02", "B03") is None  # diagonal
    assert heading_between("A02", "A02") is None


def test_leg_plan_from_path():
    plan = LegPlan.from_path(["A01", "A02", "A03", "B03"], is_workspace_target=True)
    assert [leg[2] for leg in plan.legs] == ["south", "south", "east"]
    assert plan.legs[0] == ("A01", "A02", "south")
    assert plan.is_workspace_target


def test_leg_plan_rejects_non_adjacent():
    with pytest.raises(ValueError):
        LegPlan.from_path(["A01", "C01"], is_workspace_target=False)
