import pytest

from wpt_adjustment_turtlebot.coil_transit import (
    compute_line_follow_cmd,
    make_leg,
    normalize_coil_name,
    plan_transit_legs,
    stop_condition_met,
    travel_direction,
)


def test_global_compass_directions():
    # 1|2 / 3|4 grid, one shared compass for every coil.
    assert travel_direction("coil_1", "coil_3") == "south"
    assert travel_direction("coil_3", "coil_1") == "north"
    assert travel_direction("coil_1", "coil_2") == "east"
    assert travel_direction("coil_4", "coil_3") == "west"
    assert travel_direction("coil_1", "coil_4") is None  # diagonal: not one leg
    assert travel_direction("coil_1", "coil_1") is None


def test_coil_1_to_coil_3_head_and_stop_markers():
    # The user's reference scenario: depart coil_1 heading south, the head
    # marker is coil_1's south tag (12), and the robot stops the moment it
    # sees coil_3's south tag (32).
    leg = make_leg("coil_1", "coil_3")
    assert leg.direction == "south"
    assert leg.head_marker_id == 12
    assert leg.stop_marker_id == 32


def test_eastward_leg_markers():
    leg = make_leg("coil_1", "coil_2")
    assert leg.direction == "east"
    assert leg.head_marker_id == 14  # coil_1 east
    assert leg.stop_marker_id == 24  # coil_2 east


def test_leg_distances_match_stage_measurements():
    # Measured coil-center distances: vertical 25.5 cm, horizontal 45.3 cm.
    assert make_leg("coil_1", "coil_3").distance_m == pytest.approx(0.255)
    assert make_leg("coil_2", "coil_4").distance_m == pytest.approx(0.255)
    assert make_leg("coil_1", "coil_2").distance_m == pytest.approx(0.453)
    assert make_leg("coil_3", "coil_4").distance_m == pytest.approx(0.453)


def test_side_markers_flank_the_travel_direction():
    south = make_leg("coil_1", "coil_3")
    assert south.side_marker_ids == (33, 34)  # coil_3 west, east flank a north-south transit
    east = make_leg("coil_1", "coil_2")
    assert east.side_marker_ids == (21, 22)  # coil_2 north, south flank an east-west transit


def test_stop_condition_needs_head_plus_both_fresh_side_markers():
    sides = (33, 34)
    # Head not visible -> never stop, even with perfect side visibility.
    assert not stop_condition_met(10.0, False, {33: 10.0, 34: 10.0}, sides, 0.7)
    # Head visible but one side marker never seen -> keep approaching.
    assert not stop_condition_met(10.0, True, {33: 10.0}, sides, 0.7)
    # Head visible but a side marker is stale (>freshness window) -> keep approaching.
    assert not stop_condition_met(10.0, True, {33: 10.0, 34: 9.0}, sides, 0.7)
    # Head visible + both sides fresh -> stop this instant.
    assert stop_condition_met(10.0, True, {33: 10.0, 34: 9.5}, sides, 0.7)


def test_plan_transit_legs_adjacent_and_same():
    assert plan_transit_legs("coil_1", "coil_1") == []
    legs = plan_transit_legs("coil_2", "coil_4")
    assert len(legs) == 1
    assert legs[0].direction == "south"
    assert legs[0].stop_marker_id == 42  # coil_4 south


def test_plan_transit_legs_diagonal_goes_vertical_first():
    legs = plan_transit_legs("coil_1", "coil_4")
    assert [(leg.from_coil, leg.to_coil, leg.direction) for leg in legs] == [
        ("coil_1", "coil_3", "south"),
        ("coil_3", "coil_4", "east"),
    ]
    assert legs[0].stop_marker_id == 32
    assert legs[1].stop_marker_id == 44


def test_normalize_coil_name_accepts_ints_and_strings():
    assert normalize_coil_name(3) == "coil_3"
    assert normalize_coil_name("COIL_2") == "coil_2"
    assert normalize_coil_name("4") == "coil_4"
    with pytest.raises(ValueError):
        normalize_coil_name("coil_9")


def test_line_follow_cmd_steers_and_clamps():
    cmd = compute_line_follow_cmd(
        50.0,
        5.0,
        cruise_linear=0.05,
        k_x_to_angular=-0.01,
        k_angle_to_angular=-0.02,
        max_angular=0.3,
    )
    assert cmd.linear_x == 0.05
    assert cmd.angular_z == -0.3  # -0.5 - 0.1 clamped to max_angular

    centered = compute_line_follow_cmd(
        1.0,
        0.2,
        cruise_linear=0.05,
        k_x_to_angular=-0.01,
        k_angle_to_angular=-0.02,
        max_angular=0.3,
        x_deadband_px=3.0,
        angle_deadband_deg=1.0,
    )
    assert centered.angular_z == 0.0

    inverted = compute_line_follow_cmd(
        50.0,
        0.0,
        cruise_linear=0.05,
        k_x_to_angular=-0.01,
        k_angle_to_angular=0.0,
        max_angular=1.0,
        invert_angular=True,
    )
    assert inverted.angular_z == 0.5
