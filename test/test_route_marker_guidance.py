import pytest

from wpt_adjustment_turtlebot.global_map import expected_route_marker_ids
from wpt_adjustment_turtlebot.global_route_control import MarkerRouteGuide, NavigationRecoveryPolicy


def test_coil_4_to_3_uses_west_departure_and_goal_markers():
    assert expected_route_marker_ids("coil_4", "coil_3") == (43, 33)


def test_departure_requires_centered_43_marker():
    guide = MarkerRouteGuide(43, 33, image_width=320, center_tolerance_px=45)

    assert not guide.update_departure([(43, 80.0)])
    assert guide.update_departure([(43, 158.0)])
    assert guide.departure_acquired


def test_rotation_sign_is_preserved_when_marker_disappears():
    guide = MarkerRouteGuide(43, 33, image_width=320, center_tolerance_px=45)

    guide.update_turn_direction([41])
    assert guide.rotation_sign == 1.0
    guide.update_turn_direction([])
    assert guide.rotation_sign == 1.0


def test_goal_marker_is_33_after_departure():
    guide = MarkerRouteGuide(43, 33, image_width=320, center_tolerance_px=45)

    assert guide.goal_visible([33])
    assert not guide.goal_visible([31, 34])


def test_recovery_allows_exactly_half_second_without_line_or_marker():
    policy = NavigationRecoveryPolicy(grace_sec=0.5)

    policy.observe(now=10.0, line_seen=True, marker_seen=False)
    assert not policy.should_reacquire(now=10.50)
    assert policy.should_reacquire(now=10.51)
