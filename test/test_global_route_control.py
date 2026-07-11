import math

import pytest

from wpt_adjustment_turtlebot.global_map import nearest_coil
from wpt_adjustment_turtlebot.global_route_control import RouteFollower
from wpt_adjustment_turtlebot.map_localization import MapPose


def test_nearest_coil_uses_global_pose():
    assert nearest_coil(0.20, -0.11) == "coil_4"


def test_reverse_facing_robot_rotates_without_linear_motion():
    follower = RouteFollower([(0.2265, -0.1270), (-0.2265, -0.1270)])

    cmd, complete = follower.compute(MapPose(0.2265, -0.1270, 0.0), line_angular_z=0.0)

    assert not complete
    assert cmd.linear_x == 0.0
    assert abs(cmd.angular_z) > 0.0


def test_aligned_robot_moves_forward_and_blends_line_correction():
    follower = RouteFollower([(0.2265, -0.1270), (-0.2265, -0.1270)])

    cmd, complete = follower.compute(MapPose(0.2265, -0.1270, math.pi), line_angular_z=0.03)

    assert not complete
    assert cmd.linear_x > 0.0
    assert cmd.angular_z == pytest.approx(0.03)


def test_final_waypoint_stops_route():
    follower = RouteFollower([(0.2265, -0.1270), (-0.2265, -0.1270)])

    cmd, complete = follower.compute(MapPose(-0.2265, -0.1270, math.pi), line_angular_z=0.0)

    assert complete
    assert cmd.linear_x == 0.0
    assert cmd.angular_z == 0.0
