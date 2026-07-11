import math

from wpt_adjustment_turtlebot.global_map import (
    COIL_CENTERS_M,
    TAG_SIZE_M,
    build_tag_world_poses,
    plan_axis_aligned_route,
)


def test_global_map_uses_center_origin_and_suffix_direction_rule():
    poses = build_tag_world_poses()

    assert TAG_SIZE_M == 0.015
    assert COIL_CENTERS_M["coil_1"] == (-0.2265, 0.1270)
    assert COIL_CENTERS_M["coil_4"] == (0.2265, -0.1270)
    assert poses[11] == (-0.2265, 0.2245)
    assert poses[12] == (-0.2265, 0.0295)
    assert poses[13] == (-0.3240, 0.1270)
    assert poses[14] == (-0.1290, 0.1270)
    assert poses[31] == (-0.2265, -0.0295)
    assert poses[32] == (-0.2265, -0.2245)
    assert poses[43] == (0.1290, -0.1270)
    assert poses[44] == (0.3240, -0.1270)


def test_route_planner_inserts_turn_for_diagonal_coils():
    route = plan_axis_aligned_route("coil_1", "coil_4")

    assert route == [(-0.2265, 0.1270), (0.2265, 0.1270), (0.2265, -0.1270)]


def test_route_planner_keeps_same_row_route_straight():
    assert plan_axis_aligned_route("coil_4", "coil_3") == [
        (0.2265, -0.1270),
        (-0.2265, -0.1270),
    ]
