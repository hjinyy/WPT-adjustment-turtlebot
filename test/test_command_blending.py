from wpt_adjustment_turtlebot.controller_math import VelocityCommand, blend_marker_and_line_cmd


def test_blend_marker_and_line_cmd_keeps_marker_primary():
    marker = VelocityCommand(linear_x=0.01, angular_z=0.04)
    line = VelocityCommand(linear_x=0.03, angular_z=-0.10)

    blended = blend_marker_and_line_cmd(marker, line, line_weight=0.25, max_linear=0.02, max_angular=0.08)

    assert blended.linear_x == 0.01
    assert round(blended.angular_z, 4) == 0.015


def test_blend_marker_and_line_cmd_can_follow_line_when_marker_missing():
    line = VelocityCommand(linear_x=0.03, angular_z=-0.04)

    blended = blend_marker_and_line_cmd(None, line, line_weight=1.0, max_linear=0.02, max_angular=0.08)

    assert blended.linear_x == 0.02
    assert blended.angular_z == -0.04
