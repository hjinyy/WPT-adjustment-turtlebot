from wpt_adjustment_turtlebot.controller_math import (
    AlignmentError,
    VelocityCommand,
    block_reverse_linear_cmd,
    is_undershoot_aligned,
)


def test_undershoot_alignment_accepts_target_approach_side_only():
    err = AlignmentError(x=1.0, y=-2.0, angle_deg=0.5)

    assert is_undershoot_aligned(
        err,
        threshold_x_px=2.0,
        undershoot_y_px=3.0,
        overshoot_y_px=0.5,
        threshold_angle_deg=1.0,
        approach_y_sign=-1,
    )


def test_undershoot_alignment_rejects_same_size_overshoot():
    err = AlignmentError(x=1.0, y=2.0, angle_deg=0.5)

    assert not is_undershoot_aligned(
        err,
        threshold_x_px=2.0,
        undershoot_y_px=3.0,
        overshoot_y_px=0.5,
        threshold_angle_deg=1.0,
        approach_y_sign=-1,
    )


def test_block_reverse_linear_cmd_keeps_forward_and_zeroes_reverse():
    forward = block_reverse_linear_cmd(VelocityCommand(linear_x=0.01, angular_z=0.02), forward_linear_sign=1)
    reverse = block_reverse_linear_cmd(VelocityCommand(linear_x=-0.01, angular_z=0.02), forward_linear_sign=1)

    assert forward.linear_x == 0.01
    assert forward.angular_z == 0.02
    assert reverse.linear_x == 0.0
    assert reverse.angular_z == 0.02
