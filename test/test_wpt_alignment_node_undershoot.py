from wpt_adjustment_turtlebot.controller_math import AlignmentError, VelocityCommand
from wpt_adjustment_turtlebot.wpt_alignment_node import WptAlignmentNode


def make_node_with_undershoot_config(enabled=True):
    node = WptAlignmentNode.__new__(WptAlignmentNode)
    node.config = {
        "alignment": {
            "coil": {"threshold_x_px": 4, "threshold_y_px": 4, "threshold_angle_deg": 2.0},
            "undershoot_stop": {
                "enabled": enabled,
                "approach_y_sign": -1,
                "forward_linear_sign": 1,
                "threshold_x_px": 4,
                "undershoot_y_px": 3.0,
                "overshoot_y_px": 0.5,
                "threshold_angle_deg": 2.0,
                "block_reverse_linear": True,
            },
        }
    }
    return node


def test_coil_stop_ready_prefers_undershoot_side():
    node = make_node_with_undershoot_config()

    assert node._coil_stop_ready(AlignmentError(x=1.0, y=-2.0, angle_deg=0.5))
    assert not node._coil_stop_ready(AlignmentError(x=1.0, y=2.0, angle_deg=0.5))


def test_coil_stop_ready_can_fall_back_to_symmetric_thresholds():
    node = make_node_with_undershoot_config(enabled=False)

    assert node._coil_stop_ready(AlignmentError(x=1.0, y=2.0, angle_deg=0.5))


def test_apply_undershoot_velocity_policy_blocks_reverse_linear_only():
    node = make_node_with_undershoot_config()

    reverse = node._apply_undershoot_velocity_policy(VelocityCommand(linear_x=-0.01, angular_z=0.02))
    forward = node._apply_undershoot_velocity_policy(VelocityCommand(linear_x=0.01, angular_z=0.02))

    assert reverse.linear_x == 0.0
    assert reverse.angular_z == 0.02
    assert forward.linear_x == 0.01
    assert forward.angular_z == 0.02


def test_pair_loss_recovery_keeps_angular_backoff_but_blocks_linear_reverse():
    node = make_node_with_undershoot_config()
    node._last_cmd = VelocityCommand(linear_x=0.01, angular_z=0.03)

    recovery = node._pair_loss_recovery_cmd()

    assert recovery.linear_x == 0.0
    assert recovery.angular_z == -0.03