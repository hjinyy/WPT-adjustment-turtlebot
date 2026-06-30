from wpt_adjustment_turtlebot.controller_math import (
    TagObservation,
    TargetPoseInImage,
    angle_error_deg,
    compute_alignment_cmd,
    compute_alignment_error,
    is_aligned,
)
from wpt_adjustment_turtlebot.tag_layout import coil_tag_id, decode_coil_tag, head_tag_id


def test_tag_id_rules():
    assert head_tag_id(6) == 106
    assert coil_tag_id(6, "east") == 64
    assert decode_coil_tag(64) == (6, "east")


def test_angle_wraparound():
    assert angle_error_deg(179, -179) == -2
    assert angle_error_deg(-179, 179) == 2


def test_alignment_error_and_threshold():
    obs = TagObservation(tag_id=11, center_x=326, center_y=216, angle_deg=1.0)
    target = TargetPoseInImage(x=320, y=220, angle_deg=0.0)
    err = compute_alignment_error(obs, target)
    assert err.x == 6
    assert err.y == -4
    assert err.angle_deg == 1.0
    assert is_aligned(err, 7, 5, 2.0)


def test_velocity_clamping_and_deadband():
    err = compute_alignment_error(
        TagObservation(tag_id=11, center_x=400, center_y=100, angle_deg=10),
        TargetPoseInImage(x=320, y=220, angle_deg=0),
    )
    cmd = compute_alignment_cmd(
        err,
        k_y_to_linear=-0.001,
        k_x_to_angular=-0.001,
        k_angle_to_angular=-0.002,
        max_linear=0.015,
        max_angular=0.08,
        x_deadband_px=3,
        y_deadband_px=3,
        angle_deadband_deg=1,
    )
    assert cmd.linear_x == 0.015
    assert cmd.angular_z == -0.08
