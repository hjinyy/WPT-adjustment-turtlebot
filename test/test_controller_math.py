from wpt_adjustment_turtlebot.controller_math import (
    TagObservation,
    TargetPoseInImage,
    angle_error_deg,
    compute_alignment_cmd,
    compute_alignment_error,
    compute_pair_alignment_error,
    compute_pair_observation,
    grid_cell,
    is_aligned,
)
from wpt_adjustment_turtlebot.tag_layout import coil_pair_ids, coil_tag_id, decode_coil_tag


def test_tag_id_rules():
    assert coil_tag_id(1, "west") == 13
    assert coil_tag_id(1, "east") == 14
    assert coil_pair_ids(1, "west_east") == (13, 14)
    assert coil_tag_id(4, "east") == 44
    assert decode_coil_tag(44) == (4, "east")


def test_pair_alignment_error_uses_midpoint_and_pair_angle():
    west = TagObservation(tag_id=13, center_x=300, center_y=220, angle_deg=5.0, camera_name="left_bottom")
    east = TagObservation(tag_id=14, center_x=340, center_y=220, angle_deg=-3.0, camera_name="left_bottom")
    pair = compute_pair_observation(west, east)
    target = TargetPoseInImage(x=320, y=220, angle_deg=0.0)
    err = compute_pair_alignment_error(pair, target)

    assert pair.midpoint_x == 320
    assert pair.midpoint_y == 220
    assert pair.pair_angle_deg == 0
    assert pair.camera_name == "left_bottom"
    assert err.x == 0
    assert err.y == 0
    assert err.angle_deg == 0
    assert is_aligned(err, 1, 1, 1)


def test_pair_alignment_angle_error():
    north = TagObservation(tag_id=11, center_x=320, center_y=200, angle_deg=0.0)
    south = TagObservation(tag_id=12, center_x=320, center_y=240, angle_deg=0.0)
    pair = compute_pair_observation(north, south)
    err = compute_pair_alignment_error(pair, TargetPoseInImage(x=320, y=220, angle_deg=90.0))

    assert pair.pair_angle_deg == 90
    assert err.x == 0
    assert err.y == 0
    assert err.angle_deg == 0


def test_grid_cell_center_and_corners():
    assert grid_cell(320, 240, 640, 480) == (2, 2)
    assert grid_cell(0, 0, 640, 480) == (1, 1)
    assert grid_cell(639, 479, 640, 480) == (3, 3)
    assert grid_cell(640, 480, 640, 480) == (3, 3)


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
