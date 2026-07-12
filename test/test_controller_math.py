from wpt_adjustment_turtlebot.controller_math import (
    TagObservation,
    TargetPoseInImage,
    alignment_state_for_report,
    angle_error_deg,
    compute_alignment_cmd,
    compute_alignment_error,
    compute_pair_alignment_error,
    compute_pair_observation,
    grid_cell,
    is_aligned,
)
from wpt_adjustment_turtlebot.tag_layout import (
    coil_pair_ids,
    coil_tag_id,
    decode_coil_tag,
    decode_four_coil_tag,
    decode_station_tag,
    four_coil_pair_ids,
    four_coil_tag_id,
    head_tag_id,
    station_pair_ids,
    station_tag_id,
)


def test_tag_id_rules():
    # Legacy shelf/head-tag scheme (backwards-compatible only, not used by the
    # default four_coil_map layout). See tag_layout.py docstring.
    assert head_tag_id(1) == 101
    assert head_tag_id(6) == 106
    assert coil_tag_id(1, "west") == 113
    assert coil_tag_id(1, "east") == 114
    assert coil_pair_ids(1, "west_east") == (113, 114)
    assert coil_tag_id(6, "east") == 164
    assert decode_coil_tag(164) == (6, "east")
    assert decode_coil_tag(64) is None


def test_station_map_id_rules():
    assert station_tag_id("A02", "north") == 5
    assert station_tag_id("A02", "east") == 6
    assert station_tag_id("A02", "south") == 7
    assert station_tag_id("A02", "west") == 8
    assert station_pair_ids("A02", "west_east") == (8, 6)
    assert station_pair_ids("A02", "north_south") == (5, 7)
    assert station_pair_ids("B02", "west_east") == (12, 10)
    assert station_pair_ids("C04", "north_south") == (33, 35)
    assert decode_station_tag(6) == ("A02", "east")
    assert decode_station_tag(113) is None


def test_four_coil_map_id_rules():
    # Default layout_mode. IDs (11-44) match the already-printed physical
    # tags used by the no-ROS 3x3-grid workflow -- no re-printing needed.
    assert four_coil_tag_id("coil_1", "north") == 11
    assert four_coil_tag_id("coil_1", "south") == 12
    assert four_coil_tag_id("coil_1", "west") == 13
    assert four_coil_tag_id("coil_1", "east") == 14
    assert four_coil_pair_ids("coil_1", "west_east") == (13, 14)
    assert four_coil_pair_ids("coil_1", "north_south") == (11, 12)
    assert four_coil_pair_ids("coil_2", "west_east") == (23, 24)
    assert four_coil_pair_ids("coil_3", "north_south") == (32, 31)
    assert four_coil_pair_ids("coil_3", "west_east") == (34, 33)
    assert four_coil_pair_ids("coil_4", "west_east") == (44, 43)
    assert decode_four_coil_tag(44) == ("coil_4", "west")
    assert decode_four_coil_tag(5) is None


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


def test_alignment_state_for_report():
    assert alignment_state_for_report(any_tag_detected=False, all_cameras_aligned=False, locked=False) == "None"
    assert alignment_state_for_report(any_tag_detected=True, all_cameras_aligned=False, locked=False) == "Searching"
    assert alignment_state_for_report(any_tag_detected=True, all_cameras_aligned=True, locked=False) == "Aligned"
    assert alignment_state_for_report(any_tag_detected=True, all_cameras_aligned=True, locked=True) == "Locked"


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
