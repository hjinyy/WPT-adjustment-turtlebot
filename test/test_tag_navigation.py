from wpt_adjustment_turtlebot.tag_navigation import (
    NavigationTag,
    estimate_pose_from_tags,
    heading_from_tag_orientation,
    heading_between_shelves,
    rectilinear_shelf_path,
    turn_delta_deg,
)


def test_estimate_pose_uses_visible_four_coil_tags():
    pose = estimate_pose_from_tags(
        [
            NavigationTag(tag_id=13, camera_name="left_bottom", area_px=20),
            NavigationTag(tag_id=14, camera_name="right_bottom", area_px=25),
        ]
    )

    assert pose is not None
    assert pose.shelf == 1
    assert pose.coil_name == "coil_1"
    assert pose.heading_deg is None
    assert pose.detected_tag_ids == (13, 14)


def test_estimate_heading_from_front_camera_tag_position():
    pose = estimate_pose_from_tags([NavigationTag(tag_id=24, camera_name="front", area_px=50)])

    assert pose is not None
    assert pose.shelf == 2
    assert pose.heading_deg == 180.0
    assert pose.heading_source == "tag_position"


def test_estimate_heading_from_physical_tag_orientation():
    pose = estimate_pose_from_tags([NavigationTag(tag_id=11, camera_name="front", area_px=50, angle_deg=-180.0)])

    assert pose is not None
    assert pose.shelf == 1
    assert pose.heading_deg == 0.0
    assert pose.heading_source == "tag_orientation"


def test_bottom_row_north_tag_orientation_can_point_opposite_top_row():
    top_row = estimate_pose_from_tags([NavigationTag(tag_id=11, camera_name="front", area_px=50, angle_deg=-180.0)])
    bottom_row = estimate_pose_from_tags([NavigationTag(tag_id=32, camera_name="front", area_px=50, angle_deg=0.0)])

    assert top_row is not None
    assert bottom_row is not None
    assert top_row.heading_deg == 0.0
    assert bottom_row.heading_deg == 0.0


def test_heading_from_tag_orientation_uses_camera_x_axis_offset():
    assert heading_from_tag_orientation(
        tag_stage_yaw_deg=90.0,
        observed_image_angle_deg=0.0,
        camera_x_axis_offset_deg=90.0,
    ) == 0.0


def test_heading_between_shelves_matches_stage_layout():
    assert heading_between_shelves(1, 2) == 0.0
    assert heading_between_shelves(1, 3) == 90.0
    assert heading_between_shelves(2, 1) == -180.0


def test_turn_delta_uses_shortest_signed_rotation():
    assert turn_delta_deg(90.0, 0.0) == -90.0
    assert turn_delta_deg(170.0, -170.0) == 20.0


def test_rectilinear_shelf_path_uses_rectangle_edges():
    assert rectilinear_shelf_path(1, 4) == [1, 2, 4]
    assert rectilinear_shelf_path(2, 3) == [2, 1, 3]
    assert rectilinear_shelf_path(1, 2) == [1, 2]
    assert rectilinear_shelf_path(3, 3) == [3]
