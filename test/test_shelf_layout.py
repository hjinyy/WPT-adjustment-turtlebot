from wpt_adjustment_turtlebot.shelf_layout import (
    COIL_SPACING_X_M,
    COIL_SPACING_Y_M,
    Point2D,
    distance_and_heading,
    shelf_position,
)


def test_shelf_positions_form_a_2x2_grid():
    assert shelf_position(1) == Point2D(0.0, 0.0)
    assert shelf_position(2) == Point2D(COIL_SPACING_X_M, 0.0)
    assert shelf_position(3) == Point2D(0.0, COIL_SPACING_Y_M)
    assert shelf_position(4) == Point2D(COIL_SPACING_X_M, COIL_SPACING_Y_M)


def test_distance_and_heading_along_x_axis():
    distance, heading = distance_and_heading(Point2D(0.0, 0.0), Point2D(1.0, 0.0))
    assert distance == 1.0
    assert heading == 0.0


def test_distance_and_heading_along_y_axis():
    distance, heading = distance_and_heading(Point2D(0.0, 0.0), Point2D(0.0, 1.0))
    assert distance == 1.0
    assert heading == 90.0


def test_distance_and_heading_shelf1_to_shelf4():
    distance, heading = distance_and_heading(shelf_position(1), shelf_position(4))
    assert round(distance, 4) == round((COIL_SPACING_X_M**2 + COIL_SPACING_Y_M**2) ** 0.5, 4)
    assert heading > 0
