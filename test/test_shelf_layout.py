from wpt_adjustment_turtlebot.shelf_layout import (
    COIL_SPACING_X_M,
    COIL_SPACING_Y_M,
    NODE_TO_SHELF,
    SHELF_TO_NODE,
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


def test_node_shelf_mapping_is_consistent():
    assert NODE_TO_SHELF == {"A02": 1, "B02": 2, "A03": 3, "B03": 4}
    assert SHELF_TO_NODE == {1: "A02", 2: "B02", 3: "A03", 4: "B03"}
    for node, shelf in NODE_TO_SHELF.items():
        assert SHELF_TO_NODE[shelf] == node
