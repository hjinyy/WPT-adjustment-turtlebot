import math

import numpy as np
import pytest

from wpt_adjustment_turtlebot.map_localization import estimate_map_pose_from_tag_corners


def test_homography_projects_camera_center_and_top_as_global_pose():
    tag_center = (-0.2265, 0.2245)
    half = 0.015 / 2.0
    world_corners = [
        (tag_center[0] - half, tag_center[1] + half),
        (tag_center[0] + half, tag_center[1] + half),
        (tag_center[0] + half, tag_center[1] - half),
        (tag_center[0] - half, tag_center[1] - half),
    ]
    image_corners = np.array(
        [[1000.0 * (x + 0.4), 1000.0 * (0.3 - y)] for x, y in world_corners],
        dtype=np.float32,
    )

    pose = estimate_map_pose_from_tag_corners(
        {11: image_corners},
        frame_size=(400, 300),
        tag_world_poses={11: tag_center},
        tag_size_m=0.015,
    )

    assert pose is not None
    assert pose.x_m == pytest.approx(-0.2, abs=1e-4)
    assert pose.y_m == pytest.approx(0.15, abs=1e-4)
    assert pose.yaw_rad == pytest.approx(math.pi / 2.0, abs=1e-4)


def test_homography_requires_a_known_tag():
    assert estimate_map_pose_from_tag_corners(
        {99: np.zeros((4, 2), dtype=np.float32)},
        frame_size=(400, 300),
        tag_world_poses={11: (-0.2265, 0.2245)},
        tag_size_m=0.015,
    ) is None
