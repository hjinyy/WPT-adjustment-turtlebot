# Marker-Centered Line/Kalman Alignment Design

## Goal

Make the TurtleBot stop as accurately as possible at the midpoint between the selected AprilTag marker pair. The black tape line is an auxiliary stabilizer for keeping heading and lateral drift under control while approaching marker-free path sections; it must not become the final alignment authority.

## Operating Assumptions

- Cameras look downward along the z axis.
- For the front camera, the top of the image points toward the desired travel direction and the robot body is toward the opposite side.
- Black electrical tape may exist only on marker-free path sections.
- Missing line observations are non-fatal. The robot should continue using AprilTag observations, Kalman prediction, and the existing safe search behavior.
- The default physical tag layout remains `four_coil_map` with marker IDs 11-44.

## Architecture

Add a line detector that extracts a dark tape line from configurable camera ROIs and reports lateral error, heading error, and confidence. Add a small Kalman-style estimator for image-space alignment errors so AprilTag and line observations can be smoothed without introducing heavy dependencies. Keep the final stop condition tied to AprilTag pair midpoint and angle errors over stable frames.

## Control Priority

1. AprilTag pair visible: use pair midpoint and pair angle as the primary command source. Blend in a small line correction only when line confidence is sufficient.
2. AprilTag pair absent and line visible: follow the line at a conservative search speed.
3. AprilTag pair absent and line absent: fall back to the current `SEARCH_COIL` burst/pause behavior.
4. Final stop: only transition to `FINAL_STOP` after the AprilTag pair is inside configured thresholds for the required stable frame count.

## Files

- `wpt_adjustment_turtlebot/line_detection.py`: pure OpenCV line extraction helpers.
- `wpt_adjustment_turtlebot/sensor_fusion.py`: dependency-free Kalman-style error estimator.
- `wpt_adjustment_turtlebot/controller_math.py`: command blending helpers.
- `wpt_adjustment_turtlebot/wpt_alignment_node.py`: integrate line observations and fused commands.
- `scripts/check_camera_alignment.py`: report line diagnostics during camera-only checks.
- `scripts/generate_tag_sheet.py`: use 11-44 four-coil tags, not legacy 111-144 tags.
- `config/wpt_alignment.yaml`: add line/fusion/blending parameters.
- `docs/algorithm.md`: document the marker-centered control priority.

## Testing Strategy

Use pure unit tests for tag generation, line detection on synthetic images, Kalman prediction/update behavior, and command blending. Keep ROS2 and camera hardware out of unit tests. Run the full pytest suite from the local virtual environment before calling the work complete.
