# Marker-Centered Line/Kalman Alignment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use test-first implementation for each behavior. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the TurtleBot stop accurately at the midpoint between the target AprilTag pair while using line tracing and Kalman-style filtering only as auxiliary stabilization.

**Architecture:** Add pure modules for line detection and error filtering, then wire them into the existing ROS2 node. Keep the final stop authority based on AprilTag pair midpoint and angle, with an undershoot-first final band to avoid overshoot/backtrack oscillation.

**Tech Stack:** Python 3.11, OpenCV, PyYAML, ROS2 rclpy when running on robot, pytest for local verification.

## Global Constraints

- Missing line observations must not stop the robot.
- AprilTag pair alignment remains the final stop authority.
- The default generated tag IDs must be 11-14, 21-24, 31-34, and 41-44.
- Unit tests must not require ROS2, cameras, or network access.

---

## Tasks

- [ ] Add failing tests for four-coil tag sheet generation.
- [ ] Add failing tests for synthetic black line detection and missing-line behavior.
- [ ] Add failing tests for Kalman prediction/update smoothing.
- [ ] Add failing tests for blending AprilTag and line commands.
- [x] Add failing tests for undershoot-first final stop and reverse linear blocking.
- [ ] Implement tag sheet fix and pure modules.
- [ ] Wire line/fusion into `wpt_alignment_node.py`.
- [ ] Add line diagnostics to `check_camera_alignment.py`.
- [ ] Update configuration and algorithm docs.
- [ ] Run full pytest verification.
