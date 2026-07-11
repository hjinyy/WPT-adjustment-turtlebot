# Global Map Coil 4 to Coil 3 Design

## Goal

Move a TurtleBot from coil 4 to coil 3 using the fixed AprilTag map as the
primary navigation reference. The robot must wait without moving until the
operator starts the mission, rotate automatically when it begins facing away
from the route, stop safely when localization is unavailable, and finish with
the existing undershoot-first coil alignment.

## Canonical Marker Layout

The physical layout is the user-supplied diagram. Marker orientation in the
map is therefore:

| Coil | Top | Bottom | Left | Right |
| --- | ---: | ---: | ---: | ---: |
| coil_1 | 11 | 12 | 13 | 14 |
| coil_2 | 21 | 22 | 23 | 24 |
| coil_3 | 32 | 31 | 34 | 33 |
| coil_4 | 42 | 41 | 44 | 43 |

`coil_3` and `coil_4` must be corrected from the current reversed mapping
before map localization is enabled. The map coordinate system uses the
existing shelf layout: coil 1 is `(0, 0)`, positive x points from coil 1 to
coil 2, and positive y points from coil 1 to coil 3. Coil 4 to coil 3 is
therefore a straight `-x` route.

## Required Calibration Contract

Map navigation is only armed when these configurable values are valid:

- Camera intrinsics: focal lengths and principal point for the camera used for
  map localization.
- Physical AprilTag side length in metres.
- Rigid transform from that camera to the TurtleBot base frame.
- Metric world poses for every fixed marker, derived from the measured stage
  layout and the canonical marker table above.

The node must expose a dry-run diagnostic that prints the estimated map pose
and rejects navigation if the contract is absent or inconsistent. Placeholder
stage dimensions may remain for tests but must not silently arm real motion.

## Architecture

### Map Localization

AprilTag corners are passed through `solvePnP` with the configured camera
model and tag side length. Each observation produces a candidate robot pose
in the map frame by composing the fixed map-to-tag transform, the inverse
camera-to-tag transform, and the inverse base-to-camera transform.

Candidates with excessive reprojection error or disagreement from the median
pose are discarded. A planar EKF stores `[x, y, yaw]`, predicts from the most
recent velocity command, and updates with the remaining observations. Yaw
residuals are normalized to `[-pi, pi]`.

### Route State Machine

`IDLE` publishes zero velocity and waits for an explicit start request.
`LOCALIZE` obtains a valid EKF update. If no map tag is visible, it performs a
bounded, in-place scan; it never drives forward without a pose fix.

`ROTATE_TO_ROUTE` computes the heading from the current map pose toward a
pre-approach waypoint for coil 3 and rotates in the shortest direction.
`FOLLOW_ROUTE` sends only non-negative linear velocity, controls heading and
cross-track error from map pose, and adds the front-camera line estimate only
as a bounded angular correction. Missing line data does not stop the route.

On reaching the coil 3 pre-approach waypoint, `ALIGN_COIL` reuses the existing
pair-centre AprilTag visual servoing, Kalman filtering, and undershoot-first
final stop. `COMPLETE` publishes zero velocity and reports success.

### Localization Loss

During a route, the EKF may predict briefly when tags disappear. After the
configured loss timeout, linear velocity is set to zero. The node may perform
only a bounded in-place scan to reacquire map tags; scan timeout transitions
to `ERROR` with zero velocity. It must not use unbounded blind line following
as a substitute for map localization.

## Operator Interface

`start_bringup.sh` remains a foreground ROS bringup command. `start_alignment.sh`
starts the alignment node in `IDLE`, waits for Enter, and calls the node's
start service for the fixed `coil_4 -> coil_3` mission. It traps `Ctrl+C` to
publish a zero command and terminate the node.

The node provides `start` and `stop` ROS Trigger services. `stop` immediately
returns to `IDLE` and commands zero velocity. `COMPLETE`, `ERROR`, and process
shutdown also command zero velocity.

## Non-Goals

- No SLAM or dynamic obstacle avoidance is added.
- No time-based `drive_to_shelf.py` command participates in this mission.
- The black tape line does not determine destination identity or replace the
  marker-map pose.

## Test Strategy

- Unit-test marker-map orientation, map transform composition, yaw wrapping,
  EKF outlier rejection, route heading, cross-track control, and loss timeout.
- Unit-test state transitions for IDLE/start/stop, reverse-facing start,
  marker loss, successful coil 3 acquisition, and final zero command.
- Keep existing final-alignment and undershoot tests green.
- Run the full pytest suite and a dry-run ROS diagnostic before real motion.
