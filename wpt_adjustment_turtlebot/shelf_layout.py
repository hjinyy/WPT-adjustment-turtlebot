"""Physical layout of the 2x2 coil stage.

Shelf numbering (matches the coil tag shelf numbers in tag_layout.py, i.e.
tags 11-14/21-24/31-34/41-44):

    1  2
    3  4

Shelf 1 is the origin (0, 0). +x runs from shelf 1 towards shelf 2 (a
COIL_SPACING_X_M step), +y runs from shelf 1 towards shelf 3 (a
COIL_SPACING_Y_M step).
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, degrees, hypot

STAGE_WIDTH_M = 0.80
STAGE_HEIGHT_M = 0.60
# Measured coil-center to coil-center travel distances (2026-07):
# horizontal 1<->2 / 3<->4 = 45.3 cm, vertical 1<->3 / 2<->4 = 25.5 cm.
COIL_SPACING_X_M = 0.453
COIL_SPACING_Y_M = 0.255

SHELF_ROW = {1: 0, 2: 0, 3: 1, 4: 1}
SHELF_COL = {1: 0, 2: 1, 3: 0, 4: 1}

# charging-control server (tserver.local:8000) node id <-> our local shelf number.
# Only these 4 of the server's 8 grid nodes are "Workspace" (coil) nodes.
NODE_TO_SHELF = {"A02": 1, "B02": 2, "A03": 3, "B03": 4}
SHELF_TO_NODE = {shelf: node for node, shelf in NODE_TO_SHELF.items()}


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float


def shelf_position(shelf: int) -> Point2D:
    """Coil center position in meters, relative to shelf 1 at the origin."""
    return Point2D(x=SHELF_COL[shelf] * COIL_SPACING_X_M, y=SHELF_ROW[shelf] * COIL_SPACING_Y_M)


def distance_and_heading(start: Point2D, target: Point2D) -> tuple[float, float]:
    """Straight-line distance (m) and heading (deg, 0 = +x axis, CCW positive) from start to target."""
    dx = target.x - start.x
    dy = target.y - start.y
    return hypot(dx, dy), degrees(atan2(dy, dx))
