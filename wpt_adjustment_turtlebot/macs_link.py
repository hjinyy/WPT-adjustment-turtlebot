"""Pure logic for the 10 Hz MACS <-> TurtleBot link (no ROS / no HTTP here).

scripts/macs_bridge.py wires this to ServerClient + a motion backend. Keeping
the state machine and node-geometry helpers here makes them unit-testable.
"""

from __future__ import annotations

from dataclasses import dataclass, field


def parse_node(node_id: str) -> tuple[int, int]:
    """MACS node id ('A01', 'B03', 'AA10') -> (col, row) zero-based.

    Matches the server's node_id_for(): leading letters are a base-26-ish
    column label (A=0, B=1, ... Z=25, AA=26), trailing digits are 1-based row.
    """
    letters = ""
    i = 0
    while i < len(node_id) and node_id[i].isalpha():
        letters += node_id[i].upper()
        i += 1
    digits = node_id[i:]
    if not letters or not digits.isdigit():
        raise ValueError(f"bad node id {node_id!r}")
    col = 0
    for ch in letters:
        col = col * 26 + (ord(ch) - ord("A") + 1)
    col -= 1
    row = int(digits) - 1
    return col, row


def heading_between(from_node: str, to_node: str) -> str | None:
    """Compass heading for one grid step, matching the web map (north = up =
    decreasing row). Returns None if the nodes aren't a single H/V step apart.
    """
    fc, fr = parse_node(from_node)
    tc, tr = parse_node(to_node)
    dc, dr = tc - fc, tr - fr
    if (abs(dc), abs(dr)) == (0, 1):
        return "north" if dr < 0 else "south"
    if (abs(dc), abs(dr)) == (1, 0):
        return "east" if dc > 0 else "west"
    return None


# Robot phases reported to the server (must match schemas.RobotPhase).
IDLE = "Idle"
DRIVING = "Driving"
ALIGNING = "Aligning"
DWELLING = "Dwelling"
CHARGING = "Charging"
STOPPED = "Stopped"
ESTOPPED = "EStopped"
FAULT = "Fault"


@dataclass
class LegPlan:
    """The sequence of single-step legs for one navigate_to path."""

    path: list[str]
    is_workspace_target: bool
    legs: list[tuple[str, str, str]] = field(default_factory=list)  # (from, to, heading)

    @classmethod
    def from_path(cls, path: list[str], is_workspace_target: bool) -> "LegPlan":
        legs: list[tuple[str, str, str]] = []
        for a, b in zip(path, path[1:]):
            h = heading_between(a, b)
            if h is None:
                raise ValueError(f"non-adjacent path step {a}->{b}")
            legs.append((a, b, h))
        return cls(path=list(path), is_workspace_target=is_workspace_target, legs=legs)
