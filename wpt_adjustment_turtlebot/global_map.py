"""전역 원점 기준 WPT 코일과 AprilTag 지도."""

from __future__ import annotations


TAG_SIZE_M = 0.015
TAG_RADIUS_M = 0.0975

COIL_CENTERS_M: dict[str, tuple[float, float]] = {
    "coil_1": (-0.2265, 0.1270),
    "coil_2": (0.2265, 0.1270),
    "coil_3": (-0.2265, -0.1270),
    "coil_4": (0.2265, -0.1270),
}

SUFFIX_OFFSETS_M: dict[int, tuple[float, float]] = {
    1: (0.0, TAG_RADIUS_M),   # 북
    2: (0.0, -TAG_RADIUS_M),  # 남
    3: (-TAG_RADIUS_M, 0.0),  # 서
    4: (TAG_RADIUS_M, 0.0),   # 동
}


def build_tag_world_poses() -> dict[int, tuple[float, float]]:
    poses: dict[int, tuple[float, float]] = {}
    for coil_number, center in enumerate(COIL_CENTERS_M.values(), start=1):
        for suffix, offset in SUFFIX_OFFSETS_M.items():
            poses[coil_number * 10 + suffix] = (
                round(center[0] + offset[0], 4),
                round(center[1] + offset[1], 4),
            )
    return poses


def plan_axis_aligned_route(start_coil: str, target_coil: str) -> list[tuple[float, float]]:
    start = COIL_CENTERS_M[start_coil]
    target = COIL_CENTERS_M[target_coil]
    if start[0] == target[0] or start[1] == target[1]:
        return [start, target]
    return [start, (target[0], start[1]), target]


def nearest_coil(x_m: float, y_m: float) -> str:
    return min(
        COIL_CENTERS_M,
        key=lambda name: (COIL_CENTERS_M[name][0] - float(x_m)) ** 2 + (COIL_CENTERS_M[name][1] - float(y_m)) ** 2,
    )


def _direction_suffix(from_point: tuple[float, float], to_point: tuple[float, float]) -> int:
    from_x, from_y = from_point
    to_x, to_y = to_point
    if to_x != from_x:
        return 4 if to_x > from_x else 3
    if to_y != from_y:
        return 1 if to_y > from_y else 2
    raise ValueError("route endpoints must differ")


def expected_route_marker_ids(start_coil: str, target_coil: str) -> tuple[int, int]:
    """Return the departure and final-leg marker ids for an axis-aligned route."""
    if start_coil == target_coil:
        raise ValueError("start and target coils must differ")
    start = COIL_CENTERS_M[start_coil]
    target = COIL_CENTERS_M[target_coil]
    route = plan_axis_aligned_route(start_coil, target_coil)
    departure_suffix = _direction_suffix(route[0], route[1])
    goal_suffix = _direction_suffix(route[-2], route[-1])
    return (
        int(start_coil[-1]) * 10 + departure_suffix,
        int(target_coil[-1]) * 10 + goal_suffix,
    )
