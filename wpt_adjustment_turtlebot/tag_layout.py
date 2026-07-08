"""AprilTag ID layout helpers.

Recommended physical layout:
- four coil map uses 16 markers around four WPT coils.
- marker IDs are grouped by coil: 11-14, 21-24, 31-34, 41-44.

Backwards-compatible shelf rule:
- head tag: 100 + shelf_number
- coil tag: 100 + 10 * shelf_number + position_number
  - north/up = 1
  - south/down = 2
  - west/left = 3
  - east/right = 4
"""

from __future__ import annotations

from dataclasses import dataclass

POSITION_TO_NUMBER = {"north": 1, "south": 2, "west": 3, "east": 4}
NUMBER_TO_POSITION = {v: k for k, v in POSITION_TO_NUMBER.items()}
PAIR_TO_POSITIONS = {
    "north_south": ("north", "south"),
    "south_north": ("south", "north"),
    "west_east": ("west", "east"),
    "east_west": ("east", "west"),
}

FLOOR_NODE_TAGS = {
    "A01": 1,
    "B01": 2,
    "C01": 3,
    "D01": 4,
    "D02": 17,
    "A03": 18,
    "B03": 19,
    "C03": 20,
    "D04": 37,
    "A05": 38,
    "B05": 39,
    "C05": 40,
    "D05": 41,
}

STATION_TAGS = {
    "A02": {"north": 5, "east": 6, "south": 7, "west": 8},
    "B02": {"north": 9, "east": 10, "south": 11, "west": 12},
    "C02": {"north": 13, "east": 14, "south": 15, "west": 16},
    "D03": {"north": 21, "east": 22, "south": 23, "west": 24},
    "A04": {"north": 25, "east": 26, "south": 27, "west": 28},
    "B04": {"north": 29, "east": 30, "south": 31, "west": 32},
    "C04": {"north": 33, "east": 34, "south": 35, "west": 36},
}

FOUR_COIL_TAGS = {
    "coil_1": {"north": 11, "south": 12, "west": 13, "east": 14},
    "coil_2": {"north": 21, "south": 22, "west": 23, "east": 24},
    "coil_3": {"north": 31, "south": 32, "west": 33, "east": 34},
    "coil_4": {"north": 41, "south": 42, "west": 43, "east": 44},
}


@dataclass(frozen=True)
class ShelfTagSet:
    shelf: int
    head: int
    north: int
    south: int
    west: int
    east: int


def head_tag_id(shelf: int) -> int:
    return 100 + int(shelf)


def coil_tag_id(shelf: int, position: str) -> int:
    return 100 + int(shelf) * 10 + POSITION_TO_NUMBER[position]


def coil_pair_ids(shelf: int, pair_name: str) -> tuple[int, int]:
    positions = PAIR_TO_POSITIONS[pair_name]
    return coil_tag_id(shelf, positions[0]), coil_tag_id(shelf, positions[1])


def station_tag_id(station_name: str, position: str) -> int:
    return STATION_TAGS[station_name.upper()][position.lower()]


def station_pair_ids(station_name: str, pair_name: str) -> tuple[int, int]:
    positions = PAIR_TO_POSITIONS[pair_name]
    return station_tag_id(station_name, positions[0]), station_tag_id(station_name, positions[1])


def four_coil_tag_id(coil_name: str, position: str) -> int:
    return FOUR_COIL_TAGS[coil_name.lower()][position.lower()]


def four_coil_pair_ids(coil_name: str, pair_name: str) -> tuple[int, int]:
    positions = PAIR_TO_POSITIONS[pair_name]
    return four_coil_tag_id(coil_name, positions[0]), four_coil_tag_id(coil_name, positions[1])


def tag_set_for_shelf(shelf: int) -> ShelfTagSet:
    return ShelfTagSet(
        shelf=int(shelf),
        head=head_tag_id(shelf),
        north=coil_tag_id(shelf, "north"),
        south=coil_tag_id(shelf, "south"),
        west=coil_tag_id(shelf, "west"),
        east=coil_tag_id(shelf, "east"),
    )


def decode_coil_tag(tag_id: int) -> tuple[int, str] | None:
    tag_id = int(tag_id)
    if tag_id < 111:
        return None
    shelf = (tag_id - 100) // 10
    position_number = tag_id % 10
    position = NUMBER_TO_POSITION.get(position_number)
    if shelf <= 0 or position is None:
        return None
    return shelf, position


def decode_station_tag(tag_id: int) -> tuple[str, str] | None:
    tag_id = int(tag_id)
    for station_name, positions in STATION_TAGS.items():
        for position, marker_id in positions.items():
            if marker_id == tag_id:
                return station_name, position
    return None


def decode_four_coil_tag(tag_id: int) -> tuple[str, str] | None:
    tag_id = int(tag_id)
    for coil_name, positions in FOUR_COIL_TAGS.items():
        for position, marker_id in positions.items():
            if marker_id == tag_id:
                return coil_name, position
    return None
