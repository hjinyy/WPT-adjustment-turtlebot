"""AprilTag ID layout helpers.

Default rule:
- head tag: 100 + shelf_number
- coil tag: shelf_number * 10 + position_number
  - north/up = 1
  - south/down = 2
  - west/left = 3
  - east/right = 4
"""

from dataclasses import dataclass

POSITION_TO_NUMBER = {"north": 1, "south": 2, "west": 3, "east": 4}
NUMBER_TO_POSITION = {v: k for k, v in POSITION_TO_NUMBER.items()}


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
    return int(shelf) * 10 + POSITION_TO_NUMBER[position]


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
    shelf = tag_id // 10
    position_number = tag_id % 10
    position = NUMBER_TO_POSITION.get(position_number)
    if shelf <= 0 or position is None:
        return None
    return shelf, position
