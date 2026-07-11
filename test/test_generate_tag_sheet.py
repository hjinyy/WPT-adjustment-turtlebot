from scripts.generate_tag_sheet import build_tag_ids


def test_build_tag_ids_uses_default_four_coil_marker_ids():
    items = build_tag_ids([1, 4])

    assert items == [
        ("coil_1 north", 11),
        ("coil_1 south", 12),
        ("coil_1 west", 13),
        ("coil_1 east", 14),
        ("coil_4 north", 41),
        ("coil_4 south", 42),
        ("coil_4 west", 43),
        ("coil_4 east", 44),
    ]
