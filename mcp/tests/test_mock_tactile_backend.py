from gripper_mcp.mock_tactile_backend import (
    REST_COUNTS,
    TAXEL_MAX_COUNTS,
    TOUCH_COUNTS,
    TSF_85_LAYOUT,
    MockTactileBackend,
)
from gripper_mcp.tactile_backend import TactileBackend

OBJECT_WIDTH_MM = 40.0
FULLY_OPEN_MM = 85.0
JUST_TOUCHING_MM = 40.0
LIGHTLY_PRESSED_MM = 38.0
FIRMLY_PRESSED_MM = 30.0
CRUSHED_MM = 0.0

CENTRE_INDEX = 3 * TSF_85_LAYOUT.cols + 1
EDGE_INDEX = 0


def backend(opening_mm: float, object_width_mm: float | None) -> MockTactileBackend:
    return MockTactileBackend(
        read_opening_mm=lambda: opening_mm, object_width_mm=object_width_mm
    )


def all_resting(reading) -> bool:
    return all(taxel == REST_COUNTS for pad in reading.pads for taxel in pad.taxels)


def test_the_mock_satisfies_the_tactile_protocol():
    assert isinstance(backend(FULLY_OPEN_MM, OBJECT_WIDTH_MM), TactileBackend)


def test_fingers_wider_than_the_object_leave_every_taxel_at_rest():
    assert all_resting(backend(FULLY_OPEN_MM, OBJECT_WIDTH_MM).read_tactile())


def test_closing_fully_on_nothing_leaves_every_taxel_at_rest():
    assert all_resting(backend(CRUSHED_MM, None).read_tactile())


def test_fingers_exactly_at_the_object_register_a_light_touch():
    reading = backend(JUST_TOUCHING_MM, OBJECT_WIDTH_MM).read_tactile()

    assert reading.pads[0].taxels[CENTRE_INDEX] == REST_COUNTS + TOUCH_COUNTS


def test_closing_past_the_object_raises_the_taxels():
    reading = backend(LIGHTLY_PRESSED_MM, OBJECT_WIDTH_MM).read_tactile()

    assert reading.pads[0].taxels[CENTRE_INDEX] > REST_COUNTS


def test_deeper_penetration_reads_higher():
    light = backend(LIGHTLY_PRESSED_MM, OBJECT_WIDTH_MM).read_tactile()
    firm = backend(FIRMLY_PRESSED_MM, OBJECT_WIDTH_MM).read_tactile()

    assert firm.pads[0].taxels[CENTRE_INDEX] > light.pads[0].taxels[CENTRE_INDEX]


def test_both_pads_agree_on_a_symmetric_object():
    reading = backend(FIRMLY_PRESSED_MM, OBJECT_WIDTH_MM).read_tactile()

    assert reading.pads[0].taxels == reading.pads[1].taxels


def test_the_pad_centre_reads_harder_than_its_edge():
    taxels = backend(FIRMLY_PRESSED_MM, OBJECT_WIDTH_MM).read_tactile().pads[0].taxels

    assert taxels[CENTRE_INDEX] > taxels[EDGE_INDEX]


def test_extreme_penetration_saturates():
    reading = backend(CRUSHED_MM, OBJECT_WIDTH_MM).read_tactile()

    assert max(reading.pads[0].taxels) == TAXEL_MAX_COUNTS


def test_the_reading_has_the_configured_layout():
    reading = backend(FULLY_OPEN_MM, OBJECT_WIDTH_MM).read_tactile()

    assert reading.layout == TSF_85_LAYOUT
    assert [pad.name for pad in reading.pads] == list(TSF_85_LAYOUT.pad_names)
    assert all(len(pad.taxels) == TSF_85_LAYOUT.taxels_per_pad for pad in reading.pads)


def test_the_reading_follows_the_live_opening():
    opening = {"mm": FULLY_OPEN_MM}
    live = MockTactileBackend(
        read_opening_mm=lambda: opening["mm"], object_width_mm=OBJECT_WIDTH_MM
    )
    before = live.read_tactile()

    opening["mm"] = FIRMLY_PRESSED_MM
    after = live.read_tactile()

    assert all_resting(before) and not all_resting(after)
