import pytest

from gripper_mcp.ros_tactile_messages import (
    TactileMessageMismatch,
    reading_from_counts,
)
from gripper_mcp.tactile_backend import TactileLayout

LAYOUT = TactileLayout(rows=7, cols=4, pad_names=("left", "right"))
LEFT_COUNTS = [100] * LAYOUT.taxels_per_pad
RIGHT_COUNTS = [200] * LAYOUT.taxels_per_pad
LEFT_DYNAMIC = 3
RIGHT_DYNAMIC = -7


def test_two_full_pads_become_two_named_pads():
    reading = reading_from_counts(
        [LEFT_COUNTS, RIGHT_COUNTS], [LEFT_DYNAMIC, RIGHT_DYNAMIC], LAYOUT
    )

    assert [pad.name for pad in reading.pads] == ["left", "right"]
    assert reading.pads[0].taxels == tuple(LEFT_COUNTS)
    assert reading.pads[1].taxels == tuple(RIGHT_COUNTS)
    assert reading.layout == LAYOUT


def test_each_pad_carries_its_own_dynamic_value():
    reading = reading_from_counts(
        [LEFT_COUNTS, RIGHT_COUNTS], [LEFT_DYNAMIC, RIGHT_DYNAMIC], LAYOUT
    )

    assert (reading.pads[0].dynamic, reading.pads[1].dynamic) == (
        LEFT_DYNAMIC,
        RIGHT_DYNAMIC,
    )


def test_a_missing_dynamic_message_reads_as_zero():
    reading = reading_from_counts([LEFT_COUNTS, RIGHT_COUNTS], [], LAYOUT)

    assert all(pad.dynamic == 0 for pad in reading.pads)


def test_one_pad_is_a_pad_mismatch():
    with pytest.raises(TactileMessageMismatch, match="1 pad"):
        reading_from_counts([LEFT_COUNTS], [], LAYOUT)


def test_a_short_taxel_array_names_the_grid():
    with pytest.raises(TactileMessageMismatch, match="7x4"):
        reading_from_counts([LEFT_COUNTS[:-1], RIGHT_COUNTS], [], LAYOUT)


def test_a_dynamic_count_off_from_the_pads_is_a_mismatch():
    with pytest.raises(TactileMessageMismatch, match="one per pad"):
        reading_from_counts([LEFT_COUNTS, RIGHT_COUNTS], [LEFT_DYNAMIC], LAYOUT)
