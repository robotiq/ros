import pytest

from gripper_mcp.ros_tactile_messages import (
    stale_frame_message,
    TactileMessageMismatch,
    reading_from_counts,
)
from gripper_mcp.tactile_backend import TactileLayout

LAYOUT = TactileLayout(rows=7, cols=4, pad_names=("left", "right"))
LEFT_COUNTS = [100] * LAYOUT.taxels_per_pad
RIGHT_COUNTS = [200] * LAYOUT.taxels_per_pad


def test_two_full_pads_become_two_named_pads():
    reading = reading_from_counts([LEFT_COUNTS, RIGHT_COUNTS], LAYOUT)

    assert [pad.name for pad in reading.pads] == ["left", "right"]
    assert reading.pads[0].taxels == tuple(LEFT_COUNTS)
    assert reading.pads[1].taxels == tuple(RIGHT_COUNTS)
    assert reading.layout == LAYOUT


def test_one_pad_is_a_pad_mismatch():
    with pytest.raises(TactileMessageMismatch, match="1 pad"):
        reading_from_counts([LEFT_COUNTS], LAYOUT)


def test_a_short_taxel_array_names_the_grid():
    with pytest.raises(TactileMessageMismatch, match="7x4"):
        reading_from_counts([LEFT_COUNTS[:-1], RIGHT_COUNTS], LAYOUT)


def test_a_stale_frame_message_names_the_age_and_the_driver():
    message = stale_frame_message(3.24, 2.0)

    assert "3.2 s old" in message
    assert "stopped publishing" in message
