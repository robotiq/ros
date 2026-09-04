import pytest

from gripper_mcp.tactile import (
    TactileShapeMismatch,
    average_readings,
    contact_signal,
    pad_sums,
    peak_taxel,
)
from gripper_mcp.tactile_backend import TactileLayout, TactilePad, TactileReading

LAYOUT = TactileLayout(rows=7, cols=4, pad_names=("left", "right"))
TAXELS_PER_PAD = LAYOUT.taxels_per_pad

REST_COUNTS = 100
PRESSED_COUNTS = 300
RISE_COUNTS = PRESSED_COUNTS - REST_COUNTS
FULL_SCALE_COUNTS = float(RISE_COUNTS * TAXELS_PER_PAD * 2)
HOT_TAXEL_COUNTS = 900
DRIFTED_LOW_COUNTS = 40


def reading(left_counts: int, right_counts: int) -> TactileReading:
    return TactileReading(
        pads=(
            TactilePad(name="left", taxels=(left_counts,) * TAXELS_PER_PAD),
            TactilePad(name="right", taxels=(right_counts,) * TAXELS_PER_PAD),
        ),
        layout=LAYOUT,
    )


def with_hot_taxel(base: TactileReading, counts: int) -> TactileReading:
    first = base.pads[0]
    spiked = TactilePad(name=first.name, taxels=(counts,) + first.taxels[1:])
    return TactileReading(pads=(spiked,) + base.pads[1:], layout=base.layout)


def resting_baseline():
    return average_readings([reading(REST_COUNTS, REST_COUNTS)])


def test_the_layout_knows_its_taxel_count():
    assert TAXELS_PER_PAD == LAYOUT.rows * LAYOUT.cols


def test_identical_readings_average_to_themselves():
    baseline = average_readings([reading(REST_COUNTS, REST_COUNTS)] * 3)

    assert baseline.pads == ((float(REST_COUNTS),) * TAXELS_PER_PAD,) * 2


def test_the_baseline_is_the_mean_of_the_samples():
    baseline = average_readings(
        [reading(REST_COUNTS, REST_COUNTS), reading(PRESSED_COUNTS, PRESSED_COUNTS)]
    )

    assert baseline.pads[0][0] == pytest.approx((REST_COUNTS + PRESSED_COUNTS) / 2)


def test_averaging_nothing_is_an_error():
    with pytest.raises(TactileShapeMismatch):
        average_readings([])


def test_averaging_readings_of_different_layouts_is_an_error():
    other = TactileLayout(rows=1, cols=1, pad_names=("left", "right"))
    odd = TactileReading(
        pads=(
            TactilePad(name="left", taxels=(REST_COUNTS,)),
            TactilePad(name="right", taxels=(REST_COUNTS,)),
        ),
        layout=other,
    )

    with pytest.raises(TactileShapeMismatch):
        average_readings([reading(REST_COUNTS, REST_COUNTS), odd])


def test_a_reading_at_baseline_has_no_signal():
    signal = contact_signal(
        reading(REST_COUNTS, REST_COUNTS), resting_baseline(), FULL_SCALE_COUNTS
    )

    assert signal == pytest.approx(0.0)


def test_both_pads_at_full_scale_read_one():
    signal = contact_signal(
        reading(PRESSED_COUNTS, PRESSED_COUNTS), resting_baseline(), FULL_SCALE_COUNTS
    )

    assert signal == pytest.approx(1.0)


def test_one_pressed_pad_reads_half():
    signal = contact_signal(
        reading(PRESSED_COUNTS, REST_COUNTS), resting_baseline(), FULL_SCALE_COUNTS
    )

    assert signal == pytest.approx(0.5)


def test_a_non_positive_full_scale_is_an_error():
    with pytest.raises(TactileShapeMismatch):
        contact_signal(reading(REST_COUNTS, REST_COUNTS), resting_baseline(), 0.0)


def test_a_taxel_below_baseline_clamps_to_zero():
    sums = pad_sums(reading(DRIFTED_LOW_COUNTS, REST_COUNTS), resting_baseline())

    assert sums == (0.0, 0.0)


def test_a_pad_drifted_low_cannot_mask_the_other():
    sums = pad_sums(reading(DRIFTED_LOW_COUNTS, PRESSED_COUNTS), resting_baseline())

    assert sums[1] == pytest.approx(RISE_COUNTS * TAXELS_PER_PAD)


def test_each_pad_reports_its_own_total():
    sums = pad_sums(reading(PRESSED_COUNTS, REST_COUNTS), resting_baseline())

    assert sums == (pytest.approx(RISE_COUNTS * TAXELS_PER_PAD), pytest.approx(0.0))


def test_the_peak_is_the_hottest_taxel_rise():
    spiked = with_hot_taxel(reading(REST_COUNTS, REST_COUNTS), HOT_TAXEL_COUNTS)

    assert peak_taxel(spiked, resting_baseline()) == pytest.approx(
        HOT_TAXEL_COUNTS - REST_COUNTS
    )


def test_a_baseline_of_the_wrong_shape_is_an_error():
    single_pad = TactileReading(
        pads=(TactilePad(name="left", taxels=(REST_COUNTS,) * TAXELS_PER_PAD),),
        layout=LAYOUT,
    )

    with pytest.raises(TactileShapeMismatch, match="re-tare"):
        pad_sums(single_pad, resting_baseline())
