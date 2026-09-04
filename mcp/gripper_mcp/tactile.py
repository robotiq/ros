"""Baseline subtraction and contact-signal arithmetic over raw taxel counts.

Raw counts carry a large, drifting rest offset, so nothing is meaningful until a
baseline is subtracted. The baseline is the mean of many samples, matching the
tactile SDK's `findBaseline()`, which averages 1000.

Per-taxel rises clamp at zero. Without that clamp a pad reading below its
baseline, ordinary drift, would subtract from the contact detected on the other
pad, and a firm one-sided grasp could report no contact at all.
"""

from dataclasses import dataclass

from gripper_mcp.tactile_backend import TactileReading


class TactileShapeMismatch(Exception):
    pass


@dataclass(frozen=True)
class TactileBaseline:
    pads: tuple[tuple[float, ...], ...]


def average_readings(readings: list[TactileReading]) -> TactileBaseline:
    if not readings:
        raise TactileShapeMismatch("Cannot average an empty list of tactile readings.")

    first = readings[0]
    for reading in readings:
        _require_same_shape(reading, first)

    return TactileBaseline(
        pads=tuple(
            tuple(
                sum(reading.pads[index].taxels[taxel] for reading in readings)
                / len(readings)
                for taxel in range(len(pad.taxels))
            )
            for index, pad in enumerate(first.pads)
        )
    )


def pad_sums(reading: TactileReading, baseline: TactileBaseline) -> tuple[float, ...]:
    _require_matching_baseline(reading, baseline)

    return tuple(
        sum(max(0.0, taxel - rest) for taxel, rest in zip(pad.taxels, rests))
        for pad, rests in zip(reading.pads, baseline.pads)
    )


def peak_taxel(reading: TactileReading, baseline: TactileBaseline) -> float:
    _require_matching_baseline(reading, baseline)

    return max(
        (
            max(0.0, taxel - rest)
            for pad, rests in zip(reading.pads, baseline.pads)
            for taxel, rest in zip(pad.taxels, rests)
        ),
        default=0.0,
    )


def contact_signal(
    reading: TactileReading, baseline: TactileBaseline, full_scale_counts: float
) -> float:
    if full_scale_counts <= 0.0:
        raise TactileShapeMismatch(
            f"full_scale_counts must be positive, got {full_scale_counts}."
        )

    return sum(pad_sums(reading, baseline)) / full_scale_counts


def _require_same_shape(reading: TactileReading, other: TactileReading) -> None:
    same = reading.layout == other.layout and all(
        len(pad.taxels) == len(reference.taxels)
        for pad, reference in zip(reading.pads, other.pads)
    )
    if not same:
        raise TactileShapeMismatch(
            "Tactile readings differ in shape; they cannot be averaged together."
        )


def _require_matching_baseline(
    reading: TactileReading, baseline: TactileBaseline
) -> None:
    matches = len(reading.pads) == len(baseline.pads) and all(
        len(pad.taxels) == len(rests) for pad, rests in zip(reading.pads, baseline.pads)
    )
    if not matches:
        raise TactileShapeMismatch(
            f"Baseline covers {len(baseline.pads)} pad(s) but the reading has "
            f"{len(reading.pads)}; re-tare before reading."
        )
