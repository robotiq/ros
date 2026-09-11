"""Baseline subtraction and contact-signal arithmetic over raw taxel counts.

Raw counts carry a large, drifting rest offset, so nothing is meaningful until a
baseline is subtracted. The baseline is the mean of many samples, matching the
tactile SDK's `findBaseline()`, which averages 1000.

Pads are matched to their baseline by name, never by position, so a source that
reports its pads in a different order cannot have one pad's rest subtracted from
the other's counts.

Per-taxel rises clamp at zero. Without that clamp a pad reading below its
baseline, ordinary drift, would subtract from the contact detected on the other
pad, and a firm one-sided grasp could report no contact at all.

`contact_signal` is normalised across both pads: 1.0 means every taxel on both
pads sits at the working ceiling, so a firm one-sided contact reads about half
of what the same force spread over both pads reads. Thresholds against it are
in those units.

`rest_noise` is the largest `contact_signal` any tare sample shows against the
baseline they average to: the noise floor, measured for free at tare time, that
a runtime threshold has to sit above.
"""

from dataclasses import dataclass

from gripper_mcp.tactile_backend import TactilePad, TactileReading


class TactileShapeMismatch(Exception):
    pass


@dataclass(frozen=True)
class TactileBaseline:
    pad_names: tuple[str, ...]
    pads: tuple[tuple[float, ...], ...]

    def rests_for(self, pad_name: str) -> tuple[float, ...]:
        return self.pads[self.pad_names.index(pad_name)]


def average_readings(readings: list[TactileReading]) -> TactileBaseline:
    if not readings:
        raise ValueError("Cannot average an empty list of tactile readings.")

    first = readings[0]
    for reading in readings:
        _require_same_shape(reading, first)

    pad_names = tuple(pad.name for pad in first.pads)

    return TactileBaseline(
        pad_names=pad_names,
        pads=tuple(_average_pad(readings, name) for name in pad_names),
    )


def pad_sums(reading: TactileReading, baseline: TactileBaseline) -> tuple[float, ...]:
    return tuple(sum(rises) for rises in _rises(reading, baseline))


def peak_rise(reading: TactileReading, baseline: TactileBaseline) -> float:
    return max(
        (rise for rises in _rises(reading, baseline) for rise in rises), default=0.0
    )


def contact_signal(
    reading: TactileReading, baseline: TactileBaseline, full_scale_counts: float
) -> float:
    if full_scale_counts <= 0.0:
        raise ValueError(
            f"full_scale_counts must be positive, got {full_scale_counts}."
        )

    return sum(pad_sums(reading, baseline)) / full_scale_counts


def rest_noise(
    readings: list[TactileReading], baseline: TactileBaseline, full_scale_counts: float
) -> float:
    return max(
        contact_signal(reading, baseline, full_scale_counts) for reading in readings
    )


def _average_pad(readings: list[TactileReading], pad_name: str) -> tuple[float, ...]:
    samples = zip(*(_pad(reading, pad_name).taxels for reading in readings))

    return tuple(sum(taxels) / len(readings) for taxels in samples)


def _rises(
    reading: TactileReading, baseline: TactileBaseline
) -> tuple[tuple[float, ...], ...]:
    _require_matching_baseline(reading, baseline)

    return tuple(
        tuple(
            max(0.0, taxel - rest)
            for taxel, rest in zip(pad.taxels, baseline.rests_for(pad.name))
        )
        for pad in reading.pads
    )


def _pad(reading: TactileReading, pad_name: str) -> TactilePad:
    return next(pad for pad in reading.pads if pad.name == pad_name)


def _require_same_shape(reading: TactileReading, other: TactileReading) -> None:
    same = reading.layout == other.layout and _taxel_counts(reading) == _taxel_counts(
        other
    )
    if not same:
        raise TactileShapeMismatch(
            "Tactile readings differ in shape; they cannot be averaged together."
        )


def _require_matching_baseline(
    reading: TactileReading, baseline: TactileBaseline
) -> None:
    covered = dict(zip(baseline.pad_names, (len(rests) for rests in baseline.pads)))
    if _taxel_counts(reading) != covered:
        raise TactileShapeMismatch(
            f"Baseline covers pads {sorted(covered)} but the reading has "
            f"{sorted(_taxel_counts(reading))}; re-tare before reading."
        )


def _taxel_counts(reading: TactileReading) -> dict[str, int]:
    return {pad.name: len(pad.taxels) for pad in reading.pads}
