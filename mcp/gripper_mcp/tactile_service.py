"""Tactile tools over a gripper's TSF-85 pads.

Kept apart from `GripperService` because tactile is optional per gripper and
has its own state, the rest baseline. The one piece of domain judgement here is
`judge_grasp`: contact on the pads with the fingers apart is a hold; fingers
that met hold nothing whatever the pads say (on a TSF-85 the pads touch each
other when fully closed, and read well above the contact threshold, so position
has to make that call, never the pads).

An internal hold, the fingers pressing outward on the inside of a bore, is
recognised only weakly: the pads face away from the part, and what they read
comes from finger deflection or the bore edge, a few times the noise floor
against an order of magnitude for an external hold. Do not tune the threshold
against it. The gripper's own object detection names that case outright and
will replace the inference once the backend reads it.

Per-pad signals are each pad's fraction of its own ceiling
(`full_scale_counts / pads`), so one saturated pad reads 1.0 on its own line
while `contact_signal` stays normalised across both.

The contact threshold is set at tare time, not copied from the datasheet: the
rest frames the tare averages also show how noisy the pads are, and the
threshold is the datasheet floor or `noise_margin` times that measured noise,
whichever is higher. Real pads are far noisier than the twin the floor was
tuned on, and a threshold under the noise makes an empty open gripper read as
a hold.
"""

import threading
from dataclasses import dataclass

from gripper_mcp.config import TactileSpec
from gripper_mcp.models import (
    GraspVerdict,
    GraspVerification,
    TactileReadingResult,
    TactileTareResult,
)
from gripper_mcp.service import GripperService, timestamp
from gripper_mcp.tactile import (
    TactileBaseline,
    average_readings,
    contact_signal,
    pad_sums,
    peak_rise,
    rest_noise,
)
from gripper_mcp.tactile_backend import TactileBackend, TactileReading

VERDICT_DETAIL: dict[GraspVerdict, str] = {
    "held": "The pads register contact and the fingers stopped before meeting.",
    "closed_on_nothing": "The fingers are fully closed; nothing is between them.",
    "no_contact": "The fingers are apart but the pads register no contact.",
}


class TactileUnavailableError(Exception):
    pass


@dataclass(frozen=True)
class Tare:
    baseline: TactileBaseline
    noise_floor: float
    threshold: float


class TactileService:
    def __init__(
        self,
        grippers: GripperService,
        pads: dict[str, tuple[TactileBackend, TactileSpec]],
    ) -> None:
        self._grippers = grippers
        self._pads = pads
        self._tares: dict[str, Tare] = {}
        self._tare_lock = threading.Lock()

    def tare(self, gripper_name: str) -> TactileTareResult:
        tactile, spec = self._pads_for(gripper_name)
        tare = self._capture_tare(gripper_name, tactile, spec)

        return TactileTareResult(
            gripper_name=gripper_name,
            samples=spec.baseline_samples,
            rest_counts_mean=round(mean_counts(tare.baseline), 2),
            noise_floor=round(tare.noise_floor, 5),
            threshold=round(tare.threshold, 5),
            tactile_backend=tactile.name,
            measured_at=timestamp(),
        )

    def read(self, gripper_name: str) -> TactileReadingResult:
        opening_mm = self._grippers.get_state(gripper_name).opening_mm

        return self._read_at(gripper_name, opening_mm)

    def verify_grasp(self, gripper_name: str) -> GraspVerification:
        opening_mm = self._grippers.get_state(gripper_name).opening_mm
        reading = self._read_at(gripper_name, opening_mm)
        stroke = self._grippers.stroke_of(gripper_name)
        verdict = judge_grasp(reading.contact, stroke.fingers_met(opening_mm))

        return GraspVerification(
            gripper_name=gripper_name,
            verdict=verdict,
            object_held=verdict == "held",
            opening_mm=opening_mm,
            contact_signal=reading.contact_signal,
            threshold=reading.threshold,
            detail=VERDICT_DETAIL[verdict],
            tactile_backend=reading.tactile_backend,
        )

    def _read_at(self, gripper_name: str, opening_mm: float) -> TactileReadingResult:
        tactile, spec = self._pads_for(gripper_name)
        tare = self._tare(gripper_name, tactile, spec, opening_mm)

        return self._reading(gripper_name, tactile, tare, spec)

    def _pads_for(self, gripper_name: str) -> tuple[TactileBackend, TactileSpec]:
        self._grippers.assert_known(gripper_name)
        if gripper_name not in self._pads:
            raise TactileUnavailableError(
                f"Gripper '{gripper_name}' has no tactile pads configured."
            )
        return self._pads[gripper_name]

    def _tare(
        self,
        gripper_name: str,
        tactile: TactileBackend,
        spec: TactileSpec,
        opening_mm: float,
    ) -> Tare:
        if gripper_name in self._tares:
            return self._tares[gripper_name]
        if not self._grippers.stroke_of(gripper_name).fingers_open(opening_mm):
            raise TactileUnavailableError(
                "No tactile baseline yet, and the fingers are not fully open to "
                "capture one. Open the gripper, or call gripper_tare_tactile."
            )
        return self._capture_tare(gripper_name, tactile, spec)

    def _capture_tare(
        self, gripper_name: str, tactile: TactileBackend, spec: TactileSpec
    ) -> Tare:
        with self._tare_lock:
            tare = measure_tare(tactile.sample(spec.baseline_samples), spec)
            self._tares[gripper_name] = tare
        return tare

    def _reading(
        self,
        gripper_name: str,
        tactile: TactileBackend,
        tare: Tare,
        spec: TactileSpec,
    ) -> TactileReadingResult:
        reading = tactile.read_tactile()
        signal = contact_signal(reading, tare.baseline, spec.full_scale_counts)
        per_pad = zip(reading.pads, pad_sums(reading, tare.baseline))
        pad_full_scale = spec.full_scale_counts / len(reading.pads)

        return TactileReadingResult(
            gripper_name=gripper_name,
            contact=signal >= tare.threshold,
            contact_signal=round(signal, 5),
            threshold=round(tare.threshold, 5),
            pad_signals={
                pad.name: round(total / pad_full_scale, 5) for pad, total in per_pad
            },
            peak_taxel_counts=round(peak_rise(reading, tare.baseline), 1),
            tactile_backend=tactile.name,
            measured_at=timestamp(),
        )


def measure_tare(readings: list[TactileReading], spec: TactileSpec) -> Tare:
    baseline = average_readings(readings)
    noise_floor = rest_noise(readings, baseline, spec.full_scale_counts)

    return Tare(
        baseline=baseline,
        noise_floor=noise_floor,
        threshold=max(spec.contact_threshold, spec.noise_margin * noise_floor),
    )


def mean_counts(baseline: TactileBaseline) -> float:
    total = sum(sum(pad) for pad in baseline.pads)
    count = sum(len(pad) for pad in baseline.pads)
    return total / count


def judge_grasp(contact: bool, fingers_met: bool) -> GraspVerdict:
    if fingers_met:
        return "closed_on_nothing"
    if contact:
        return "held"
    return "no_contact"
