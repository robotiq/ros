"""Tactile tools over a gripper's TSF-85 pads.

Kept apart from `GripperService` because tactile is optional per gripper and
has its own state, the rest baseline. The one piece of domain judgement here is
`judge_grasp`: contact on the pads with the fingers apart is a hold; fingers
that met hold nothing whatever the pads say (on a TSF-85 the pads touch each
other when fully closed).
"""

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
    peak_taxel,
)
from gripper_mcp.tactile_backend import TactileBackend

VERDICT_DETAIL: dict[GraspVerdict, str] = {
    "held": "The pads register contact and the fingers stopped before meeting.",
    "closed_on_nothing": "The fingers are fully closed; nothing is between them.",
    "no_contact": "The fingers are apart but the pads register no contact.",
}


class TactileUnavailableError(Exception):
    pass


class TactileService:
    def __init__(
        self,
        grippers: GripperService,
        backends: dict[str, TactileBackend],
        specs: dict[str, TactileSpec],
    ) -> None:
        self._grippers = grippers
        self._backends = backends
        self._specs = specs
        self._baselines: dict[str, TactileBaseline] = {}

    def spec_of(self, robot_name: str) -> TactileSpec:
        return self._source(robot_name)[1]

    def tare(self, robot_name: str) -> TactileTareResult:
        tactile, spec = self._source(robot_name)
        baseline = self._capture_baseline(robot_name, tactile, spec)

        return TactileTareResult(
            robot_name=robot_name,
            samples=spec.baseline_samples,
            rest_counts_mean=round(mean_counts(baseline), 2),
            tactile_backend=tactile.name,
            measured_at=timestamp(),
        )

    def read(self, robot_name: str) -> TactileReadingResult:
        tactile, spec = self._source(robot_name)
        baseline = self._baseline(robot_name, tactile, spec)

        return self._reading(robot_name, tactile, baseline, spec)

    def verify_grasp(self, robot_name: str) -> GraspVerification:
        reading = self.read(robot_name)
        verdict = judge_grasp(reading.contact, self._grippers.fingers_met(robot_name))

        return GraspVerification(
            robot_name=robot_name,
            verdict=verdict,
            object_held=verdict == "held",
            opening_mm=self._grippers.get_state(robot_name).opening_mm,
            contact_signal=reading.contact_signal,
            threshold=reading.threshold,
            detail=VERDICT_DETAIL[verdict],
            tactile_backend=reading.tactile_backend,
        )

    def _source(self, robot_name: str) -> tuple[TactileBackend, TactileSpec]:
        self._grippers.require(robot_name)
        if robot_name not in self._backends:
            raise TactileUnavailableError(
                f"Gripper '{robot_name}' has no tactile pads configured."
            )
        return self._backends[robot_name], self._specs[robot_name]

    def _baseline(
        self, robot_name: str, tactile: TactileBackend, spec: TactileSpec
    ) -> TactileBaseline:
        if robot_name in self._baselines:
            return self._baselines[robot_name]
        if not self._grippers.fingers_open(robot_name):
            raise TactileUnavailableError(
                "No tactile baseline yet, and the fingers are not fully open to "
                "capture one. Open the gripper, or call gripper_tare_tactile."
            )
        return self._capture_baseline(robot_name, tactile, spec)

    def _capture_baseline(
        self, robot_name: str, tactile: TactileBackend, spec: TactileSpec
    ) -> TactileBaseline:
        baseline = average_readings(
            [tactile.read_tactile() for _ in range(spec.baseline_samples)]
        )
        self._baselines[robot_name] = baseline
        return baseline

    def _reading(
        self,
        robot_name: str,
        tactile: TactileBackend,
        baseline: TactileBaseline,
        spec: TactileSpec,
    ) -> TactileReadingResult:
        reading = tactile.read_tactile()
        signal = contact_signal(reading, baseline, spec.full_scale_counts)
        per_pad = zip(reading.pads, pad_sums(reading, baseline))

        return TactileReadingResult(
            robot_name=robot_name,
            contact=signal >= spec.contact_threshold,
            contact_signal=round(signal, 5),
            threshold=spec.contact_threshold,
            pad_signals={
                pad.name: round(total / spec.full_scale_counts, 5)
                for pad, total in per_pad
            },
            peak_taxel_counts=round(peak_taxel(reading, baseline), 1),
            tactile_backend=tactile.name,
            measured_at=timestamp(),
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
