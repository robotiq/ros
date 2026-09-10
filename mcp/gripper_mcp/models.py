"""MCP-facing response models.

Every model names its units in the field name or its description, and every
model carries the `backend` that produced it: a value's trustworthiness depends
on which backend it came from, and an agent has no other way to tell. "ros" is
the driver; "mock" only ever comes from the test doubles under tests/.
"""

from datetime import datetime
from typing import Literal

from pydantic import BaseModel, Field

Backend = Literal["ros", "mock"]
TactileSource = Literal["ros", "mock"]

GraspVerdict = Literal["held", "no_contact", "closed_on_nothing"]

ContactOutcome = Literal[
    "contact_detected",
    "closed_without_contact",
    "stalled_before_contact",
    "incomplete",
    "refused",
]

Outcome = Literal[
    "reached",
    "stopped_on_object",
    "incomplete",
    "not_supported",
    "refused",
]


class GripperInfo(BaseModel):
    gripper_name: str
    model: str
    backend: Backend
    max_opening_mm: float = Field(description="Fully open, in millimetres")
    tactile: TactileSource | None = Field(
        default=None, description="Tactile source when the gripper has pads"
    )
    description: str


class GripperState(BaseModel):
    gripper_name: str
    opening_mm: float = Field(description="0.0 = closed, the model's max = fully open")
    opening_fraction: float = Field(description="0.0 = closed, 1.0 = fully open")
    knuckle_rad: float = Field(
        description=(
            "Command joint position, controller convention: 0.0 = open. "
            "Diagnostics only; command openings in opening_mm"
        )
    )
    force_n: float | None = Field(
        default=None, description="Measured grip force in newtons; null when unmeasured"
    )
    backend: Backend
    measured_at: datetime = Field(description="UTC, timezone-aware")


class GripperMotionResult(BaseModel):
    gripper_name: str
    commanded_opening_mm: float
    achieved_opening_mm: float | None = None
    reached_goal: bool
    stalled: bool = Field(description="Stopped early against resistance")
    object_detected: bool = Field(
        description=(
            "The fingers stopped on something before the commanded position, "
            "in either direction"
        )
    )
    outcome: Outcome
    detail: str = Field(description="Verbatim backend message; never invented")
    backend: Backend


class GripperHealth(BaseModel):
    gripper_name: str
    reachable: bool
    controller_active: bool | None = Field(
        default=None, description="robotiq_gripper_controller is loaded and active"
    )
    detail: str
    backend: Backend


class TactileTareResult(BaseModel):
    gripper_name: str
    samples: int = Field(description="Distinct frames averaged into the new baseline")
    rest_counts_mean: float = Field(description="Mean raw count across every taxel")
    noise_floor: float = Field(
        description="Peak contact_signal the rest frames showed against the baseline"
    )
    threshold: float = Field(
        description="Contact threshold from now on: the datasheet floor or "
        "noise_margin x noise_floor, whichever is higher"
    )
    tactile_backend: TactileSource
    measured_at: datetime = Field(description="UTC, timezone-aware")


class TactileReadingResult(BaseModel):
    gripper_name: str
    contact: bool = Field(description="contact_signal is at or above threshold")
    contact_signal: float = Field(
        description="Baseline-subtracted pressure over both pads: 0.0 rest, 1.0 full scale"
    )
    threshold: float = Field(
        description="Signal at or above which contact is declared; set at tare as the "
        "datasheet floor or noise_margin x the measured rest noise, whichever is higher"
    )
    pad_signals: dict[str, float] = Field(
        description="Per pad, as a fraction of that pad's own ceiling (1.0 = saturated)"
    )
    peak_taxel_counts: float = Field(
        description="Largest single-taxel rise, raw counts"
    )
    tactile_backend: TactileSource
    measured_at: datetime = Field(description="UTC, timezone-aware")


class ContactGraspResult(BaseModel):
    gripper_name: str
    outcome: ContactOutcome
    object_detected: bool = Field(description="True only for contact_detected")
    opening_mm: float = Field(description="Where the fingers stopped")
    contact_signal: float = Field(description="Signal at the stop, 1.0 = full scale")
    threshold: float = Field(description="Signal at or above which closing stopped")
    steps: int = Field(description="Closing increments issued before stopping")
    detail: str
    backend: Backend
    tactile_backend: TactileSource


class GraspVerification(BaseModel):
    gripper_name: str
    verdict: GraspVerdict
    object_held: bool = Field(description="True only for verdict held")
    opening_mm: float
    contact_signal: float
    threshold: float
    detail: str
    tactile_backend: TactileSource
