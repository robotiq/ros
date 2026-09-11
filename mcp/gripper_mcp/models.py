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
