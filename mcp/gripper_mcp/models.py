"""MCP-facing response models.

Every model names its units in the field name or its description, and every
model carries the `backend` that produced it: a value's trustworthiness depends
on which backend it came from, and an agent has no other way to tell.
"""

from typing import Literal

from pydantic import BaseModel, Field

Backend = Literal["ros", "mock"]

Outcome = Literal[
    "reached",
    "grasped",
    "closed_without_object",
    "stalled_unexpectedly",
    "incomplete",
    "not_supported",
    "refused",
]


class GripperInfo(BaseModel):
    name: str
    model: str
    backend: Backend
    description: str


class GripperState(BaseModel):
    robot_name: str
    opening_mm: float = Field(description="0.0 = closed, the model's max = fully open")
    opening_fraction: float = Field(description="0.0 = closed, 1.0 = fully open")
    knuckle_rad: float = Field(
        description="Command joint position, controller convention: 0.0 = open"
    )
    force_n: float | None = Field(
        default=None, description="Measured grip force in newtons; null when unmeasured"
    )
    backend: Backend
    measured_at: str = Field(description="ISO-8601 UTC")


class GripperMotionResult(BaseModel):
    robot_name: str
    commanded_opening_mm: float
    achieved_opening_mm: float | None = None
    reached_goal: bool
    stalled: bool = Field(description="Stopped early against resistance")
    object_grasped: bool | None = Field(
        default=None,
        description="Set by gripper_grasp: stopped on something before fully closing",
    )
    outcome: Outcome
    detail: str = Field(description="Verbatim backend message; never invented")
    backend: Backend


class GripperHealth(BaseModel):
    robot_name: str
    reachable: bool
    controller_active: bool | None = Field(
        default=None, description="robotiq_gripper_controller is loaded and active"
    )
    detail: str
    backend: Backend
