"""Conversions between the agent-facing opening (mm) and the controller's joint angle.

Two domains are in play and they must never be confused:

- **opening_mm**: what agents and tools speak. 0.0 = closed, the model's max
  opening (85.0 on a 2F-85) = fully open.
- **knuckle_rad**: what `robotiq_gripper_controller` speaks, the position of the
  gripper's command joint. 0.0 = open, the joint's upper limit (0.8 on a 2F-85)
  = closed. INVERTED relative to opening_mm.

The two ends have two owners. The stroke in mm is the product datasheet's and
ships with the server. The command joint's name and range belong to the robot
description the backend is actually talking to, so the backend supplies them at
runtime and nothing here is copied from a URDF.

The mapping between them is linear. The real linkage is a four-bar, so
mid-stroke openings are approximate; both ends are exact.

This module is a stopgap. The driver already maps the joint angle to register
counts in C++ (robotiq_driver's gripper_scaling.hpp), and the counts-to-mm
conversion belongs with it, in the SDK, so that every consumer shares one
calibration (robotiq/grippers#16). Until that exists the MCP can only reach the
joint angle over ROS, so it carries this small second map.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Stroke:
    max_opening_mm: float
    min_opening_mm: float = 0.0

    def __post_init__(self) -> None:
        if self.max_opening_mm <= self.min_opening_mm:
            raise ValueError("max_opening_mm must exceed min_opening_mm")


@dataclass(frozen=True)
class JointGeometry:
    name: str
    rad_open: float
    rad_closed: float

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("the command joint needs a name")
        if self.rad_closed == self.rad_open:
            raise ValueError("rad_closed must differ from rad_open")


@dataclass(frozen=True)
class GripperGeometry:
    max_opening_mm: float
    min_opening_mm: float
    knuckle_rad_open: float
    knuckle_rad_closed: float

    def __post_init__(self) -> None:
        if self.max_opening_mm <= self.min_opening_mm:
            raise ValueError("max_opening_mm must exceed min_opening_mm")
        if self.knuckle_rad_closed == self.knuckle_rad_open:
            raise ValueError("knuckle_rad_closed must differ from knuckle_rad_open")

    @classmethod
    def of(cls, stroke: Stroke, joint: JointGeometry) -> "GripperGeometry":
        return cls(
            max_opening_mm=stroke.max_opening_mm,
            min_opening_mm=stroke.min_opening_mm,
            knuckle_rad_open=joint.rad_open,
            knuckle_rad_closed=joint.rad_closed,
        )


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def clamp_opening_mm(opening_mm: float, geometry: GripperGeometry) -> float:
    return clamp(opening_mm, geometry.min_opening_mm, geometry.max_opening_mm)


def opening_mm_to_fraction(opening_mm: float, geometry: GripperGeometry) -> float:
    span = geometry.max_opening_mm - geometry.min_opening_mm
    return (clamp_opening_mm(opening_mm, geometry) - geometry.min_opening_mm) / span


def opening_mm_to_knuckle_rad(opening_mm: float, geometry: GripperGeometry) -> float:
    fraction = opening_mm_to_fraction(opening_mm, geometry)
    span = geometry.knuckle_rad_closed - geometry.knuckle_rad_open
    return geometry.knuckle_rad_closed - fraction * span


def knuckle_rad_to_opening_mm(knuckle_rad: float, geometry: GripperGeometry) -> float:
    span = geometry.knuckle_rad_closed - geometry.knuckle_rad_open
    fraction = (geometry.knuckle_rad_closed - knuckle_rad) / span
    opening_span = geometry.max_opening_mm - geometry.min_opening_mm
    return geometry.min_opening_mm + clamp(fraction, 0.0, 1.0) * opening_span
