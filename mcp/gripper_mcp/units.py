"""Conversions between the agent-facing opening (mm) and the controller's joint angle.

Two domains are in play and they must never be confused:

- **opening_mm**: what agents and tools speak. 0.0 = closed, the model's max
  opening (85.0 on a 2F-85) = fully open.
- **knuckle_rad**: what `robotiq_gripper_controller` speaks, the position of the
  gripper's command joint. 0.0 = open, the joint's upper limit (0.8 on a 2F-85)
  = closed. INVERTED relative to opening_mm.

The mapping between them is linear. The real linkage is a four-bar, so the
opening this yields is an approximation, good to a couple of millimetres at
mid-stroke and exact at both ends.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class GripperGeometry:
    max_opening_mm: float
    min_opening_mm: float
    knuckle_rad_open: float
    knuckle_rad_closed: float


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
