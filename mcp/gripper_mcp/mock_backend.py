"""Pure-Python mock gripper, the CI default backend.

It exists to exercise the behaviours the service layer has to get right and a
unit test cannot reach through the real driver: a close that stalls on an
object, travel time, and a goal that outlives its timeout. An optional virtual
object of a given width sits between the fingers; closing past it stalls there.

Travel is simulated through an injected `sleep_fn` so tests run instantly while
a container still moves in something like real time.
"""

import time
from typing import Callable

from gripper_mcp.backend import BackendHealth, BackendMotion, BackendState
from gripper_mcp.units import (
    GripperGeometry,
    clamp,
    knuckle_rad_to_opening_mm,
    opening_mm_to_knuckle_rad,
)

MOCK_GEOMETRY = GripperGeometry(
    max_opening_mm=85.0,
    min_opening_mm=0.0,
    knuckle_rad_open=0.0,
    knuckle_rad_closed=0.8,
)

NOMINAL_TRAVEL_SPEED_MM_S = 150.0


class MockGripperBackend:
    name = "mock"

    def __init__(
        self,
        object_width_mm: float | None = None,
        travel_speed_mm_s: float = NOMINAL_TRAVEL_SPEED_MM_S,
        sleep_fn: Callable[[float], None] = time.sleep,
        geometry: GripperGeometry = MOCK_GEOMETRY,
    ) -> None:
        self._object_width_mm = object_width_mm
        self._travel_speed_mm_s = travel_speed_mm_s
        self._sleep = sleep_fn
        self._geometry = geometry

        self._position_rad = geometry.knuckle_rad_open
        self._holding_force_n = 0.0

    def opening_mm_for(self, position_rad: float) -> float:
        return knuckle_rad_to_opening_mm(position_rad, self._geometry)

    def position_rad_for(self, opening_mm: float) -> float:
        return opening_mm_to_knuckle_rad(opening_mm, self._geometry)

    def read_state(self) -> BackendState:
        return BackendState(
            position_rad=self._position_rad, force_n=self._holding_force_n
        )

    def move_to(
        self, position_rad: float, max_effort_n: float, timeout_s: float
    ) -> BackendMotion:
        target = clamp(
            position_rad,
            self._geometry.knuckle_rad_open,
            self._geometry.knuckle_rad_closed,
        )
        stop_at, stalled = self._resolve_stop(target)

        travel_mm = abs(
            self.opening_mm_for(stop_at) - self.opening_mm_for(self._position_rad)
        )
        duration_s = travel_mm / self._travel_speed_mm_s
        if duration_s > timeout_s:
            self._sleep(timeout_s)
            return BackendMotion(
                final_position_rad=self._position_rad,
                reached_goal=False,
                stalled=False,
                timed_out=True,
                detail=(
                    f"Motion did not complete within {timeout_s:.3f} s "
                    f"(needs {duration_s:.3f} s)."
                ),
            )

        self._sleep(duration_s)
        self._position_rad = stop_at
        self._holding_force_n = max_effort_n if stalled else 0.0

        return BackendMotion(
            final_position_rad=stop_at,
            reached_goal=not stalled,
            stalled=stalled,
            detail=(
                "Stopped early against an object."
                if stalled
                else "Reached the commanded position."
            ),
        )

    def health(self) -> BackendHealth:
        opening_mm = self.opening_mm_for(self._position_rad)
        return BackendHealth(
            reachable=True,
            controller_active=True,
            detail=f"Mock gripper at {opening_mm:.1f} mm.",
        )

    def _resolve_stop(self, target: float) -> tuple[float, bool]:
        closing = target > self._position_rad
        if not closing or self._object_width_mm is None:
            return target, False

        obstruction = self.position_rad_for(self._object_width_mm)
        if target <= obstruction:
            return target, False
        return obstruction, True
