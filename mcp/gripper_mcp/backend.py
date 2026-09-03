"""What a gripper backend has to provide.

Backends speak the controller's domain, the command joint's position in radians
(0 = open), and know nothing about millimetres, pydantic or MCP; the service
layer owns that translation. This is what lets the same tool surface sit on the
ROS driver, real or simulated, or on a mock without any of them leaking into the
tool signatures.

The vocabulary is the `gripper_cmd` action's: a position and a max effort in,
`reached_goal` and `stalled` out. A stall on a close is how the driver reports
an object between the fingers, so backends pass it through untouched and never
decide for the caller whether it was wanted.
"""

from dataclasses import dataclass
from typing import Protocol, runtime_checkable


@dataclass(frozen=True)
class BackendState:
    position_rad: float
    force_n: float | None = None


@dataclass(frozen=True)
class BackendMotion:
    final_position_rad: float
    reached_goal: bool
    stalled: bool
    refused: bool = False
    timed_out: bool = False
    detail: str = ""


@dataclass(frozen=True)
class BackendHealth:
    reachable: bool
    controller_active: bool | None = None
    detail: str = ""


@runtime_checkable
class GripperBackend(Protocol):
    name: str

    def read_state(self) -> BackendState: ...

    def move_to(
        self, position_rad: float, max_effort_n: float, timeout_s: float
    ) -> BackendMotion: ...

    def health(self) -> BackendHealth: ...
