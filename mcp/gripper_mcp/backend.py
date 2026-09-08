"""What a gripper backend has to provide.

Backends speak the controller's domain, the command joint's position in radians
(0 = open), and know nothing about millimetres, pydantic or MCP; the service
layer owns that translation. The joint itself, its name and its open/closed
angles, is the backend's to report: the ROS backend reads it from the robot
description it is connected to, a test double has a built-in one. This is what
lets the same tool surface sit on the ROS driver, real or simulated, or on a
test double without any of them leaking into the tool signatures.

The vocabulary is the `gripper_cmd` action's: a position and a max effort in,
`reached_goal` and `stalled` out. A stall on a close is how the driver reports
an object between the fingers, so backends pass it through untouched and never
decide for the caller whether it was wanted. `refused` is set when the backend
rejected the goal before any motion, typically the action server turning it
down; `timed_out` when the motion outlived its deadline.

This contract is not MCP-specific: every Python client of the ROS driver wants
"move to a position with a max effort, report reached or stalled". Its home is
a ros client package alongside the ROS backend once that backend lands; it sits
here until then, with the MCP as its first consumer.
"""

from dataclasses import dataclass
from typing import Protocol, runtime_checkable

from gripper_mcp.units import JointGeometry


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

    def joint_geometry(self) -> JointGeometry: ...

    def read_state(self) -> BackendState: ...

    def move_to(
        self, position_rad: float, max_effort_n: float, timeout_s: float
    ) -> BackendMotion: ...

    def health(self) -> BackendHealth: ...
