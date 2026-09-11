"""Translation between the MCP tool surface and the backends.

Backends speak the command joint's angle; tools speak millimetres. Everything in
between, unit conversion, outcome classification, per-gripper dispatch, lives
here so it can be tested without a FastMCP server or a ROS graph.

`classify` does not decide whether a stop was wanted. Every motion reports
`stopped_on_object` neutrally when the fingers halted short of the commanded
position, in either direction, and the caller, who knows the intent, reads it
as a grasp or an obstruction. The direction is visible from the commanded and
achieved openings. The gripper's own firmware answers the same question with
one enum (moving, detected while opening, detected while closing, at the
requested position), which is what this vocabulary mirrors.

The verdict is read from where the fingers ended up against where they were
sent, never from the controller's `stalled` flag. On the real driver that flag
is set on every goal (robotiq/ros#29: no velocity is ever computed, so stall
detection trips at once), so it is reported to the caller as-is but does not
steer the outcome. This is a stopgap in its own right: the gripper's firmware
reports object detection outright (gOBJ, the `object_status` state interface)
and once a broadcaster carries it the position comparison goes too.
"""

from datetime import datetime, timezone

from gripper_mcp.backend import BackendMotion, GripperBackend
from gripper_mcp.config import GripperConfig, GripperModelSpec
from gripper_mcp.models import (
    GripperHealth,
    GripperInfo,
    GripperMotionResult,
    GripperState,
    Outcome,
)
from gripper_mcp.units import (
    GripperGeometry,
    Stroke,
    clamp_opening_mm,
    knuckle_rad_to_opening_mm,
    opening_mm_to_fraction,
    opening_mm_to_knuckle_rad,
)

MM_DECIMALS = 2
RAD_DECIMALS = 4
FRACTION_DECIMALS = 4


class UnknownGripperError(Exception):
    pass


class GripperService:
    def __init__(
        self,
        specs: dict[str, GripperModelSpec],
        configs: dict[str, GripperConfig],
        backends: dict[str, GripperBackend],
    ) -> None:
        self._specs = specs
        self._configs = configs
        self._backends = backends

    def list_grippers(self) -> list[GripperInfo]:
        return [
            GripperInfo(
                gripper_name=name,
                model=config.model,
                backend=self._backends[name].name,
                max_opening_mm=self._specs[config.model].stroke.max_opening_mm,
                description=config.description,
            )
            for name, config in self._configs.items()
        ]

    def get_state(self, gripper_name: str) -> GripperState:
        backend = self._backend(gripper_name)
        geometry = geometry_of(self._spec(gripper_name), backend)
        state = backend.read_state()
        opening_mm = knuckle_rad_to_opening_mm(state.position_rad, geometry)

        return GripperState(
            gripper_name=gripper_name,
            opening_mm=round(opening_mm, MM_DECIMALS),
            opening_fraction=round(
                opening_mm_to_fraction(opening_mm, geometry), FRACTION_DECIMALS
            ),
            knuckle_rad=round(state.position_rad, RAD_DECIMALS),
            force_n=state.force_n,
            backend=backend.name,
            measured_at=datetime.now(timezone.utc),
        )

    def open_fully(
        self, gripper_name: str, max_effort_n: float | None = None
    ) -> GripperMotionResult:
        stroke = self._spec(gripper_name).stroke
        return self.move_to_opening(gripper_name, stroke.max_opening_mm, max_effort_n)

    def close_fully(
        self, gripper_name: str, max_effort_n: float | None = None
    ) -> GripperMotionResult:
        stroke = self._spec(gripper_name).stroke
        return self.move_to_opening(gripper_name, stroke.min_opening_mm, max_effort_n)

    def move_to_opening(
        self,
        gripper_name: str,
        opening_mm: float,
        max_effort_n: float | None = None,
    ) -> GripperMotionResult:
        backend = self._backend(gripper_name)
        spec = self._spec(gripper_name)
        geometry = geometry_of(spec, backend)
        target_mm = clamp_opening_mm(opening_mm, geometry)
        motion = backend.move_to(
            position_rad=opening_mm_to_knuckle_rad(target_mm, geometry),
            max_effort_n=(
                spec.defaults.max_effort_n if max_effort_n is None else max_effort_n
            ),
            timeout_s=spec.defaults.motion_timeout_s,
        )
        achieved_mm = knuckle_rad_to_opening_mm(motion.final_position_rad, geometry)
        stopped_on_object = stopped_on_something(target_mm, achieved_mm, spec.stroke)

        return GripperMotionResult(
            gripper_name=gripper_name,
            commanded_opening_mm=round(target_mm, MM_DECIMALS),
            achieved_opening_mm=(
                None if motion.refused else round(achieved_mm, MM_DECIMALS)
            ),
            reached_goal=motion.reached_goal,
            stalled=motion.stalled,
            object_detected=stopped_on_object,
            outcome=classify(motion, target_mm, achieved_mm, spec.stroke),
            detail=clamp_note(opening_mm, target_mm) + motion.detail,
            backend=backend.name,
        )

    def get_health(self, gripper_name: str) -> GripperHealth:
        backend = self._backend(gripper_name)
        health = backend.health()

        return GripperHealth(
            gripper_name=gripper_name,
            reachable=health.reachable,
            controller_active=health.controller_active,
            detail=health.detail,
            backend=backend.name,
        )

    def assert_known(self, gripper_name: str) -> None:
        if gripper_name not in self._configs:
            available = ", ".join(self._configs) or "(none)"
            raise UnknownGripperError(
                f"Unknown gripper '{gripper_name}'. Available: {available}"
            )

    def _backend(self, gripper_name: str) -> GripperBackend:
        self.assert_known(gripper_name)
        return self._backends[gripper_name]

    def _spec(self, gripper_name: str) -> GripperModelSpec:
        self.assert_known(gripper_name)
        return self._specs[self._configs[gripper_name].model]


def geometry_of(spec: GripperModelSpec, backend: GripperBackend) -> GripperGeometry:
    return GripperGeometry.of(spec.stroke, backend.joint_geometry())


def clamp_note(requested_mm: float, target_mm: float) -> str:
    if requested_mm == target_mm:
        return ""
    return f"Requested {requested_mm:.1f} mm, clamped to {target_mm:.1f} mm. "


def stopped_on_something(
    commanded_mm: float, achieved_mm: float, stroke: Stroke
) -> bool:
    return abs(achieved_mm - commanded_mm) > stroke.closed_tolerance_mm


def classify(
    motion: BackendMotion,
    commanded_mm: float,
    achieved_mm: float,
    stroke: Stroke,
) -> Outcome:
    if motion.refused:
        return "refused"
    if motion.timed_out:
        return "incomplete"
    if stopped_on_something(commanded_mm, achieved_mm, stroke):
        return "stopped_on_object"
    return "reached"
