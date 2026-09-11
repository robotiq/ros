"""Translation between the MCP tool surface and the backends.

Backends speak the command joint's angle; tools speak millimetres. Everything in
between, unit conversion, outcome classification, per-gripper dispatch, lives
here so it can be tested without a FastMCP server or a ROS graph.

The one piece of domain judgement in this file is `classify`: a close that
stops before the fingers meet is a *successful grasp*, not a failure, and a
close that ends with the fingers together grasped nothing. That semantic used
to live as prose in an agent prompt, where it can be forgotten; here it is in
the type system.

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
                robot_name=name,
                model=config.model,
                backend=self._backends[name].name,
                description=config.description,
            )
            for name, config in self._configs.items()
        ]

    def get_state(self, robot_name: str) -> GripperState:
        backend = self._backend(robot_name)
        geometry = geometry_of(self._spec(robot_name), backend)
        state = backend.read_state()
        opening_mm = knuckle_rad_to_opening_mm(state.position_rad, geometry)

        return GripperState(
            robot_name=robot_name,
            opening_mm=round(opening_mm, MM_DECIMALS),
            opening_fraction=round(
                opening_mm_to_fraction(opening_mm, geometry), FRACTION_DECIMALS
            ),
            knuckle_rad=round(state.position_rad, RAD_DECIMALS),
            force_n=state.force_n,
            backend=backend.name,
            measured_at=datetime.now(timezone.utc),
        )

    def open_fully(self, robot_name: str) -> GripperMotionResult:
        stroke = self._spec(robot_name).stroke
        return self.move_to_opening(robot_name, stroke.max_opening_mm)

    def close_fully(self, robot_name: str) -> GripperMotionResult:
        stroke = self._spec(robot_name).stroke
        return self.move_to_opening(robot_name, stroke.min_opening_mm)

    def grasp(
        self, robot_name: str, max_effort_n: float | None = None
    ) -> GripperMotionResult:
        stroke = self._spec(robot_name).stroke
        return self.move_to_opening(
            robot_name, stroke.min_opening_mm, max_effort_n, is_grasp=True
        )

    def move_to_opening(
        self,
        robot_name: str,
        opening_mm: float,
        max_effort_n: float | None = None,
        is_grasp: bool = False,
    ) -> GripperMotionResult:
        backend = self._backend(robot_name)
        spec = self._spec(robot_name)
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
            robot_name=robot_name,
            commanded_opening_mm=round(target_mm, MM_DECIMALS),
            achieved_opening_mm=(
                None if motion.refused else round(achieved_mm, MM_DECIMALS)
            ),
            reached_goal=motion.reached_goal,
            stalled=motion.stalled,
            object_grasped=stopped_on_object if is_grasp else None,
            outcome=classify(motion, target_mm, achieved_mm, spec.stroke, is_grasp),
            detail=clamp_note(opening_mm, target_mm) + motion.detail,
            backend=backend.name,
        )

    def get_health(self, robot_name: str) -> GripperHealth:
        backend = self._backend(robot_name)
        health = backend.health()

        return GripperHealth(
            robot_name=robot_name,
            reachable=health.reachable,
            controller_active=health.controller_active,
            detail=health.detail,
            backend=backend.name,
        )

    def assert_known(self, robot_name: str) -> None:
        if robot_name not in self._configs:
            available = ", ".join(self._configs) or "(none)"
            raise UnknownGripperError(
                f"Unknown gripper '{robot_name}'. Available: {available}"
            )

    def _backend(self, robot_name: str) -> GripperBackend:
        self.assert_known(robot_name)
        return self._backends[robot_name]

    def _spec(self, robot_name: str) -> GripperModelSpec:
        self.assert_known(robot_name)
        return self._specs[self._configs[robot_name].model]


def geometry_of(spec: GripperModelSpec, backend: GripperBackend) -> GripperGeometry:
    return GripperGeometry.of(spec.stroke, backend.joint_geometry())


def clamp_note(requested_mm: float, target_mm: float) -> str:
    if requested_mm == target_mm:
        return ""
    return f"Requested {requested_mm:.1f} mm, clamped to {target_mm:.1f} mm. "


def missed_target(commanded_mm: float, achieved_mm: float, stroke: Stroke) -> bool:
    return abs(achieved_mm - commanded_mm) > stroke.closed_tolerance_mm


def stopped_on_something(
    commanded_mm: float, achieved_mm: float, stroke: Stroke
) -> bool:
    return achieved_mm - commanded_mm > stroke.closed_tolerance_mm


def classify(
    motion: BackendMotion,
    commanded_mm: float,
    achieved_mm: float,
    stroke: Stroke,
    is_grasp: bool,
) -> Outcome:
    if motion.refused:
        return "refused"
    if motion.timed_out:
        return "incomplete"
    if is_grasp:
        if stopped_on_something(commanded_mm, achieved_mm, stroke):
            return "grasped"
        return "closed_without_object"
    if missed_target(commanded_mm, achieved_mm, stroke):
        return "stalled_unexpectedly"
    return "reached"
