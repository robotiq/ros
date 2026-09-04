"""Translation between the MCP tool surface and the backends.

Backends speak the command joint's angle; tools speak millimetres. Everything in
between, unit conversion, outcome classification, per-gripper dispatch, lives
here so it can be tested without a FastMCP server or a ROS graph.

The one piece of domain judgement in this file is `classify`: a close that
stalls before the fingers meet is a *successful grasp*, not a failure, and a
close that stalls with the fingers together grasped nothing. That semantic used
to live as prose in an agent prompt, where it can be forgotten; here it is in
the type system.
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
    clamp_opening_mm,
    knuckle_rad_to_opening_mm,
    opening_mm_to_fraction,
    opening_mm_to_knuckle_rad,
)

OPENING_TOLERANCE_MM = 0.5


class UnknownGripperError(Exception):
    pass


class GripperService:
    def __init__(
        self,
        specs: dict[str, GripperModelSpec],
        configs: dict[str, GripperConfig],
        backends: dict[str, GripperBackend],
        tactile_sources: dict[str, str] | None = None,
    ) -> None:
        self._specs = specs
        self._configs = configs
        self._backends = backends
        self._tactile_sources = tactile_sources or {}

    def list_grippers(self) -> list[GripperInfo]:
        return [
            GripperInfo(
                name=name,
                model=config.model,
                backend=self._backends[name].name,
                tactile=self._tactile_sources.get(name),
                description=config.description,
            )
            for name, config in self._configs.items()
        ]

    def require(self, robot_name: str) -> None:
        self._backend(robot_name)

    def get_state(self, robot_name: str) -> GripperState:
        backend = self._backend(robot_name)
        geometry = self._spec(robot_name).geometry
        state = backend.read_state()
        opening_mm = knuckle_rad_to_opening_mm(state.position_rad, geometry)

        return GripperState(
            robot_name=robot_name,
            opening_mm=round(opening_mm, 2),
            opening_fraction=round(opening_mm_to_fraction(opening_mm, geometry), 4),
            knuckle_rad=round(state.position_rad, 4),
            force_n=state.force_n,
            backend=backend.name,
            measured_at=timestamp(),
        )

    def fingers_met(self, robot_name: str) -> bool:
        geometry = self._spec(robot_name).geometry
        opening_mm = self.get_state(robot_name).opening_mm
        return opening_mm <= geometry.min_opening_mm + OPENING_TOLERANCE_MM

    def fingers_open(self, robot_name: str) -> bool:
        geometry = self._spec(robot_name).geometry
        opening_mm = self.get_state(robot_name).opening_mm
        return opening_mm >= geometry.max_opening_mm - OPENING_TOLERANCE_MM

    def open_fully(self, robot_name: str) -> GripperMotionResult:
        geometry = self._spec(robot_name).geometry
        return self.move_to_opening(robot_name, geometry.max_opening_mm)

    def close_fully(self, robot_name: str) -> GripperMotionResult:
        geometry = self._spec(robot_name).geometry
        return self.move_to_opening(robot_name, geometry.min_opening_mm)

    def grasp(
        self, robot_name: str, max_effort_n: float | None = None
    ) -> GripperMotionResult:
        geometry = self._spec(robot_name).geometry
        return self.move_to_opening(
            robot_name, geometry.min_opening_mm, max_effort_n, is_grasp=True
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
        target_mm = clamp_opening_mm(opening_mm, spec.geometry)
        motion = backend.move_to(
            position_rad=opening_mm_to_knuckle_rad(target_mm, spec.geometry),
            max_effort_n=(
                spec.defaults.max_effort_n if max_effort_n is None else max_effort_n
            ),
            timeout_s=spec.defaults.motion_timeout_s,
        )
        achieved_mm = knuckle_rad_to_opening_mm(
            motion.final_position_rad, spec.geometry
        )
        stopped_on_object = stopped_on_something(motion, achieved_mm, spec.geometry)

        return GripperMotionResult(
            robot_name=robot_name,
            commanded_opening_mm=round(target_mm, 2),
            achieved_opening_mm=None if motion.refused else round(achieved_mm, 2),
            reached_goal=motion.reached_goal,
            stalled=motion.stalled,
            object_grasped=stopped_on_object if is_grasp else None,
            outcome=classify(motion, stopped_on_object, is_grasp),
            detail=motion.detail,
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

    def _backend(self, robot_name: str) -> GripperBackend:
        if robot_name not in self._backends:
            available = ", ".join(self._backends) or "(none)"
            raise UnknownGripperError(
                f"Unknown gripper '{robot_name}'. Available: {available}"
            )
        return self._backends[robot_name]

    def _spec(self, robot_name: str) -> GripperModelSpec:
        return self._specs[self._configs[robot_name].model]


def timestamp() -> str:
    return datetime.now(timezone.utc).isoformat()


def stopped_on_something(
    motion: BackendMotion, achieved_mm: float, geometry: GripperGeometry
) -> bool:
    return (
        motion.stalled and achieved_mm > geometry.min_opening_mm + OPENING_TOLERANCE_MM
    )


def classify(motion: BackendMotion, stopped_on_object: bool, is_grasp: bool) -> Outcome:
    if motion.refused:
        return "refused"
    if motion.timed_out:
        return "incomplete"
    if is_grasp and (motion.stalled or motion.reached_goal):
        return "grasped" if stopped_on_object else "closed_without_object"
    if motion.stalled:
        return "stalled_unexpectedly"
    if motion.reached_goal:
        return "reached"
    return "incomplete"
