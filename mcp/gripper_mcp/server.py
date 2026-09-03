"""MCP server for Robotiq 2F adaptive grippers, on the ROS 2 driver in this repo.

Every tool takes an explicit `gripper_name` and nothing ever fans out to every
configured gripper. The server is assembled by `build_mcp` from a `GripperService`
so tests can register the tools against a test double without touching argv or
a port; `build_service` takes the backend factory for the same reason.
"""

import argparse
from collections.abc import Callable
from inspect import cleandoc
from pathlib import Path

from fastmcp import FastMCP

from gripper_mcp.backend import GripperBackend
from gripper_mcp.config import (
    SPEC_DIR,
    GripperConfig,
    GripperModelSpec,
    load_gripper_configs,
    load_model_specs,
)
from gripper_mcp.models import (
    GripperHealth,
    GripperInfo,
    GripperMotionResult,
    GripperState,
)
from gripper_mcp.service import GripperService

DEFAULT_WIRING = Path("grippers.yaml")
DEFAULT_PORT = 8300

INSTRUCTIONS = (
    "Tools for controlling Robotiq 2F adaptive grippers. Call gripper_list_grippers "
    "to discover available names, and pass one as gripper_name to every other tool. "
    "Openings are in millimetres: 0.0 is fully closed; fully open is the model's max "
    "opening, reported by gripper_list_grippers. Every motion returns an outcome "
    "field that says how it ended; read that rather than inferring from the flags."
)

READ_ONLY = {
    "read_only_hint": True,
    "destructive_hint": False,
    "idempotent_hint": True,
    "open_world_hint": True,
}
OPENS = {
    "read_only_hint": False,
    "destructive_hint": False,
    "idempotent_hint": True,
    "open_world_hint": True,
}
CLOSES = {
    "read_only_hint": False,
    "destructive_hint": True,
    "idempotent_hint": True,
    "open_world_hint": True,
}
GRASPS = {**CLOSES, "idempotent_hint": False}


def describe(text: str) -> str:
    return cleandoc(text).strip()


BackendFactory = Callable[[GripperConfig, GripperModelSpec], GripperBackend]


def build_backend(config: GripperConfig, spec: GripperModelSpec) -> GripperBackend:
    raise NotImplementedError(
        f"Gripper '{config.name}' needs the ROS backend, which is not in this build."
    )


def build_service(
    wiring: Path,
    spec_dir: Path = SPEC_DIR,
    make_backend: BackendFactory = build_backend,
) -> GripperService:
    configs = {cfg.name: cfg for cfg in load_gripper_configs(wiring)}
    specs = load_model_specs(spec_dir, {cfg.model for cfg in configs.values()})
    backends = {
        name: make_backend(cfg, specs[cfg.model]) for name, cfg in configs.items()
    }
    return GripperService(specs=specs, configs=configs, backends=backends)


def build_mcp(service: GripperService) -> FastMCP:
    mcp = FastMCP("robotiq_gripper_mcp", instructions=INSTRUCTIONS)

    @mcp.tool(
        name="gripper_list_grippers",
        annotations={**READ_ONLY, "open_world_hint": False},
        description=describe(
            """
        List all configured grippers.

        Returns each gripper's name, model, backend and fully-open width in
        millimetres. Use the returned names as the gripper_name argument for
        every other tool.
            """
        ),
    )
    def gripper_list_grippers() -> list[GripperInfo]:
        return service.list_grippers()

    @mcp.tool(
        name="gripper_get_state",
        annotations=READ_ONLY,
        description=describe(
            """
        Read the current opening of one gripper.

        Returns the opening in millimetres (0.0 closed, the model's max opening
        fully open), as a fraction, and as the controller's joint angle in
        radians. The millimetre value comes through a linear approximation of the
        finger linkage; trust it to a couple of millimetres, not better.

        Args:
            gripper_name: Name of the gripper (see gripper_list_grippers).
            """
        ),
    )
    def gripper_get_state(gripper_name: str) -> GripperState:
        return service.get_state(gripper_name)

    @mcp.tool(
        name="gripper_open",
        annotations=OPENS,
        description=describe(
            """
        Open one gripper fully, releasing anything it holds.

        Args:
            gripper_name: Name of the gripper (see gripper_list_grippers).
            """
        ),
    )
    def gripper_open(gripper_name: str) -> GripperMotionResult:
        return service.open_fully(gripper_name)

    @mcp.tool(
        name="gripper_close",
        annotations=CLOSES,
        description=describe(
            """
        Close one gripper fully.

        Use gripper_grasp instead when the intent is to pick something up: here
        the fingers stopping short is outcome="stalled_unexpectedly", whereas
        gripper_grasp reports it as outcome="grasped".

        Args:
            gripper_name: Name of the gripper (see gripper_list_grippers).
            """
        ),
    )
    def gripper_close(gripper_name: str) -> GripperMotionResult:
        return service.close_fully(gripper_name)

    @mcp.tool(
        name="gripper_move_to",
        annotations=CLOSES,
        description=describe(
            """
        Move one gripper to a specific position.

        Args:
            gripper_name: Name of the gripper (see gripper_list_grippers).
            position_mm: Target opening in millimetres, 0.0 (closed) up to the
                model's max opening. Values outside that range are clamped.
            """
        ),
    )
    def gripper_move_to(gripper_name: str, position_mm: float) -> GripperMotionResult:
        return service.move_to_opening(gripper_name, position_mm)

    @mcp.tool(
        name="gripper_grasp",
        annotations=GRASPS,
        description=describe(
            """
        Close one gripper onto an object.

        Same motion as gripper_close, different reading of the result: stopping
        on something is outcome="grasped" with object_grasped=true, and the
        fingers meeting with nothing between them is
        outcome="closed_without_object".

        Args:
            gripper_name: Name of the gripper (see gripper_list_grippers).
            max_effort_n: Grip force ceiling in newtons. Omit for the model's
                default. Lower it for fragile objects.
            """
        ),
    )
    def gripper_grasp(
        gripper_name: str, max_effort_n: float | None = None
    ) -> GripperMotionResult:
        return service.grasp(gripper_name, max_effort_n)

    @mcp.tool(
        name="gripper_get_health",
        annotations=READ_ONLY,
        description=describe(
            """
        Check whether one gripper is reachable and its controller is active.

        Call this before concluding that a refused command means broken hardware.

        Args:
            gripper_name: Name of the gripper (see gripper_list_grippers).
            """
        ),
    )
    def gripper_get_health(gripper_name: str) -> GripperHealth:
        return service.get_health(gripper_name)

    return mcp


def main() -> None:
    parser = argparse.ArgumentParser(description="Robotiq gripper MCP server")
    parser.add_argument("--config", type=Path, default=DEFAULT_WIRING, metavar="PATH")
    parser.add_argument("--spec-dir", type=Path, default=SPEC_DIR, metavar="DIR")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    args = parser.parse_args()

    mcp = build_mcp(build_service(args.config, args.spec_dir))
    mcp.run(transport="http", host=args.host, port=args.port)


if __name__ == "__main__":
    main()
