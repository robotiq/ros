"""MCP server for Robotiq 2F adaptive grippers, on the ROS 2 driver in this repo.

Every tool takes an explicit `robot_name` and nothing ever fans out to every
configured gripper. The server is assembled by `build_mcp` from a `GripperService`
so tests can register the tools against a mock without touching argv or a port.
"""

import argparse
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
from gripper_mcp.mock_backend import MockGripperBackend
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
    "to discover available names, and pass one as robot_name to every other tool. "
    "Openings are in millimetres: 0.0 is fully closed; fully open is the model's max "
    "opening (85.0 on a 2F-85, 140.0 on a 2F-140). On a grasp, reached_goal=false "
    "with stalled=true means the fingers stopped on an object: that is SUCCESS, not "
    "a failure. Read the outcome field, it already says which."
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


def describe(text: str) -> str:
    return cleandoc(text).strip()


def build_backend(config: GripperConfig, spec: GripperModelSpec) -> GripperBackend:
    if config.backend == "mock":
        return MockGripperBackend(
            object_width_mm=config.object_width_mm, geometry=spec.geometry
        )
    try:
        from gripper_mcp.ros_backend import RosGripperBackend
    except ImportError as error:
        raise RuntimeError(
            f"Gripper '{config.name}' uses the ros backend, which needs a sourced "
            f"ROS 2 install with rclpy and control_msgs: {error}"
        ) from error
    return RosGripperBackend(config.name, config.namespace, spec.command_joint)


def build_service(wiring: Path, spec_dir: Path = SPEC_DIR) -> GripperService:
    configs = {cfg.name: cfg for cfg in load_gripper_configs(wiring)}
    specs = load_model_specs(spec_dir, {cfg.model for cfg in configs.values()})
    backends = {
        name: build_backend(cfg, specs[cfg.model]) for name, cfg in configs.items()
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

        Returns each gripper's name, model and backend. Use the returned names as
        the robot_name argument for every other tool.
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
            robot_name: Name of the gripper (see gripper_list_grippers).
            """
        ),
    )
    def gripper_get_state(robot_name: str) -> GripperState:
        return service.get_state(robot_name)

    @mcp.tool(
        name="gripper_open",
        annotations=OPENS,
        description=describe(
            """
        Open one gripper fully, releasing anything it holds.

        Args:
            robot_name: Name of the gripper (see gripper_list_grippers).
            """
        ),
    )
    def gripper_open(robot_name: str) -> GripperMotionResult:
        return service.open_fully(robot_name)

    @mcp.tool(
        name="gripper_close",
        annotations=CLOSES,
        description=describe(
            """
        Close one gripper fully.

        Use gripper_grasp instead when the intent is to pick something up: this
        tool reports a stall as an unexpected obstruction, whereas gripper_grasp
        reports it as a successful grasp.

        Args:
            robot_name: Name of the gripper (see gripper_list_grippers).
            """
        ),
    )
    def gripper_close(robot_name: str) -> GripperMotionResult:
        return service.close_fully(robot_name)

    @mcp.tool(
        name="gripper_set_opening",
        annotations=CLOSES,
        description=describe(
            """
        Move one gripper to a specific opening.

        Args:
            robot_name: Name of the gripper (see gripper_list_grippers).
            opening_mm: Target opening in millimetres, 0.0 (closed) up to the
                model's max opening. Values outside that range are clamped.
            """
        ),
    )
    def gripper_set_opening(robot_name: str, opening_mm: float) -> GripperMotionResult:
        return service.move_to_opening(robot_name, opening_mm)

    @mcp.tool(
        name="gripper_grasp",
        annotations=CLOSES,
        description=describe(
            """
        Close one gripper onto an object.

        Same motion as gripper_close, different reading of the result. Stopping
        before the fingers meet is outcome="grasped" with object_grasped=true; a
        Robotiq 2F cannot reach the fully-closed position while holding
        something, so reached_goal=false is the NORMAL outcome of a successful
        grasp. Fingers meeting with nothing between them is
        outcome="closed_without_object".

        Args:
            robot_name: Name of the gripper (see gripper_list_grippers).
            max_effort_n: Grip force ceiling in newtons. Omit for the model's
                default. Lower it for fragile objects.
            """
        ),
    )
    def gripper_grasp(
        robot_name: str, max_effort_n: float | None = None
    ) -> GripperMotionResult:
        return service.grasp(robot_name, max_effort_n)

    @mcp.tool(
        name="gripper_get_health",
        annotations=READ_ONLY,
        description=describe(
            """
        Check whether one gripper is reachable and its controller is active.

        Call this before concluding that a refused command means broken hardware.

        Args:
            robot_name: Name of the gripper (see gripper_list_grippers).
            """
        ),
    )
    def gripper_get_health(robot_name: str) -> GripperHealth:
        return service.get_health(robot_name)

    return mcp


def main() -> None:
    parser = argparse.ArgumentParser(description="Robotiq gripper MCP server")
    parser.add_argument("--config", type=Path, default=DEFAULT_WIRING, metavar="PATH")
    parser.add_argument("--spec-dir", type=Path, default=SPEC_DIR, metavar="DIR")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    args = parser.parse_args()

    mcp = build_mcp(build_service(args.config, args.spec_dir))
    mcp.run(transport="http", host=args.host, port=args.port)


if __name__ == "__main__":
    main()
