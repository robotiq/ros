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
from gripper_mcp.mock_tactile_backend import MockTactileBackend
from gripper_mcp.models import (
    GraspVerification,
    GripperHealth,
    GripperInfo,
    GripperMotionResult,
    GripperState,
    TactileReadingResult,
    TactileTareResult,
)
from gripper_mcp.service import GripperService
from gripper_mcp.tactile_backend import TactileBackend
from gripper_mcp.tactile_service import TactileService
from gripper_mcp.units import knuckle_rad_to_opening_mm

DEFAULT_WIRING = Path("grippers.yaml")
DEFAULT_PORT = 8300

INSTRUCTIONS = (
    "Tools for controlling Robotiq 2F adaptive grippers. Call gripper_list_grippers "
    "to discover available names, and pass one as robot_name to every other tool. "
    "Openings are in millimetres: 0.0 is fully closed; fully open is the model's max "
    "opening (85.0 on a 2F-85, 140.0 on a 2F-140). On a grasp, reached_goal=false "
    "with stalled=true means the fingers stopped on an object: that is SUCCESS, not "
    "a failure. Read the outcome field, it already says which. Grippers listed with "
    "a tactile source have TSF-85 pads: gripper_verify_grasp confirms a hold from "
    "touch, gripper_read_tactile reads the pads, gripper_tare_tactile re-zeroes them."
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
REZEROES = {
    "read_only_hint": False,
    "destructive_hint": False,
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


def build_tactile(
    config: GripperConfig, spec: GripperModelSpec, gripper: GripperBackend
) -> TactileBackend | None:
    if config.tactile is None:
        return None
    if spec.tactile is None:
        raise RuntimeError(
            f"Gripper '{config.name}' asks for tactile pads, but model "
            f"'{config.model}' has no tactile block in its datasheet."
        )
    return MockTactileBackend(
        read_opening_mm=lambda: knuckle_rad_to_opening_mm(
            gripper.read_state().position_rad, spec.geometry
        ),
        object_width_mm=config.object_width_mm,
        layout=spec.tactile.layout,
    )


def build_services(
    wiring: Path, spec_dir: Path = SPEC_DIR
) -> tuple[GripperService, TactileService]:
    configs = {cfg.name: cfg for cfg in load_gripper_configs(wiring)}
    specs = load_model_specs(spec_dir, {cfg.model for cfg in configs.values()})
    backends = {
        name: build_backend(cfg, specs[cfg.model]) for name, cfg in configs.items()
    }
    tactile_backends = {}
    for name, cfg in configs.items():
        tactile = build_tactile(cfg, specs[cfg.model], backends[name])
        if tactile is not None:
            tactile_backends[name] = tactile

    grippers = GripperService(
        specs=specs,
        configs=configs,
        backends=backends,
        tactile_sources={name: t.name for name, t in tactile_backends.items()},
    )
    tactile = TactileService(
        grippers,
        tactile_backends,
        {name: specs[configs[name].model].tactile for name in tactile_backends},
    )
    return grippers, tactile


def build_mcp(grippers: GripperService, tactile: TactileService) -> FastMCP:
    mcp = FastMCP("robotiq_gripper_mcp", instructions=INSTRUCTIONS)
    register_gripper_tools(mcp, grippers)
    register_tactile_tools(mcp, tactile)
    return mcp


def register_gripper_tools(mcp: FastMCP, service: GripperService) -> None:
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


def register_tactile_tools(mcp: FastMCP, service: TactileService) -> None:
    @mcp.tool(
        name="gripper_read_tactile",
        annotations=READ_ONLY,
        description=describe(
            """
        Read the tactile pads of one gripper fitted with TSF-85 fingers.

        Returns a contact signal from 0.0 (nothing touching) to 1.0 (both pads
        at full scale), the per-pad split, the hottest single taxel, and whether
        the signal is at or above the contact threshold. The signal is relative
        to a baseline captured with the fingers fully open; the first read takes
        it automatically if the gripper is open, otherwise call
        gripper_tare_tactile first. Fails for grippers without a tactile source.

        Args:
            robot_name: Name of the gripper (see gripper_list_grippers).
            """
        ),
    )
    def gripper_read_tactile(robot_name: str) -> TactileReadingResult:
        return service.read(robot_name)

    @mcp.tool(
        name="gripper_tare_tactile",
        annotations=REZEROES,
        description=describe(
            """
        Re-zero the tactile pads of one gripper.

        Averages many readings into a new rest baseline. Only call it with
        NOTHING between the fingers: taring on a held object makes that object
        invisible to every later reading.

        Args:
            robot_name: Name of the gripper (see gripper_list_grippers).
            """
        ),
    )
    def gripper_tare_tactile(robot_name: str) -> TactileTareResult:
        return service.tare(robot_name)

    @mcp.tool(
        name="gripper_verify_grasp",
        annotations=READ_ONLY,
        description=describe(
            """
        Confirm from touch whether one gripper is holding something.

        Combines the pads and the opening. verdict="held" means the pads
        register contact and the fingers stopped before meeting.
        "closed_on_nothing" means the fingers are fully closed. "no_contact"
        means the fingers are apart but nothing presses on the pads: either the
        object slipped, or the gripper stopped on something outside the pads.
        Call it after gripper_grasp, before lifting.

        Args:
            robot_name: Name of the gripper (see gripper_list_grippers).
            """
        ),
    )
    def gripper_verify_grasp(robot_name: str) -> GraspVerification:
        return service.verify_grasp(robot_name)


def main() -> None:
    parser = argparse.ArgumentParser(description="Robotiq gripper MCP server")
    parser.add_argument("--config", type=Path, default=DEFAULT_WIRING, metavar="PATH")
    parser.add_argument("--spec-dir", type=Path, default=SPEC_DIR, metavar="DIR")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    args = parser.parse_args()

    mcp = build_mcp(*build_services(args.config, args.spec_dir))
    mcp.run(transport="http", host=args.host, port=args.port)


if __name__ == "__main__":
    main()
