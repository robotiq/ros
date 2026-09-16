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

DEFAULT_WIRING = Path("grippers.yaml")
DEFAULT_PORT = 8300

INSTRUCTIONS = (
    "Tools for controlling Robotiq 2F adaptive grippers. Call gripper_list_grippers "
    "to discover available names, and pass one as gripper_name to every other tool. "
    "Openings are in millimetres: 0.0 is fully closed; fully open is the model's max "
    "opening, reported by gripper_list_grippers. Every motion returns an outcome "
    "field that says how it ended; read that rather than inferring from the flags. "
    "stopped_on_object means the fingers met something; whether that is a grasp "
    "is for the caller to judge. "
    "Grippers listed with a tactile source have TSF-85 pads: gripper_verify_grasp "
    "confirms a hold from touch, gripper_read_tactile reads the pads, "
    "gripper_tare_tactile re-zeroes them."
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


BackendFactory = Callable[[GripperConfig, GripperModelSpec], GripperBackend]
SHUTDOWN_HOOKS: list[Callable[[], None]] = []


def build_backend(config: GripperConfig, spec: GripperModelSpec) -> GripperBackend:
    try:
        from gripper_mcp.ros_backend import RosGraph, RosGripperBackend
    except ImportError as error:
        raise RuntimeError(
            f"Gripper '{config.name}' needs a sourced ROS 2 install with rclpy and "
            f"control_msgs: {error}"
        ) from error
    if RosGraph.shutdown not in SHUTDOWN_HOOKS:
        SHUTDOWN_HOOKS.append(RosGraph.shutdown)
    return RosGripperBackend(config.name, config.namespace)


TactileFactory = Callable[[GripperConfig, GripperModelSpec], TactileBackend | None]


def build_tactile(
    config: GripperConfig, spec: GripperModelSpec
) -> TactileBackend | None:
    if config.tactile is None:
        return None
    if spec.tactile is None:
        raise RuntimeError(
            f"Gripper '{config.name}' asks for tactile pads, but model "
            f"'{config.model}' names no tactile_model in its datasheet."
        )
    return build_ros_tactile(config, spec)


def build_ros_tactile(config: GripperConfig, spec: GripperModelSpec) -> TactileBackend:
    try:
        from gripper_mcp.ros_tactile_backend import RosTactileBackend
    except ImportError as error:
        raise RuntimeError(
            f"Gripper '{config.name}' uses the ros tactile source, which needs a "
            f"sourced ROS 2 install with rclpy and robotiq_tsf: {error}"
        ) from error
    return RosTactileBackend(
        config.name,
        config.tactile_namespace or config.namespace,
        spec.tactile.layout,
    )


def build_services(
    wiring: Path,
    spec_dir: Path = SPEC_DIR,
    make_backend: BackendFactory = build_backend,
    make_tactile: TactileFactory = build_tactile,
) -> tuple[GripperService, TactileService]:
    configs = {cfg.name: cfg for cfg in load_gripper_configs(wiring)}
    specs = load_model_specs(spec_dir, {cfg.model for cfg in configs.values()})
    backends = {
        name: make_backend(cfg, specs[cfg.model]) for name, cfg in configs.items()
    }
    tactile_backends = {}
    for name, cfg in configs.items():
        tactile = make_tactile(cfg, specs[cfg.model])
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
        {
            name: (backend, specs[configs[name].model].tactile)
            for name, backend in tactile_backends.items()
        },
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

        Stopping on something before the commanded position is
        outcome="stopped_on_object" with object_detected=true; whether that is
        a grasp or an obstruction is the caller's call.

        Args:
            gripper_name: Name of the gripper (see gripper_list_grippers).
            max_effort_n: Force ceiling in newtons. Omit for the model's
                default. Lower it for fragile objects.
            """
        ),
    )
    def gripper_open(
        gripper_name: str, max_effort_n: float | None = None
    ) -> GripperMotionResult:
        return service.open_fully(gripper_name, max_effort_n)

    @mcp.tool(
        name="gripper_close",
        annotations=CLOSES,
        description=describe(
            """
        Close one gripper fully.

        Stopping on something before the commanded position is
        outcome="stopped_on_object" with object_detected=true; whether that is
        a grasp or an obstruction is the caller's call.
        A full close with nothing between the fingers is outcome="reached".

        Args:
            gripper_name: Name of the gripper (see gripper_list_grippers).
            max_effort_n: Force ceiling in newtons. Omit for the model's
                default. Lower it for fragile objects.
            """
        ),
    )
    def gripper_close(
        gripper_name: str, max_effort_n: float | None = None
    ) -> GripperMotionResult:
        return service.close_fully(gripper_name, max_effort_n)

    @mcp.tool(
        name="gripper_move_to",
        annotations=CLOSES,
        description=describe(
            """
        Move one gripper to a specific position.

        Stopping on something before the commanded position is
        outcome="stopped_on_object" with object_detected=true; whether that is
        a grasp or an obstruction is the caller's call.

        Args:
            gripper_name: Name of the gripper (see gripper_list_grippers).
            position_mm: Target opening in millimetres, 0.0 (closed) up to the
                model's max opening. Values outside that range are clamped.
            max_effort_n: Force ceiling in newtons. Omit for the model's
                default. Lower it for fragile objects.
            """
        ),
    )
    def gripper_move_to(
        gripper_name: str, position_mm: float, max_effort_n: float | None = None
    ) -> GripperMotionResult:
        return service.move_to_opening(gripper_name, position_mm, max_effort_n)

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
            gripper_name: Name of the gripper (see gripper_list_grippers).
            """
        ),
    )
    def gripper_read_tactile(gripper_name: str) -> TactileReadingResult:
        return service.read(gripper_name)

    @mcp.tool(
        name="gripper_tare_tactile",
        annotations=REZEROES,
        description=describe(
            """
        Re-zero the tactile pads of one gripper.

        Averages many distinct frames into a new rest baseline and sets the
        contact threshold from the noise those frames show (never below the
        datasheet floor). Only call it with NOTHING between the fingers: taring
        on a held object makes that object invisible to every later reading.

        Args:
            gripper_name: Name of the gripper (see gripper_list_grippers).
            """
        ),
    )
    def gripper_tare_tactile(gripper_name: str) -> TactileTareResult:
        return service.tare(gripper_name)

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
        Call it after gripper_close, before lifting. A hold made by opening
        into a bore is recognised only weakly, since the pads face away from
        the part; treat "held" after gripper_open as a hint, not a verdict.

        Args:
            gripper_name: Name of the gripper (see gripper_list_grippers).
            """
        ),
    )
    def gripper_verify_grasp(gripper_name: str) -> GraspVerification:
        return service.verify_grasp(gripper_name)


def main() -> None:
    parser = argparse.ArgumentParser(description="Robotiq gripper MCP server")
    parser.add_argument("--config", type=Path, default=DEFAULT_WIRING, metavar="PATH")
    parser.add_argument("--spec-dir", type=Path, default=SPEC_DIR, metavar="DIR")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    args = parser.parse_args()

    mcp = build_mcp(*build_services(args.config, args.spec_dir))
    try:
        mcp.run(transport="http", host=args.host, port=args.port)
    finally:
        for hook in SHUTDOWN_HOOKS:
            hook()


if __name__ == "__main__":
    main()
