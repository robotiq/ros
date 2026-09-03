import asyncio
import sys

import pytest
import yaml
from fakes.gripper import MockGripperBackend
from fakes.tactile import MockTactileBackend
from fastmcp import Client

from gripper_mcp.server import build_mcp, build_services

EXPECTED_TOOLS = {
    "gripper_list_grippers",
    "gripper_get_state",
    "gripper_open",
    "gripper_close",
    "gripper_move_to",
    "gripper_get_health",
    "gripper_read_tactile",
    "gripper_tare_tactile",
    "gripper_verify_grasp",
}
CLOSING_TOOLS = {"gripper_close", "gripper_move_to"}
TACTILE_READS = {"gripper_read_tactile", "gripper_verify_grasp"}
CUBE_WIDTH_MM = 40.0


def write_wiring(directory):
    wiring = directory / "grippers.yaml"
    wiring.write_text(
        yaml.safe_dump(
            [
                {
                    "name": "left",
                    "model": "robotiq_2f_85",
                    "namespace": "/left",
                    "tactile": "ros",
                }
            ]
        )
    )
    return wiring


def gripper_holding_a_cube(config, spec):
    return MockGripperBackend(
        object_width_mm=CUBE_WIDTH_MM,
        stroke=spec.stroke,
        sleep_fn=lambda _seconds: None,
    )


def pads_on_the_cube(config, spec, gripper):
    return MockTactileBackend(
        read_opening_mm=lambda: gripper.opening_mm_for(
            gripper.read_state().position_rad
        ),
        object_width_mm=CUBE_WIDTH_MM,
        layout=spec.tactile.layout,
    )


@pytest.fixture
def mcp(tmp_path):
    wiring = write_wiring(tmp_path)
    return build_mcp(
        *build_services(
            wiring, make_backend=gripper_holding_a_cube, make_tactile=pads_on_the_cube
        )
    )


@pytest.fixture
def tools(mcp):
    registered = asyncio.run(mcp.list_tools())
    return {tool.name: tool for tool in registered}


def call(mcp, tool, **arguments):
    async def run():
        async with Client(mcp) as client:
            return (await client.call_tool(tool, arguments)).data

    return asyncio.run(run())


def test_every_expected_tool_is_registered(tools):
    assert set(tools) == EXPECTED_TOOLS


def test_every_tool_but_the_listing_takes_an_explicit_target(tools):
    for name, tool in tools.items():
        if name == "gripper_list_grippers":
            continue
        assert "gripper_name" in tool.parameters["properties"], name


def test_every_tool_documents_itself_without_leading_indent(tools):
    for name, tool in tools.items():
        assert tool.description, name
        assert tool.description == tool.description.strip(), name


def test_tools_that_reduce_the_opening_are_flagged_destructive(tools):
    for name in CLOSING_TOOLS:
        assert tools[name].annotations.destructive_hint is True, name
        assert tools[name].annotations.read_only_hint is False, name


def test_opening_is_not_destructive_and_reads_are_read_only(tools):
    assert tools["gripper_open"].annotations.destructive_hint is False
    assert tools["gripper_get_state"].annotations.read_only_hint is True
    assert tools["gripper_get_health"].annotations.read_only_hint is True
    for name in TACTILE_READS:
        assert tools[name].annotations.read_only_hint is True, name
    assert tools["gripper_tare_tactile"].annotations.read_only_hint is False
    assert tools["gripper_tare_tactile"].annotations.destructive_hint is False


def test_a_close_then_verify_through_the_wire_confirms_the_hold(mcp):
    call(mcp, "gripper_open", gripper_name="left")
    call(mcp, "gripper_tare_tactile", gripper_name="left")

    call(mcp, "gripper_close", gripper_name="left")
    verification = call(mcp, "gripper_verify_grasp", gripper_name="left")

    assert verification.verdict == "held"
    assert verification.tactile_backend == "mock"


def test_tactile_on_a_model_without_pads_fails_at_startup(tmp_path):
    wiring = tmp_path / "grippers.yaml"
    wiring.write_text(
        yaml.safe_dump(
            [
                {
                    "name": "right",
                    "model": "robotiq_2f_140",
                    "namespace": "/right",
                    "tactile": "ros",
                }
            ]
        )
    )

    with pytest.raises(RuntimeError, match="tactile_model"):
        build_services(wiring, make_backend=gripper_holding_a_cube)


def test_every_motion_is_safe_to_repeat(tools):
    assert tools["gripper_close"].annotations.idempotent_hint is True
    assert tools["gripper_open"].annotations.idempotent_hint is True


def test_a_close_through_the_wire_stops_on_the_virtual_cube(mcp):
    result = call(mcp, "gripper_close", gripper_name="left")

    assert result.outcome == "stopped_on_object"
    assert result.object_detected is True
    assert result.achieved_opening_mm == pytest.approx(CUBE_WIDTH_MM)
    assert result.backend == "mock"


def test_the_listing_names_the_configured_gripper(mcp):
    (entry,) = call(mcp, "gripper_list_grippers")

    assert (entry.gripper_name, entry.backend) == ("left", "mock")
    assert entry.max_opening_mm == pytest.approx(85.0)


def test_a_wiring_entry_names_the_missing_ros_install(tmp_path, monkeypatch):
    monkeypatch.setitem(sys.modules, "rclpy", None)

    with pytest.raises(RuntimeError, match="rclpy"):
        build_services(write_wiring(tmp_path))
