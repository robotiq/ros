import asyncio
import sys

import pytest
import yaml
from fastmcp import Client

from gripper_mcp.server import build_mcp, build_service

EXPECTED_TOOLS = {
    "gripper_list_grippers",
    "gripper_get_state",
    "gripper_open",
    "gripper_close",
    "gripper_set_opening",
    "gripper_grasp",
    "gripper_get_health",
}
CLOSING_TOOLS = {"gripper_close", "gripper_set_opening", "gripper_grasp"}
CUBE_WIDTH_MM = 40.0


@pytest.fixture(scope="module")
def mcp(tmp_path_factory):
    wiring = tmp_path_factory.mktemp("config") / "grippers.yaml"
    wiring.write_text(
        yaml.safe_dump(
            [
                {
                    "name": "left",
                    "model": "robotiq_2f_85",
                    "backend": "mock",
                    "object_width_mm": CUBE_WIDTH_MM,
                }
            ]
        )
    )
    return build_mcp(build_service(wiring))


@pytest.fixture(scope="module")
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
        assert "robot_name" in tool.parameters["properties"], name


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


def test_a_grasp_through_the_wire_stops_on_the_virtual_cube(mcp):
    result = call(mcp, "gripper_grasp", robot_name="left")

    assert result.outcome == "grasped"
    assert result.object_grasped is True
    assert result.achieved_opening_mm == pytest.approx(CUBE_WIDTH_MM)
    assert result.backend == "mock"


def test_the_listing_names_the_configured_gripper(mcp):
    (entry,) = call(mcp, "gripper_list_grippers")

    assert (entry.name, entry.backend) == ("left", "mock")


def test_a_ros_wiring_entry_names_the_missing_ros_install(tmp_path, monkeypatch):
    monkeypatch.setitem(sys.modules, "rclpy", None)
    wiring = tmp_path / "grippers.yaml"
    wiring.write_text(
        yaml.safe_dump([{"name": "left", "model": "robotiq_2f_85", "backend": "ros"}])
    )

    with pytest.raises(RuntimeError, match="rclpy"):
        build_service(wiring)
