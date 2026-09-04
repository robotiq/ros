"""Walk the gripper MCP tool surface against the demo stack (demo/docker-compose.yml).

Two grippers, two stories. `driver` goes through robotiq_gripper_controller on
ros2_control's fake hardware, which teleports to every goal: it shows the ROS
path end to end, and why a grasp there closes on nothing. `bench` is the mock
holding a 40 mm virtual cube with tactile pads: it shows what a grasp, a tactile
read and a contact grasp look like when there is something between the fingers.
"""

import argparse
import asyncio
import json

from fastmcp import Client

DEFAULT_URL = "http://127.0.0.1:8301/mcp"

SHOWN_FIELDS = (
    "outcome",
    "verdict",
    "achieved_opening_mm",
    "opening_mm",
    "contact_signal",
    "peak_pad",
    "steps",
    "reachable",
    "controller_active",
    "detail",
)

STORY = (
    (
        "The ROS path: robotiq_gripper_controller on fake hardware",
        "driver",
        (
            ("Is the controller there?", "gripper_get_health", {}),
            ("Open fully", "gripper_open", {}),
            ("Go to 30 mm", "gripper_set_opening", {"opening_mm": 30.0}),
            ("Read it back", "gripper_get_state", {}),
            (
                "Grasp: fake hardware never stalls, nothing is caught",
                "gripper_grasp",
                {},
            ),
        ),
    ),
    (
        "The mock: a 40 mm cube between TSF-85 pads",
        "bench",
        (
            ("Open fully", "gripper_open", {}),
            ("Zero the pads with the fingers empty", "gripper_tare_tactile", {}),
            ("Grasp: the fingers stop on the cube", "gripper_grasp", {}),
            ("What do the pads feel?", "gripper_read_tactile", {}),
            ("Is it really held?", "gripper_verify_grasp", {}),
            ("Open again", "gripper_open", {}),
            ("Close gently until first touch", "gripper_grasp_until_contact", {}),
        ),
    ),
)


def shown(result: dict) -> str:
    picked = {k: result[k] for k in SHOWN_FIELDS if k in result}
    return json.dumps(picked, ensure_ascii=False)


async def run(url: str) -> None:
    async with Client(url) as client:
        grippers = (await client.call_tool("gripper_list_grippers")).structured_content
        print("grippers:", [g["name"] for g in grippers["result"]])
        for title, robot_name, steps in STORY:
            print(f"\n== {title} ({robot_name})")
            for step_title, tool, extra in steps:
                result = await client.call_tool(
                    tool, {"robot_name": robot_name, **extra}
                )
                print(f"-- {step_title}\n   {tool}: {shown(result.structured_content)}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--url", default=DEFAULT_URL)
    asyncio.run(run(parser.parse_args().url))


if __name__ == "__main__":
    main()
