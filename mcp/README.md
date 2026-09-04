# gripper_mcp

MCP server for Robotiq 2F adaptive grippers. It gives an LLM agent a small,
gripper-shaped tool surface and drives the `ros2_control` driver in
[`grippers/`](../grippers/) underneath, so the same tools work against real
hardware and against a simulator launched with `sim_isaac:=true`.

## Tools

| Tool | What it does |
|---|---|
| `gripper_list_grippers` | Discover configured gripper names |
| `gripper_get_state` | Opening (mm, fraction, joint rad) and grip force |
| `gripper_open` | Open fully |
| `gripper_close` | Close fully; a stall is an unexpected obstruction |
| `gripper_set_opening` | Move to a specific opening in mm |
| `gripper_grasp` | Close onto an object; stopping before the fingers meet is a success |
| `gripper_get_health` | Reachable, controller active |
| `gripper_read_tactile` | TSF-85 pads: contact signal, per-pad split, hottest taxel |
| `gripper_tare_tactile` | Re-zero the pads (fingers empty) |
| `gripper_verify_grasp` | Confirm a hold from touch plus opening |

Every tool takes an explicit `robot_name`; nothing fans out to every gripper.
Openings are in **millimetres**: `0.0` closed, the model's max opening (`85.0`
on a 2F-85, `140.0` on a 2F-140) fully open. Every motion result carries an
`outcome`:

| Outcome | Meaning |
|---|---|
| `reached` | Fingers arrived at the commanded opening |
| `grasped` | A grasp stopped on something before the fingers met (`reached_goal: false`, `stalled: true`; this is success) |
| `closed_without_object` | A grasp closed fully; nothing was between the fingers |
| `stalled_unexpectedly` | A plain move stopped early against resistance |
| `incomplete` | Timed out or never finished |
| `refused` | The backend rejected the goal |

## Tactile

A gripper fitted with TSF-85 fingers gets a `tactile` source in `grippers.yaml`
next to its gripper backend; the three tactile tools fail for any other
gripper. Readings are raw taxel counts, made meaningful by subtracting a
baseline averaged over many samples with the fingers open (the first read takes
it automatically if the gripper is open; `gripper_tare_tactile` retakes it).
`gripper_verify_grasp` gives one of:

| Verdict | Meaning |
|---|---|
| `held` | Pads register contact and the fingers stopped before meeting |
| `closed_on_nothing` | Fingers fully closed |
| `no_contact` | Fingers apart, pads quiet: the object slipped, or the stop was outside the pads |

Only the `mock` tactile source exists so far; the ROS source on
`robotiq_tsf`'s `TactileSensor/StaticData` topic is next.

## Quick start (no hardware)

```bash
cd mcp
cp grippers.yaml.example grippers.yaml   # keep only the `mock` entry
uv run gripper-mcp --config grippers.yaml
```

The server is then at `http://127.0.0.1:8300/mcp` (streamable-http); point any
MCP client at it. The `mock` backend needs no ROS and no gripper: `object_width_mm`
in the wiring puts a virtual object between the fingers so `gripper_grasp` has
something to stop on.

## With the driver

The `ros` backend talks to `robotiq_gripper_controller` through rclpy: it sends
`gripper_cmd` goals and reads `joint_states` under the gripper's `namespace`
from `grippers.yaml`. Source your ROS 2 install before starting the server so
`rclpy` and `control_msgs` import. The action type is whatever the running
controller advertises (`ParallelGripperCommand` from Jazzy's controller,
`GripperCommand` from Humble's), read off the graph rather than guessed from
the distro. A goal the controller aborts on a stall comes back
as `stalled: true`, which the service reads as a grasp: this is what the sim
controller configs (`allow_stalling: false`) produce when the fingers meet an
object.

## Configuration

Two layers, deliberately split:

- `robot_specifications/<model>.yaml`: one datasheet per Robotiq model, the same
  on every host. The command joint's name and range come straight from
  `robotiq_description`'s URDF. Supporting another model is adding a file.
- `grippers.yaml`: the cell's wiring, which grippers exist, their model, backend,
  ROS namespace and optional tactile source. Host-specific, so only
  `grippers.yaml.example` ships.

## Not a ROS package

`mcp/` is a plain Python project managed with [uv](https://docs.astral.sh/uv/),
not an ament package, and `COLCON_IGNORE` keeps it out of every colcon build:

- its dependencies (fastmcp, pydantic v2) have no rosdep keys, so a `package.xml`
  could not declare them;
- fastmcp needs Python 3.12, which Humble (Python 3.10) does not ship.

It talks to the driver over ROS 2 topics and actions at runtime, so it needs a
sourced ROS 2 install with `rclpy` on the machine that runs it, but nothing in
`grippers/` or `robotiq_tsf/` depends on it.

## Development

```bash
cd mcp
uv sync            # creates .venv with the dev dependencies
uv run pytest
```

Formatting and linting run through the repo-wide pre-commit config
(`pre-commit run -a` from the repo root). CI for this directory is
[`ci-mcp.yml`](../.github/workflows/ci-mcp.yml).
