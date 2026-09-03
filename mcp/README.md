# gripper_mcp

MCP server for Robotiq 2F adaptive grippers. It gives an LLM agent a small,
gripper-shaped tool surface and drives the `ros2_control` driver in
[`grippers/`](../grippers/) underneath, so the same tools work against real
hardware and against a simulator launched with `sim_topic_based:=true`.

## Tools

| Tool | What it does |
|---|---|
| `gripper_list_grippers` | Discover configured gripper names |
| `gripper_get_state` | Opening (mm, fraction, joint rad); the driver reports no force |
| `gripper_open` | Open fully; optional `max_effort_n` |
| `gripper_close` | Close fully; optional `max_effort_n` |
| `gripper_move_to` | Move to a specific position, in mm of opening; optional `max_effort_n` |
| `gripper_get_health` | Reachable, controller active |
| `gripper_read_tactile` | TSF-85 pads: contact signal, per-pad split, hottest taxel |
| `gripper_tare_tactile` | Re-zero the pads (fingers empty) |
| `gripper_verify_grasp` | Confirm a hold from touch plus opening |

Every tool takes an explicit `gripper_name`; nothing fans out to every gripper.
Openings are in **millimetres**: `0.0` closed, the model's max opening (`85.0`
on a 2F-85, `140.0` on a 2F-140) fully open. Every motion result carries an
`outcome`:

| Outcome | Meaning |
|---|---|
| `reached` | Fingers arrived at the commanded opening |
| `stopped_on_object` | The fingers stopped on something before the commanded position, either direction; `object_detected: true`. Whether that is a grasp is the caller's call |
| `incomplete` | Timed out or never finished |
| `refused` | The backend rejected the goal |

## Tactile

A gripper fitted with TSF-85 fingers gets a `tactile` source in `grippers.yaml`
next to its namespace; the three tactile tools fail for any other gripper.
Readings are raw taxel counts, made meaningful by subtracting a baseline
averaged over many samples with the fingers open (the first read takes it
automatically if the gripper is open; `gripper_tare_tactile` retakes it).
The tare also sets the contact threshold: the datasheet floor, or `noise_margin`
times the peak rest signal its own frames showed, whichever is higher, so pads
noisier than the twin the floor was tuned on do not read an empty gripper as a
hold. `gripper_verify_grasp` gives one of:

| Verdict | Meaning |
|---|---|
| `held` | Pads register contact and the fingers stopped before meeting |
| `closed_on_nothing` | Fingers fully closed |
| `no_contact` | Fingers apart, pads quiet: the object slipped, or the stop was outside the pads |

The ROS source on `robotiq_tsf`'s `TactileSensor/StaticData` topic is next;
until it lands, `build_services` refuses a `tactile` entry by name. The tests
drive the tools from a scripted pad model under `tests/fakes/`.

## Quick start

```bash
cd mcp
cp grippers.yaml.example grippers.yaml   # one entry per gripper, with its namespace
uv run gripper-mcp --config grippers.yaml
```

The server is then at `http://127.0.0.1:8300/mcp` (streamable-http); point any
MCP client at it. The tools carry no authentication: anything that can reach
the port can close a gripper. The default binds localhost only; pass
`--host 0.0.0.0` to serve the cell network, and firewall the port when you do.
Without hardware, run the driver on ros2_control's fake hardware
(`use_fake_hardware:=true`) rather than a Python stand-in: that exercises the
driver, the controller and the action, which a mock never will. The unit tests
use a scripted double under `tests/fakes/`.

## With the driver

The backend talks to `robotiq_gripper_controller` through rclpy: it sends
`gripper_cmd` goals and reads `joint_states` under the gripper's `namespace`
from `grippers.yaml`. Source your ROS 2 install before starting the server so
`rclpy` and `control_msgs` import. The action type is whatever the running
controller advertises (`ParallelGripperCommand` from Jazzy's controller,
`GripperCommand` from Humble's), read off the graph rather than guessed from
the distro. The controller's `stalled` flag is passed through in the result but
the outcome is read from where the fingers ended up (see `service.py`), because
the driver sets that flag on every goal (robotiq/ros#29).

### Connecting an agent

[Claude Code](https://code.claude.com/docs/en/mcp), once, then start a new
session:

```bash
claude mcp add --transport http robotiq-gripper http://127.0.0.1:8300/mcp
```

[Codex](https://developers.openai.com/codex/mcp), in `~/.codex/config.toml`
(or `.codex/config.toml` in the project):

```toml
[mcp_servers.robotiq-gripper]
url = "http://127.0.0.1:8300/mcp"
```

Either client then lists the `gripper_*` tools; `/mcp` in the session shows
the connection. Ask in plain language ("open the left gripper to 30 mm") and
the agent picks the tool.

## Configuration

Two layers, deliberately split:

- `gripper_mcp/datasheets/<model>.yaml`: one datasheet per Robotiq model, the same
  on every host. The command joint's name and range come straight from
  `robotiq_description`'s URDF. Supporting another model is adding a file.
- `grippers.yaml`: the cell's wiring, which grippers exist, their model, the
  ROS namespace their driver runs under and an optional tactile source.
  Host-specific, so only
  `grippers.yaml.example` ships.

## Not a ROS package

`mcp/` is a plain Python project managed with [uv](https://docs.astral.sh/uv/),
not an ament package, and `COLCON_IGNORE` keeps it out of every colcon build:
its dependencies (fastmcp, pydantic v2) have no rosdep keys, so a `package.xml`
could not declare them. It runs on Python 3.10 and up, Humble's interpreter
and Jazzy's, and CI tests both.

It talks to the driver over ROS 2 topics and actions at runtime, so it needs a
sourced ROS 2 install with `rclpy` on the machine that runs it, but nothing in
`grippers/` or `robotiq_tsf/` depends on it.

## Development

```bash
cd mcp
uv sync            # creates .venv with the dev dependencies
uv run pytest
```

`uv.lock` is generated from `pyproject.toml` and committed: it pins every
dependency to an exact file and hash, and CI installs from it with
`uv sync --locked`, which fails if it is stale.

- `uv add <pkg>` / `uv remove <pkg>` edit `pyproject.toml` and the lock
  together; plain `uv sync` also refreshes the lock after a hand edit.
- `uv lock --upgrade` (or `--upgrade-package <pkg>`) moves pinned versions
  within the ranges `pyproject.toml` declares. Nothing else does.
- The lock also records this project's own version, so a release bump
  rewrites it (see `dev/version.py`).

Formatting and linting run through the repo-wide pre-commit config
(`pre-commit run -a` from the repo root). CI for this directory is
[`ci-mcp.yml`](../.github/workflows/ci-mcp.yml).
