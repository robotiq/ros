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

The tactile source subscribes to `robotiq_tsf`'s `TactileSensor/StaticData`
under the gripper's `namespace` (the tactile driver runs in the same namespace
as the gripper's controller); it needs `robotiq_tsf` built
and sourced alongside `rclpy`. The tests drive the tools from a scripted pad
model under `tests/fakes/`.

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

- `gripper_mcp/datasheets/<model>.yaml`: one datasheet per Robotiq
  model, shipped inside the package. It holds only what the robot cannot tell
  us: the stroke in mm, the rated grip force and timing defaults. Supporting
  another model is adding a file.
- `grippers.yaml`: the cell's wiring, which grippers exist, their model, the
  ROS namespace their driver runs under and an optional tactile source.
  Host-specific, so only `grippers.yaml.example` ships; `grippers.yaml` itself
  is gitignored.

Both refuse unknown keys, so a misspelled field fails at startup by name.

The command joint's name and range are not configured anywhere: the ROS backend
reads them from the `robot_description` that `robot_state_publisher` latches in
the gripper's namespace, picking the joint the finger joints `mimic` and the
driver's `gripper_closed_position` as the closed end. A cell built with a xacro
`prefix` therefore resolves by itself.

## Not a ROS package

`mcp/` is a plain Python project managed with [uv](https://docs.astral.sh/uv/),
not an ament package, and `COLCON_IGNORE` keeps it out of every colcon build:
its dependencies (fastmcp, pydantic v2) have no rosdep keys, so a `package.xml`
could not declare them. It runs on Python 3.10 and up, Humble's interpreter
and Jazzy's, and CI tests both.

It talks to the driver over ROS 2 topics and actions at runtime, so it needs a
sourced ROS 2 install with `rclpy` on the machine that runs it, but nothing in
`grippers/` or `robotiq_tsf/` depends on it.

## Docker

[`mcp/Dockerfile`](Dockerfile) layers the server on this repo's ROS 2 image
([`docker/Dockerfile`](../docker/Dockerfile)); its header explains the base
image, the venv on the system interpreter and the two build commands.
[`docker/docker-compose.yml`](../docker/docker-compose.yml) runs it as a
service; its header lists the environment knobs and how the wiring file is
mounted. The short version:

```bash
docker build -f docker/Dockerfile -t robotiq_ros2:jazzy .
docker build -t robotiq_gripper_mcp:jazzy mcp
docker compose -f docker/docker-compose.yml up --build
```

`grippers.yaml` is host-specific and is mounted at run time, never baked in.

### Demo, no hardware

[`demo/docker-compose.yml`](demo/docker-compose.yml) starts the driver on
ros2_control's fake hardware with RViz showing the 2F-85, next to the server
with [`demo/grippers.yaml`](demo/grippers.yaml); its header covers the
headless variant, RViz's lifecycle and why it sits on its own ROS domain.

```bash
./mcp/demo/demo.sh          # checks the images, xhost, compose up; `demo.sh down` tears it down
```

The server is now a tool server for any MCP client. With
[Claude Code](https://code.claude.com/docs/en/mcp), register it once and start a
new session:

```bash
claude mcp add --transport http robotiq-gripper http://127.0.0.1:8301/mcp
```

Then ask in plain language, RViz following along:

> Open the driver gripper to 30 mm, then grasp with it and tell me what
> happened.

The grasp closes on nothing (`closed_without_object`), because fake hardware
moves instantly and never meets resistance. A stall on an object, and with it
`gripper_verify_grasp`, needs a real gripper with TSF-85 pads; the driver on the SDK's simulated gripper will cover the
stall once that gripper models travel and object detection. The server's own
instructions tell the agent how to read those outcomes, so a stalled close is
reported as a grasp, not a failure.

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
