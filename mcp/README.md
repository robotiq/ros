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

## Docker

`mcp/Dockerfile` layers the server on this repo's ROS 2 image
([`docker/Dockerfile`](../docker/Dockerfile)), which already carries `rclpy`,
`control_msgs` and `robotiq_tsf`'s messages on the distro's Python. uv installs
`mcp/` into a venv that sees those system packages, so one interpreter imports
both `rclpy` and `fastmcp`. Jazzy and Lyrical only: Humble's Python 3.10 cannot
run fastmcp. Build the base image first, with the repo root as its context:

```bash
docker build -f docker/Dockerfile -t robotiq_ros2:jazzy .
docker build -t robotiq_gripper_mcp:jazzy mcp
```

`BASE_IMAGE` (default `robotiq_ros2:jazzy`) selects any image built from
`docker/Dockerfile`: a headless build, or something layered on top of it.
`grippers.yaml` is host-specific and is mounted at run time, never baked in.
The container needs the host's network to share the driver's DDS graph:

```bash
docker run --rm --network host --ipc host -e ROS_DOMAIN_ID=0 \
  -v "$PWD/mcp/grippers.yaml:/config/grippers.yaml:ro" robotiq_gripper_mcp:jazzy
```

[`docker/docker-compose.yml`](../docker/docker-compose.yml) does the same as a
service, with `ROBOTIQ_ROS2_IMAGE`, `GRIPPERS_YAML`, `ROS_DOMAIN_ID`,
`RMW_IMPLEMENTATION` and `GRIPPER_MCP_PORT` as optional environment knobs:

```bash
docker compose -f docker/docker-compose.yml up --build
```

Both `rmw_fastrtps_cpp` (the default) and `rmw_cyclonedds_cpp` are installed,
so the image joins either kind of graph by setting `RMW_IMPLEMENTATION`.

### Demo, no hardware

[`demo/docker-compose.yml`](demo/docker-compose.yml) starts the driver on
ros2_control's fake hardware next to the server, with RViz showing the 2F-85
move, and `demo/demo.py` walks the tools on both a real-driver gripper and the
tactile mock:

```bash
xhost +local:                                     # let the container's RViz draw
docker compose -f mcp/demo/docker-compose.yml up -d
cd mcp && uv run demo/demo.py
docker compose -f mcp/demo/docker-compose.yml down
```

`LAUNCH_RVIZ=false` runs it headless. The stack sits on `ROS_DOMAIN_ID=42` and
port 8301 so it never joins a real cell's graph.

### From Claude Code

The same stack works as a tool server for any MCP client. With
[Claude Code](https://code.claude.com/docs/en/mcp), register it once:

```bash
claude mcp add --transport http robotiq-gripper http://127.0.0.1:8301/mcp
```

Then ask in plain language, RViz following along:

> Open the driver gripper to 30 mm, then grasp with it and tell me what
> happened. Now grasp the object in the bench gripper and check whether it
> is really held.

The server's own instructions tell the agent how to read outcomes, so a stalled
close is reported as a grasp, not a failure.

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
