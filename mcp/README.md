# gripper_mcp

MCP server for Robotiq 2F adaptive grippers. It gives an LLM agent a small,
gripper-shaped tool surface and drives the `ros2_control` driver in
[`grippers/`](../grippers/) underneath, so the same tools work against real
hardware and against a simulator launched with `sim_topic_based:=true`.

## Tools

| Tool | What it does |
|---|---|
| `gripper_list_grippers` | Discover configured gripper names |
| `gripper_get_state` | Opening (mm, fraction, joint rad) and grip force |
| `gripper_open` | Open fully |
| `gripper_close` | Close fully; a stall is an unexpected obstruction |
| `gripper_move_to` | Move to a specific position, in mm of opening |
| `gripper_grasp` | Close onto an object; stopping before the fingers meet is a success |
| `gripper_get_health` | Reachable, controller active |

Every tool takes an explicit `gripper_name`; nothing fans out to every gripper.
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
The tool surface is complete in this build; the backend that
talks to `robotiq_gripper_controller` lands in the next one, so until then
`build_service` refuses every wiring entry by name. Without hardware, run the
driver on ros2_control's fake hardware (`use_fake_hardware`) rather than a
Python stand-in: that exercises the driver, the controller and the action,
which a mock never will. The unit tests use a scripted double under
`tests/fakes/`.

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
- `grippers.yaml`: the cell's wiring, which grippers exist, their model and the
  ROS namespace their driver runs under. Host-specific, so only
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
