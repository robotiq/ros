# gripper_mcp

MCP server for Robotiq 2F adaptive grippers. It gives an LLM agent a small,
gripper-shaped tool surface (open, close, grasp, read state) and drives the
`ros2_control` driver in [`grippers/`](../grippers/) underneath, so the same
tools work against real hardware and against a simulator launched with
`sim_isaac:=true`.

Work in progress: this directory currently holds the project skeleton only. The
tool surface, the ROS 2 backend and the tactile tools land in follow-up pull
requests.

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
