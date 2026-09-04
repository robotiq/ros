#!/usr/bin/env bash
# Bring the no-hardware demo up (driver on fake hardware + RViz + MCP server)
# or down: ./mcp/demo/demo.sh [up|down]
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${GRIPPER_MCP_PORT:-8301}"
BASE_IMAGE="${ROBOTIQ_ROS2_IMAGE:-robotiq_ros2:jazzy}"
MCP_IMAGE="${GRIPPER_MCP_IMAGE:-robotiq_gripper_mcp:jazzy}"

require_image() {
  docker image inspect "$1" >/dev/null 2>&1 || {
    echo "image $1 not found; build it first, from the repo root:" >&2
    echo "  docker build -f docker/Dockerfile -t ${BASE_IMAGE} ." >&2
    echo "  docker build -t ${MCP_IMAGE} mcp" >&2
    exit 1
  }
}

case "${1:-up}" in
  up)
    require_image "${BASE_IMAGE}"
    require_image "${MCP_IMAGE}"
    xhost +local:root >/dev/null 2>&1 || echo "xhost not available; RViz may not open"
    docker compose -f "${HERE}/docker-compose.yml" up -d
    echo
    echo "MCP server: http://127.0.0.1:${PORT}/mcp"
    echo "Claude Code: claude mcp add --transport http robotiq-gripper http://127.0.0.1:${PORT}/mcp"
    echo "Tear down:   $0 down"
    ;;
  down)
    docker compose -f "${HERE}/docker-compose.yml" down
    xhost -local:root >/dev/null 2>&1 || true
    ;;
  *)
    echo "usage: $0 [up|down]" >&2
    exit 2
    ;;
esac
