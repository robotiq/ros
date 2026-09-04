#!/usr/bin/env bash
# Bring the no-hardware demo up (driver on fake hardware + RViz + MCP server)
# or down: ./mcp/demo/demo.sh [up|down]
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${GRIPPER_MCP_PORT:-8301}"

case "${1:-up}" in
  up)
    xhost +local: >/dev/null 2>&1 || echo "xhost not available; RViz may not open"
    docker compose -f "${HERE}/docker-compose.yml" up -d
    echo
    echo "MCP server: http://127.0.0.1:${PORT}/mcp"
    echo "Claude Code: claude mcp add --transport http robotiq-gripper http://127.0.0.1:${PORT}/mcp"
    echo "Tear down:   $0 down"
    ;;
  down)
    docker compose -f "${HERE}/docker-compose.yml" down
    ;;
  *)
    echo "usage: $0 [up|down]" >&2
    exit 2
    ;;
esac
