#!/bin/bash
# No `set -u`: the ROS 2 setup scripts reference unset variables.
set -eo pipefail

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /ws/install/setup.bash

echo "[gripper-mcp] ROS_DISTRO=${ROS_DISTRO} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>} RMW=${RMW_IMPLEMENTATION:-<default>}"

exec /opt/gripper_mcp/bin/gripper-mcp "$@"
