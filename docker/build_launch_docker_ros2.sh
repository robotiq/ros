#!/usr/bin/env bash

set -euo pipefail

# Get the directory containing this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Source utility scripts
source "${PROJECT_ROOT}/extern/tactile_sensors/utils/scripts/ensure_docker.sh"
source "${PROJECT_ROOT}/extern/tactile_sensors/utils/scripts/ensure_docker_image.sh"
source "${PROJECT_ROOT}/extern/tactile_sensors/utils/scripts/set_sensor_permissions.sh"
source "${PROJECT_ROOT}/extern/tactile_sensors/utils/scripts/find_sensor_devices.sh"
source "${PROJECT_ROOT}/extern/tactile_sensors/utils/scripts/setup_xhost.sh"
source "${PROJECT_ROOT}/extern/tactile_sensors/utils/scripts/apply_udev_rule.sh"


# Configuration
IMAGE_NAME="robotiq_tsf_ros2"
DOCKERFILE_PATH="${SCRIPT_DIR}/Dockerfile_TSF85_ROS2"
CONTAINER_NAME="robotiq_tsf_ros2_container"

echo "=== TSF-85 ROS2 Docker Build & Launch Script ==="

# Step 1: Ensure Docker is installed
echo "[1/6] Checking Docker installation..."
ensure_docker

# Step 2: Set up X11 forwarding for GUI applications
echo "[2/6] Setting up X11 forwarding..."
setup_xhost

# Step 3: Set sensor permissions

echo "[3/6] Applying udev rules..."
apply_udev_rule
sudo udevadm settle

echo "[4/6] Setting sensor permissions..."
set_sensor_permissions

# Step 4: Build Docker image if it doesn't exist
echo "[5/6] Ensuring Docker image exists..."
if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
  echo "Building Docker image ${IMAGE_NAME}..."

  # Build from the ROS directory with ROS_packages in context
  docker build -f "${DOCKERFILE_PATH}" -t "${IMAGE_NAME}" "${PROJECT_ROOT}"
else
  echo "Docker image ${IMAGE_NAME} already exists."
fi

# Step 5: Find sensor devices and prepare device mappings
echo "[6/6] Detecting sensor devices..."
DEVICE_ARGS=()
while IFS= read -r device; do
  if [[ -n "${device}" ]]; then
    real_device="$(readlink -f "${device}")"
    echo "  Found sensor device: ${device} -> ${real_device}"
    DEVICE_ARGS+=("--device=${real_device}:${device}")
  fi
done < <(find_sensor_devices)

if ((${#DEVICE_ARGS[@]} == 0)); then
  echo "Warning: No sensor devices detected. Container will launch without device mappings."
  read -r -p "Continue anyway? [y/N] " reply
  if [[ ! "$reply" =~ ^[Yy]$ ]]; then
    echo "Aborted."
    exit 1
  fi
fi

# Remove existing container if it exists
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
  echo "Removing existing container ${CONTAINER_NAME}..."
  docker rm -f "${CONTAINER_NAME}"
fi

# Launch the container
echo ""
echo "=== Launching Docker container ==="
docker run -it --rm \
  --name "${CONTAINER_NAME}" \
  --network host \
  --privileged \
  -e DISPLAY="${DISPLAY}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${PROJECT_ROOT}/robotiq_tsf:/root/jazzy_ws/src/robotiq_tsf:rw" \
  "${DEVICE_ARGS[@]}" \
  "${IMAGE_NAME}"

echo ""
echo "=== Container exited ==="
