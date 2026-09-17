#!/usr/bin/env bash
# Build and run the single ROS 2 image for the robotiq ros workspace
# (robotiq_tsf sensor + grippers).
#
# Usage:
#   ./run.sh build                  # build the image only
#   ./run.sh [gripper|sensor|both]  # build if needed, then launch a shell with
#                                   # that product's devices mapped (default: gripper)
#
# Humble EOL: simplify the two lines below to name lyrical.
# Distro defaults to jazzy; humble and lyrical are equally supported:
#   ROS_DISTRO=humble ./run.sh gripper
#
# Inside the container the workspace is already built and sourced:
#   gripper bringup : ros2 launch robotiq_description robotiq_control.launch.py
#   sensor node     : ros2 run robotiq_tsf poll_data_sdk_node
#   rebuild         : cd /ws && colcon build --symlink-install

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DISTRO="${ROS_DISTRO:-jazzy}"
# One image and one container name per distro, so switching distros does not
# silently reuse the other one's build.
IMG="${IMG:-robotiq_ros2:${DISTRO}}"
CONTAINER="${CONTAINER:-robotiq_ros2_${DISTRO}}"
PRODUCT="${1:-gripper}"
HELPERS="${ROOT}/extern/tactile_sensors/utils/scripts"

# Sensor utilities live in the tactile_sensors submodule; source what's present
# (absent on gripper-only checkouts without --recurse-submodules).
for h in ensure_docker set_sensor_permissions find_sensor_devices setup_xhost apply_udev_rule ; do
  [[ -f "${HELPERS}/${h}.sh" ]] && source "${HELPERS}/${h}.sh"
done

has() { command -v "$1" >/dev/null 2>&1; }
log() { echo "[${1}] ${*:2}"; }   # log <tag> <message...>  e.g. log gripper "..."

# The sensor submodule's ensure_docker_image.sh is not used here: it provisions
# the standalone sensor image (pull REMOTE_IMAGE, else build Docker/Dockerfile_UI).
# Our unified sensor+grippers image is built from this repo's docker/Dockerfile,
# so we provision it locally below.
build_image() {
  log "${PRODUCT}" "building image ${IMG} for ROS ${DISTRO} (context: ${ROOT}) ..."
  docker build --build-arg "ROS_DISTRO=${DISTRO}" \
    -f "${SCRIPT_DIR}/Dockerfile" -t "${IMG}" "${ROOT}"
}

log "${PRODUCT}" "checking Docker installation ..."
if has ensure_docker; then ensure_docker; fi
if [[ "${PRODUCT}" == "build" ]]; then build_image; exit 0; fi

log "${PRODUCT}" "ensuring image ${IMG} exists ..."
docker image inspect "${IMG}" >/dev/null 2>&1 || build_image

if [[ -n "${DISPLAY:-}" ]] && has setup_xhost; then
  log "${PRODUCT}" "setting up X11 forwarding ..."
  setup_xhost
fi

# Per-product device mapping.
DEV=()
case "${PRODUCT}" in
  gripper|both)
    log gripper "detecting serial devices (/dev/ttyUSB*) ..."
    shopt -s nullglob
    for d in /dev/ttyUSB*; do log gripper "  found gripper device: ${d}"; DEV+=(--device="${d}"); done
    shopt -u nullglob
    ;;&
  sensor|both)
    if has apply_udev_rule; then log sensor "applying udev rules ..."; apply_udev_rule; sudo udevadm settle; fi
    if has set_sensor_permissions; then log sensor "setting sensor permissions ..."; set_sensor_permissions; fi
    if has find_sensor_devices; then
      log sensor "detecting sensor devices ..."
      found=0
      while IFS= read -r d; do
        if [[ -n "${d}" ]]; then
          real="$(readlink -f "${d}")"
          log sensor "  found sensor device: ${d} -> ${real}"
          # Map at the REAL tty name (not the udev symlink): libserialport (SDK
          # poller) resolves ports via /sys/class/tty/<name>, which only knows
          # the real device. The SDK node then autodetects it by USB descriptor.
          DEV+=(--device="${real}")
          found=1
        fi
      done < <(find_sensor_devices)
      if (( ! found )); then
        log sensor "WARNING: no sensor devices detected; the container will start without them."
        read -r -p "[sensor] continue anyway? [y/N] " reply
        [[ "${reply}" =~ ^[Yy]$ ]] || { log sensor "aborted."; exit 1; }
      fi
    fi
    ;;
esac

# Remove existing container if it exists.
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
  log "${PRODUCT}" "removing existing container ${CONTAINER} ..."
  docker rm -f "${CONTAINER}"
fi

log "${PRODUCT}" "launching ${IMG} ..."
docker run -it --rm \
  --name "${CONTAINER}" \
  --network host --privileged \
  -e DISPLAY="${DISPLAY:-}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  "${DEV[@]}" \
  "${IMG}"
