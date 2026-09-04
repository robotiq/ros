# gripper_mcp on top of this repo's ROS 2 image (docker/Dockerfile): that image
# already has rclpy, control_msgs and robotiq_tsf's messages on the distro's
# Python, so the server here only adds its own pure-Python dependencies.
#
# Build the base first, then this one with mcp/ as the context:
#   docker build -f docker/Dockerfile -t robotiq_ros2:jazzy .
#   docker build -t robotiq_gripper_mcp:jazzy mcp
#
# Jazzy and Lyrical only: fastmcp needs Python 3.12 and Humble ships 3.10.
# Point BASE_IMAGE at any image built from docker/Dockerfile (a headless build,
# a downstream image layered on it, ...); ROS_DISTRO comes from that image.
ARG BASE_IMAGE=robotiq_ros2:jazzy
FROM ${BASE_IMAGE}
ARG DEBIAN_FRONTEND=noninteractive

# The cell's DDS graph may run cyclonedds; the base image ships only the default
# RMW. Both are present so RMW_IMPLEMENTATION can be chosen at run time.
RUN apt-get update \
    && apt-get install -y --no-install-recommends "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp" \
    && rm -rf /var/lib/apt/lists/*

COPY --from=ghcr.io/astral-sh/uv:0.12.9 /uv /usr/local/bin/uv

# The venv must be the system interpreter's, with system site-packages visible:
# rclpy and the message packages are compiled against it and live in
# /opt/ros/<distro> and /ws/install, not on PyPI.
ENV UV_PYTHON=/usr/bin/python3 \
    UV_PYTHON_DOWNLOADS=never \
    UV_PROJECT_ENVIRONMENT=/opt/gripper_mcp \
    UV_LINK_MODE=copy \
    UV_COMPILE_BYTECODE=1

WORKDIR /app
RUN uv venv --system-site-packages /opt/gripper_mcp
COPY pyproject.toml uv.lock README.md ./
COPY gripper_mcp ./gripper_mcp
COPY robot_specifications ./robot_specifications
RUN uv sync --locked --no-dev

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Cell wiring is host-specific and is mounted, never baked in (see README).
EXPOSE 8300
ENTRYPOINT ["/entrypoint.sh"]
CMD ["--config", "/config/grippers.yaml"]
