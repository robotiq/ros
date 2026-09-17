# Multi-stage ROS 2 image for the whole robotiq ros workspace:
#   - robotiq_tsf   (TSF-85 tactile sensor driver)
#   - grippers/*    (Robotiq gripper driver, forked from PickNik)
#   - extern/*      (the tactile-sensor and gripper SDK submodules)
#
# Stage 1 (builder): full toolchain, compiles the workspace.
# Stage 2 (runtime, default): slim — only runtime deps + the built install/, no
#   compiler / colcon / test deps. GUI (rviz2, rqt, …) is optional.
#
# Build options, examples and the build-context requirement: see the Docker
# section in README.md. ROS_DISTRO is a build arg: humble, jazzy and lyrical.
# Humble EOL: simplify the distro list above.

ARG ROS_DISTRO=jazzy

##############################  builder  ##############################
FROM ros:${ROS_DISTRO}-ros-base AS builder
ARG ROS_DISTRO
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Build tooling + sensor build deps + gripper
# build deps (ros2_control stack). ros2-controllers brings the distro's gripper
# action controller: gripper_controllers on Humble, parallel_gripper_controller
# on Jazzy and newer.
# Humble EOL: simplify — only parallel_gripper_controller remains.
RUN apt-get update && apt-get install -y \
        pkg-config \
        python3-colcon-common-extensions python3-rosdep \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-controller-manager \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-joint-state-publisher \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true; rosdep update

RUN mkdir -p /ws/src
COPY robotiq_tsf /ws/src/robotiq_tsf
COPY grippers    /ws/src/grippers
# Tactile sensors SDK: robotiq_tsf's poll_data_sdk_node compiles it via the
# relative path ../extern/tactile_sensors/sdk_cpp. COLCON_IGNORE stops colcon
# from building the SDK's own standalone CMakeLists as a separate package.
COPY extern/tactile_sensors/sdk_cpp /ws/src/extern/tactile_sensors/sdk_cpp
RUN touch /ws/src/extern/tactile_sensors/sdk_cpp/COLCON_IGNORE
# Gripper SDK (robotiq/grippers), same arrangement: COPYing the submodule
# checkout instead of cloning it means local, still-uncommitted SDK work
# compiles here too.
COPY extern/grippers/sdk_cpp /ws/src/extern/grippers/sdk_cpp
RUN touch /ws/src/extern/grippers/sdk_cpp/COLCON_IGNORE

# Resolve remaining rosdeps (build + exec only — NOT test deps, e.g.
# ament_clang_tidy → clang-tools), then build Release. No --symlink-install, so
# install/ is self-contained and can be copied into the runtime stage.
# BUILD_TESTING=OFF: this is a deployment image; tests are not run here.
RUN apt-get update \
    && rosdep install --from-paths /ws/src --ignore-src -y --rosdistro ${ROS_DISTRO} \
        --dependency-types buildtool --dependency-types build \
        --dependency-types build_export --dependency-types buildtool_export \
        --dependency-types exec \
    && rm -rf /var/lib/apt/lists/* \
    && bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash \
        && cd /ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"

##########################  runtime (default)  ########################
# ros-core (not ros-base): the minimal ROS layer, without build-essential /
# colcon / rosdep. Runtime deps are apt-installed explicitly below.
FROM ros:${ROS_DISTRO}-ros-core AS runtime
ARG ROS_DISTRO
ARG DEBIAN_FRONTEND=noninteractive
ARG WITH_GUI=true
ENV TZ=Etc/UTC

# Runtime deps only — no compiler, no colcon/rosdep, no test deps. Some -dev
# packages still arrive anyway (~350 MB): ROS binary debs hard-depend on them,
# e.g. ros-*-tf2-eigen -> libeigen3-dev, rviz-rendering -> mesa -dev. They are
# inert with no compiler present, and unavoidable short of unbundling those debs.
# GUI tools (rviz2/rqt/joint-state-publisher-gui/terminator) are optional; drop
# them for headless robots with --build-arg WITH_GUI=false.
RUN apt-get update && apt-get install -y \
        libserialport0 \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-controller-manager \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-joint-state-publisher \
    && if [ "${WITH_GUI}" = "true" ]; then apt-get install -y \
        terminator \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-rqt \
        ros-${ROS_DISTRO}-rqt-controller-manager ; fi \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /ws/install /ws/install

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
 && echo "source /ws/install/setup.bash"            >> /root/.bashrc
WORKDIR /ws
ENTRYPOINT ["/bin/bash"]
