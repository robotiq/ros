ROS packages for Robotiq grippers and sensors.

## Packages

| Package | Description | ROS Version |
|---|---|---|
| [robotiq_tsf](robotiq_tsf/) | TSF-85 tactile sensor driver | ROS 2 Jazzy ([main](https://github.com/robotiq/ROS_Packages/tree/main)) / ROS 1 Noetic ([noetic](https://github.com/robotiq/ROS_Packages/tree/noetic)) |

## Legacy

| Repository | Description | ROS Version |
|---|---|---|
| [robotiq](https://github.com/ros-industrial-attic/robotiq) | Original ROS 1 Industrial Robotiq packages (archived, 3D CAD models are outdated) | ROS 1 Indigo / Kinetic / Melodic |

## Community & Third-Party

Also check out these community-maintained ROS drivers for Robotiq products.

| Repository | Description | ROS Version |
|---|---|---|
| [ros2_robotiq_gripper](https://github.com/PickNikRobotics/ros2_robotiq_gripper) | ROS 2 driver for Robotiq grippers | ROS 2 Humble / Iron / Rolling |
| [ros2_epick_gripper](https://github.com/PickNikRobotics/ros2_epick_gripper) | ROS 2 driver for the EPick vacuum gripper | ROS 2 Humble |
| [rq_fts_ros2_driver](https://github.com/panagelak/rq_fts_ros2_driver) | ROS 2 driver for the Robotiq force-torque sensor | ROS 2 Humble |
| [ros2_RobotiqGripper_UR](https://github.com/IFRA-Cranfield/ros2_RobotiqGripper_UR) | ROS 2 driver for Robotiq grippers on UR robots | ROS 2 Humble |

## Docker

The `docker/` folder provides scripts to build and run the TSF-85 driver inside a Docker container. Clone with submodules to pull in the required utilities:

```bash
git clone --recurse-submodules https://github.com/robotiq/ros.git
```

If you already cloned without `--recurse-submodules`:

```bash
git submodule update --init
```

| Script | Description |
|---|---|
| `build_launch_docker_ros2.sh` | Builds the Docker image and launches a container with sensor devices mapped in |
| `sensor_install.sh` | Sets up udev rules and permissions for bare-metal (non-Docker) use |
