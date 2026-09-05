# Copyright (c) 2026 Robotiq
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Single-window view: the 2F-85 gripper with the tactile heatmaps mounted on
its fingers.

Brings up, in one RViz:
  - the gripper (robotiq_control.launch.py: ros2_control + robot_state_publisher),
  - two static TFs mounting the tactile pad frames on the gripper fingertip
    links (values from the demo), so the heatmaps move with the fingers,
  - the tactile driver + tactile_viz_node (publish_static_tfs:=false, so it
    renders into the gripper-mounted frames rather than its own),
  - RViz showing the RobotModel + MarkerArray + per-finger Image heatmaps.

  ros2 launch robotiq_tsf gripper_tactile_viz.launch.py com_port:=/dev/ttyUSB0

Run this INSTEAD of the separate gripper / tactile_viz launches.
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Tactile pad mount relative to its fingertip link, mirrored left/right:
# lateral offset toward the pad surface and height along the finger.
# Defaults for the 2F-85; override the mount_* / *_tip_link launch args to
# remount the pads or use another gripper (e.g. 2F-140).
TACTILE_MOUNT_X_M = 0.0252
TACTILE_MOUNT_Z_M = 0.032


def generate_launch_description():
    pkg_tsf = get_package_share_directory("robotiq_tsf")
    default_rviz = os.path.join(pkg_tsf, "rviz", "gripper_tactile.rviz")

    com_port = LaunchConfiguration("com_port")
    baudrate = LaunchConfiguration("baudrate")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    rviz_config = LaunchConfiguration("rviz_config")

    args = [
        DeclareLaunchArgument(
            "com_port", default_value="/dev/ttyUSB0", description="Gripper serial port."
        ),
        DeclareLaunchArgument(
            "baudrate",
            default_value="115200",
            description="Modbus RTU baudrate the gripper is configured for.",
        ),
        DeclareLaunchArgument(
            "poller",
            default_value="poll_data_sdk_node",
            description="Driver executable publishing StaticData. Override to "
            "point the viz at an alternative poller.",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Gripper ros2_control mock instead of real hardware.",
        ),
        DeclareLaunchArgument(
            "mount_x",
            default_value=str(TACTILE_MOUNT_X_M),
            description="Pad lateral offset from its fingertip link [m] "
            "(negated for the left finger).",
        ),
        DeclareLaunchArgument(
            "mount_z",
            default_value=str(TACTILE_MOUNT_Z_M),
            description="Pad height along the finger from its fingertip " "link [m].",
        ),
        DeclareLaunchArgument(
            "left_tip_link",
            default_value="robotiq_85_left_finger_tip_link",
            description="Parent link for the finger-0 pad frame.",
        ),
        DeclareLaunchArgument(
            "right_tip_link",
            default_value="robotiq_85_right_finger_tip_link",
            description="Parent link for the finger-1 pad frame.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz,
            description="Combined gripper + tactile RViz config.",
        ),
        DeclareLaunchArgument(
            "tactile_delay",
            default_value="8.0",
            description="Seconds to wait before starting tactile_viz (and thus "
            "its baseline capture), so the gripper finishes its "
            "activation open/close and settles OPEN — pads clear — "
            "first. The driver streams from t=0 regardless.",
        ),
    ]

    # Gripper: ros2_control + robot_state_publisher (its own RViz suppressed —
    # we run the combined one below).
    gripper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("robotiq_description"),
                    "launch",
                    "robotiq_control.launch.py",
                ]
            )
        ),
        launch_arguments={
            "com_port": com_port,
            "baudrate": baudrate,
            "use_fake_hardware": use_fake_hardware,
            "launch_rviz": "false",
            "shutdown_on_failure": "false",
        }.items(),
    )

    # Mount the tactile pad frames on the gripper fingertip links.
    # Children of the finger_tip links, so
    # they follow the fingers as the gripper opens/closes.
    anchor_0 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="finger_tip_to_tactile_finger_0",
        arguments=[
            "--x",
            PythonExpression(["-", LaunchConfiguration("mount_x")]),
            "--y",
            "0",
            "--z",
            LaunchConfiguration("mount_z"),
            "--frame-id",
            LaunchConfiguration("left_tip_link"),
            "--child-frame-id",
            "tactile_finger_0",
        ],
    )
    anchor_1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="finger_tip_to_tactile_finger_1",
        arguments=[
            "--x",
            LaunchConfiguration("mount_x"),
            "--y",
            "0",
            "--z",
            LaunchConfiguration("mount_z"),
            "--yaw",
            str(math.pi),
            "--frame-id",
            LaunchConfiguration("right_tip_link"),
            "--child-frame-id",
            "tactile_finger_1",
        ],
    )

    # Sensor driver (poller:= arg declared there).
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robotiq_tsf"), "launch", "tsf_driver.launch.py"]
            )
        )
    )

    # publish_static_tfs:=false — render into the gripper-mounted tactile_finger_*
    # frames (from the anchors above) instead of the node's standalone frames.
    viz = Node(
        package="robotiq_tsf",
        executable="tactile_viz_node",
        name="tactile_viz",
        output="screen",
        parameters=[
            {
                "input_topic": "TactileSensor/StaticData",
                "publish_static_tfs": False,
                "frame_id_finger_0": "tactile_finger_0",
                "frame_id_finger_1": "tactile_finger_1",
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    # Delay tactile_viz so its baseline is captured AFTER the gripper's
    # activation open/close finishes (gripper settled open, pads clear).
    # The driver still starts at t=0, so the raw stream is available.
    viz_delayed = TimerAction(
        period=LaunchConfiguration("tactile_delay"), actions=[viz]
    )

    return LaunchDescription(
        args + [gripper, anchor_0, anchor_1, driver, viz_delayed, rviz]
    )
