# Copyright (c) 2022 PickNik, Inc.
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
#    * Neither the name of the {copyright_holder} nor the names of its
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

import launch
from launch.actions import OpaqueFunction
from launch.substitution import Substitution
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.conditions import (
    IfCondition,
    UnlessCondition,
    evaluate_condition_expression,
)
import launch_ros
from launch_ros.parameter_descriptions import ParameterFile
import os

# Humble has no parallel_gripper_controller package, so it needs its own
# controller config, and the topic_based plugin exports a different
# interface set from the driver and the mock, so it needs its own too.
# See config/robotiq_controllers*.yaml.
JAZZY_CONTROLLERS_FILE = "robotiq_controllers.yaml"
HUMBLE_CONTROLLERS_FILE = "robotiq_controllers.humble.yaml"
JAZZY_TOPIC_BASED_CONTROLLERS_FILE = "robotiq_controllers.topic_based.yaml"
HUMBLE_TOPIC_BASED_CONTROLLERS_FILE = "robotiq_controllers.topic_based.humble.yaml"


def controllers_file_for_distro(distro, topic_based=False):
    """Return the controller config file name for ROS distro `distro`.

    `topic_based` selects the config for the sim_topic_based hardware plugin.
    """
    if distro == "humble":
        return (
            HUMBLE_TOPIC_BASED_CONTROLLERS_FILE
            if topic_based
            else HUMBLE_CONTROLLERS_FILE
        )
    return JAZZY_TOPIC_BASED_CONTROLLERS_FILE if topic_based else JAZZY_CONTROLLERS_FILE


class ControllersFile(Substitution):
    def __init__(self, distro, topic_based):
        super().__init__()
        self.distro = distro
        self.topic_based = topic_based

    def perform(self, context):
        topic_based = evaluate_condition_expression(context, [self.topic_based])
        return controllers_file_for_distro(self.distro, topic_based)


# The xacro emits one <plugin> per flag that is set, and ros2_control_node
# accepts only one, so two flags would fail late and obscurely.
HARDWARE_FLAGS = ("use_fake_hardware", "sim_topic_based")


def reject_conflicting_hardware_flags(context):
    enabled = [
        flag
        for flag in HARDWARE_FLAGS
        if evaluate_condition_expression(context, [LaunchConfiguration(flag)])
    ]
    if len(enabled) > 1:
        raise RuntimeError(
            f"{' and '.join(enabled)} both select a hardware plugin; set at most one"
        )


class ParameterFilePath(Substitution):
    def __init__(self, parameter_file):
        super().__init__()
        self.parameter_file = parameter_file

    def perform(self, context):
        return str(self.parameter_file.evaluate(context))


# Joint carrying the position command, per gripper_model. The launch argument
# is restricted to these keys, so an unknown model fails at launch instead of
# as a controller that never activates.
GRIPPER_JOINTS = {
    "2f_85": "robotiq_85_left_knuckle_joint",
    "2f_140": "finger_joint",
}


class DefaultGripperJoint(Substitution):
    def __init__(self, gripper_model):
        super().__init__()
        self.gripper_model = gripper_model

    def perform(self, context):
        return GRIPPER_JOINTS[self.gripper_model.perform(context)]


def xacro_command():
    return Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            LaunchConfiguration("model"),
            " ",
            "gripper_model:=",
            LaunchConfiguration("gripper_model"),
            " ",
            "use_fake_hardware:=",
            LaunchConfiguration("use_fake_hardware"),
            " ",
            "com_port:=",
            LaunchConfiguration("com_port"),
            " ",
            "baudrate:=",
            LaunchConfiguration("baudrate"),
            " ",
            "sim_topic_based:=",
            LaunchConfiguration("sim_topic_based"),
            " ",
            "joint_commands_topic:=",
            LaunchConfiguration("joint_commands_topic"),
            " ",
            "joint_states_topic:=",
            LaunchConfiguration("joint_states_topic"),
        ]
    )


def generate_launch_description():
    description_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="robotiq_description"
    ).find("robotiq_description")
    default_model_path = os.path.join(
        description_pkg_share, "urdf", "robotiq_2f_85_gripper.urdf.xacro"
    )
    default_rviz_config_path = os.path.join(
        description_pkg_share, "rviz", "view_urdf.rviz"
    )

    args = []
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="model",
            default_value=default_model_path,
            description="Absolute path to gripper URDF file",
        )
    )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="rvizconfig",
            default_value=default_rviz_config_path,
            description="Absolute path to rviz config file",
        )
    )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="launch_rviz", default_value="false", description="Launch RViz?"
        )
    )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="gripper_model",
            default_value="2f_85",
            choices=sorted(GRIPPER_JOINTS),
            description="Gripper the description and the controller are built for",
        )
    )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="gripper_joint",
            default_value=DefaultGripperJoint(LaunchConfiguration("gripper_model")),
            description="Joint the gripper controller drives; defaults from gripper_model",
        )
    )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="com_port",
            default_value="/dev/ttyUSB0",
            description="Port for communicating with Robotiq hardware",
        )
    )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="baudrate",
            default_value="115200",
            description="Modbus RTU baudrate the gripper is configured for",
        )
    )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="use_fake_hardware",
            default_value="false",
            description="Use ros2_control mock (fake) hardware instead of a real gripper",
        )
    )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="sim_topic_based",
            default_value="false",
            description="Drive a simulator over topic_based_ros2_control instead of a real gripper",
        )
    )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="joint_commands_topic",
            default_value="/sim/joint_commands",
            description="sim_topic_based only: JointState topic the simulator takes commands on",
        )
    )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="joint_states_topic",
            default_value="/sim/joint_states",
            description="sim_topic_based only: JointState topic the simulator publishes",
        )
    )

    topic_based = LaunchConfiguration("sim_topic_based")

    robot_description_param = {
        "robot_description": launch_ros.parameter_descriptions.ParameterValue(
            xacro_command(), value_type=str
        )
    }

    controllers_file = ControllersFile(os.environ.get("ROS_DISTRO"), topic_based)
    initial_joint_controllers = ParameterFile(
        PathJoinSubstitution([description_pkg_share, "config", controllers_file]),
        allow_substs=True,
    )

    control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description_param,
            initial_joint_controllers,
        ],
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description_param],
    )

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    # Each spawner is handed the controller config explicitly with --param-file.
    # Passing it to ros2_control_node alone is not enough: up to Jazzy the
    # controller_manager forwarded its own parameter file to each controller node,
    # but from Lyrical (ros2_control 6.x) it does not, and the gripper controller
    # then dies with "parameter 'joint' is not initialized". --param-file has been
    # a spawner option since Humble, and each node reads only its own section of
    # the file, so this is correct on every supported distro.
    def spawner(controller_name, condition=None):
        return launch_ros.actions.Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                controller_name,
                "--controller-manager",
                "/controller_manager",
                "--param-file",
                ParameterFilePath(initial_joint_controllers),
            ],
            condition=condition,
        )

    joint_state_broadcaster_spawner = spawner("joint_state_broadcaster")
    robotiq_gripper_controller_spawner = spawner("robotiq_gripper_controller")
    # The reactivate_gripper GPIO this controller claims is declared for the
    # driver and the mock only; the topic_based plugin has nothing to reactivate.
    robotiq_activation_controller_spawner = spawner(
        "robotiq_activation_controller", condition=UnlessCondition(topic_based)
    )

    nodes = [
        OpaqueFunction(function=reject_conflicting_hardware_flags),
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        robotiq_gripper_controller_spawner,
        robotiq_activation_controller_spawner,
        rviz_node,
    ]

    return launch.LaunchDescription(args + nodes)
