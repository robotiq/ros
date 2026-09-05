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
import launch.logging
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

# PickNik's names for the topic-based path, released in 1.1.0. Each maps to its
# replacement and, for the topics, to the default it had then, which sim_isaac
# restores so a 1.1.0 command line keeps driving the same simulator.
DEPRECATED_ISAAC_ARGUMENTS = {
    "sim_isaac": ("sim_topic_based", None),
    "isaac_joint_commands": ("sim_joint_commands_topic", "/isaac_joint_commands"),
    "isaac_joint_states": ("sim_joint_states_topic", "/isaac_joint_states"),
}
DEPRECATION_NOTICE = (
    "{} deprecated since 1.2.0 and removed in the next major release; "
    "use sim_topic_based, sim_joint_commands_topic and sim_joint_states_topic"
)


def alias_deprecated_isaac_arguments(context):
    # Runs before the DeclareLaunchArguments, so the context holds only what the
    # caller actually passed and an explicit new-style argument always wins.
    given = context.launch_configurations
    used = [old for old in DEPRECATED_ISAAC_ARGUMENTS if old in given]
    if not used:
        return
    launch.logging.get_logger("robotiq_control.launch").warning(
        DEPRECATION_NOTICE.format(
            f"{', '.join(used)} {'is' if len(used) == 1 else 'are'}"
        )
    )
    restore_isaac_defaults = "sim_isaac" in used
    for old, (new, isaac_default) in DEPRECATED_ISAAC_ARGUMENTS.items():
        if old in used:
            given.setdefault(new, given[old])
        elif restore_isaac_defaults and isaac_default is not None:
            given.setdefault(new, isaac_default)


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
            "sim_joint_commands_topic:=",
            LaunchConfiguration("sim_joint_commands_topic"),
            " ",
            "sim_joint_states_topic:=",
            LaunchConfiguration("sim_joint_states_topic"),
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
            name="sim_joint_commands_topic",
            default_value="/sim/joint_commands",
            description="sim_topic_based only: JointState topic the simulator takes commands on",
        )
    )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="sim_joint_states_topic",
            default_value="/sim/joint_states",
            description="sim_topic_based only: JointState topic the simulator publishes",
        )
    )
    for old, (new, _) in DEPRECATED_ISAAC_ARGUMENTS.items():
        args.append(
            launch.actions.DeclareLaunchArgument(
                name=old,
                default_value="",
                description=f"Deprecated since 1.2.0: use {new}",
            )
        )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="shutdown_on_failure",
            default_value="true",
            description="End the launch when ros2_control_node exits, or, on Humble, when no "
            "controller could be activated. Set false when including this file next to "
            "nodes that should outlive the gripper",
        )
    )

    topic_based = LaunchConfiguration("sim_topic_based")

    robot_description_param = {
        "robot_description": launch_ros.parameter_descriptions.ParameterValue(
            xacro_command(), value_type=str
        )
    }

    distro = os.environ.get("ROS_DISTRO")
    controllers_file = ControllersFile(distro, topic_based)
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
                "--controller-manager-timeout",
                "10",
                "--param-file",
                ParameterFilePath(initial_joint_controllers),
            ],
            condition=condition,
        )

    # The reactivate_gripper GPIO the activation controller claims is declared for
    # the driver and the mock only; the topic_based plugin has nothing to reactivate.
    spawned_controllers = {
        "joint_state_broadcaster": None,
        "robotiq_gripper_controller": None,
        "robotiq_activation_controller": UnlessCondition(topic_based),
    }
    spawners = [
        spawner(name, condition) for name, condition in spawned_controllers.items()
    ]

    def starts(action, context):
        return action.condition is None or action.condition.evaluate(context)

    def is_set(context, name):
        return evaluate_condition_expression(context, [LaunchConfiguration(name)])

    # After a failed bringup the last line on screen is otherwise a spawner's
    # "process has died", which says nothing about the cause or the way out.
    returncodes = []

    def hint_after_the_last_spawner_exits(event, context):
        returncodes.append(event.returncode)
        if context.is_shutdown or len(returncodes) < sum(
            starts(s, context) for s in spawners
        ):
            return None
        # A negative code is a signal: the spawners were killed, they did not fail.
        if not any(code > 0 for code in returncodes):
            return None
        # Only a real gripper can be unplugged.
        if any(is_set(context, flag) for flag in HARDWARE_FLAGS):
            return None
        if distro == "humble":
            hint = launch.actions.LogInfo(
                msg="A controller could not be activated. If the gripper is not "
                "connected, ros2_control_node did not survive it (see its error above); "
                "reconnect the gripper and relaunch."
            )
            if not is_set(context, "shutdown_on_failure"):
                return [hint]
            return [hint, launch.actions.Shutdown(reason="no controller activated")]
        names = " ".join(
            name
            for name, action in zip(spawned_controllers, spawners)
            if starts(action, context)
        )
        return [
            launch.actions.LogInfo(
                msg="A controller could not be activated. If the gripper is not "
                "connected, follow the driver's error above to bring the hardware "
                f"back, then re-run the spawners: ros2 run controller_manager spawner {names}"
            )
        ]

    spawner_hint = launch.actions.RegisterEventHandler(
        launch.event_handlers.OnProcessExit(
            target_action=lambda action: action in spawners,
            on_exit=hint_after_the_last_spawner_exits,
        )
    )

    shutdown_on_control_node_exit = launch.actions.RegisterEventHandler(
        launch.event_handlers.OnProcessExit(
            target_action=control_node,
            on_exit=[launch.actions.Shutdown(reason="ros2_control_node exited")],
        ),
        condition=IfCondition(LaunchConfiguration("shutdown_on_failure")),
    )

    nodes = [
        OpaqueFunction(function=reject_conflicting_hardware_flags),
        control_node,
        robot_state_publisher_node,
        *spawners,
        rviz_node,
        shutdown_on_control_node_exit,
        spawner_hint,
    ]

    return launch.LaunchDescription(
        [OpaqueFunction(function=alias_deprecated_isaac_arguments)] + args + nodes
    )
