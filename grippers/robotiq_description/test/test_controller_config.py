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

# The gripper controller can only activate if the command interfaces it is
# configured to claim exist under the exact names the hardware exports (see
# RobotiqGripperHardwareInterface::export_command_interfaces in
# robotiq_driver). A mismatch is silent until runtime, where
# robotiq_gripper_controller fails to activate.
#
# There are two configs — Humble has no parallel_gripper_controller package — and
# robotiq_control.launch.py picks one per $ROS_DISTRO. Both are checked here
# regardless of the distro the tests run on, so neither can rot unnoticed.

import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

CONFIG_DIR = Path(__file__).parents[1] / "config"
JAZZY_CONFIG = CONFIG_DIR / "robotiq_controllers.yaml"
HUMBLE_CONFIG = CONFIG_DIR / "robotiq_controllers.humble.yaml"
ALL_CONFIGS = (JAZZY_CONFIG, HUMBLE_CONFIG)

# The configs name the joint through this launch placeholder so one file serves
# both models; robotiq_control.launch.py resolves it via ParameterFile.
JOINT_PLACEHOLDER = "$(var gripper_joint)"

# The topic_based plugin exports neither the set_gripper_max_* command
# interfaces nor the reactivate_gripper GPIO, so it gets a config of its own per
# distro, selected by the same launch file.
JAZZY_TOPIC_BASED_CONFIG = CONFIG_DIR / "robotiq_controllers.topic_based.yaml"
HUMBLE_TOPIC_BASED_CONFIG = CONFIG_DIR / "robotiq_controllers.topic_based.humble.yaml"
TOPIC_BASED_CONFIGS = (JAZZY_TOPIC_BASED_CONFIG, HUMBLE_TOPIC_BASED_CONFIG)
TOPIC_BASED_OF = {
    JAZZY_CONFIG: JAZZY_TOPIC_BASED_CONFIG,
    HUMBLE_CONFIG: HUMBLE_TOPIC_BASED_CONFIG,
}

# Names exported by robotiq_driver's hardware interface for the joint it is
# given. Kept in sync by robotiq_driver's test_robotiq_gripper_hardware_interface,
# which asserts the hardware exports exactly these.
JOINT = "robotiq_85_left_knuckle_joint"
GRIPPER_JOINTS = (JOINT, "finger_joint")
UPDATE_RATE_HZ = 500


def exported_command_interfaces(joint):
    return {
        f"{joint}/position",
        f"{joint}/set_gripper_max_velocity",
        f"{joint}/set_gripper_max_effort",
    }


# Plugin the launch expects per distro. Humble's stock controller takes a
# control_msgs/GripperCommand goal, matching PickNik's humble branch; Jazzy and
# newer take ParallelGripperCommand.
EXPECTED_CONTROLLER_TYPES = {
    JAZZY_CONFIG: "parallel_gripper_action_controller/GripperActionController",
    HUMBLE_CONFIG: "position_controllers/GripperActionController",
}
EXPECTED_CONTROLLER_TYPES.update(
    {
        TOPIC_BASED_OF[config]: plugin
        for config, plugin in EXPECTED_CONTROLLER_TYPES.items()
    }
)


def load_launch_module():
    """Import robotiq_control.launch.py for its distro -> config-file mapping."""
    path = Path(__file__).parents[1] / "launch" / "robotiq_control.launch.py"
    spec = importlib.util.spec_from_file_location("robotiq_control_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load(config):
    with open(config) as f:
        return yaml.safe_load(f)


def gripper_controller_params(config, joint=JOINT):
    params = load(config)["robotiq_gripper_controller"]["ros__parameters"]
    return {
        k: v.replace(JOINT_PLACEHOLDER, joint) if isinstance(v, str) else v
        for k, v in params.items()
    }


def perform(substitution, **launch_configurations):
    context = LaunchContext()
    context.launch_configurations.update(launch_configurations)
    return substitution.perform(context)


@pytest.mark.parametrize("joint", GRIPPER_JOINTS)
def test_claimed_interfaces_follow_the_joint(joint):
    # The two extra interfaces are named "<joint>/<interface>", so they must
    # track whichever joint the launch resolves, not the 2F-85's.
    params = gripper_controller_params(JAZZY_CONFIG, joint)
    claimed = {params["max_effort_interface"], params["max_velocity_interface"]}
    assert claimed <= exported_command_interfaces(joint)


def test_humble_config_claims_no_unsupported_interfaces():
    # Humble's gripper_controllers declares neither parameter; leaving them in
    # would read as working speed/force control that Humble silently ignores.
    params = gripper_controller_params(HUMBLE_CONFIG)
    assert "max_effort_interface" not in params
    assert "max_velocity_interface" not in params


@pytest.mark.parametrize("config", ALL_CONFIGS + TOPIC_BASED_CONFIGS)
def test_joint_is_left_to_the_launch(config):
    # Hardcoding the 2F-85 joint here is why a 2F-140 launch used to fail to
    # activate robotiq_gripper_controller.
    raw = load(config)["robotiq_gripper_controller"]["ros__parameters"]
    assert raw["joint"] == JOINT_PLACEHOLDER
    assert JOINT not in yaml.safe_dump(raw)


def test_launch_description_builds():
    # Import errors in the launch file only surface when `ros2 launch` runs it.
    load_launch_module().generate_launch_description()


@pytest.mark.parametrize(
    "gripper_model,expected",
    [("2f_85", JOINT), ("2f_140", "finger_joint")],
)
def test_launch_defaults_the_joint_from_the_gripper_model(gripper_model, expected):
    launch_module = load_launch_module()
    joint = launch_module.DefaultGripperJoint(LaunchConfiguration("gripper_model"))
    assert perform(joint, gripper_model=gripper_model) == expected


def test_launch_restricts_gripper_model_to_the_known_joints():
    launch_module = load_launch_module()
    description = launch_module.generate_launch_description()
    declared = {
        action.name: action
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert declared["gripper_model"].choices == sorted(launch_module.GRIPPER_JOINTS)


@pytest.mark.parametrize(
    "use_fake_hardware,sim_topic_based",
    [("false", "false"), ("true", "false"), ("false", "true")],
)
def test_launch_accepts_at_most_one_hardware_flag(use_fake_hardware, sim_topic_based):
    launch_module = load_launch_module()
    context = LaunchContext()
    context.launch_configurations.update(
        use_fake_hardware=use_fake_hardware, sim_topic_based=sim_topic_based
    )
    launch_module.reject_conflicting_hardware_flags(context)


def test_launch_rejects_fake_hardware_together_with_sim_topic_based():
    # Both flags set makes the xacro emit two <plugin> elements while the launch
    # picks the topic_based config; neither half can work, so fail before anything starts.
    launch_module = load_launch_module()
    context = LaunchContext()
    context.launch_configurations.update(
        use_fake_hardware="true", sim_topic_based="true"
    )
    with pytest.raises(RuntimeError, match="use_fake_hardware and sim_topic_based"):
        launch_module.reject_conflicting_hardware_flags(context)


def test_launch_forwards_the_gripper_model_to_xacro():
    # The xacro-level gripper_model selects the macro, so the launch-level one
    # has to reach it or the two could name different grippers.
    launch_module = load_launch_module()
    command = launch_module.xacro_command()
    context = LaunchContext()
    context.launch_configurations.update(
        model="/x/gripper.urdf.xacro",
        gripper_model="2f_140",
        use_fake_hardware="true",
        com_port="/dev/null",
        baudrate="115200",
        sim_topic_based="false",
        joint_commands_topic="/sim/joint_commands",
        joint_states_topic="/sim/joint_states",
    )
    rendered = "".join(s.perform(context) for s in command.command)
    assert "gripper_model:=2f_140" in rendered


@pytest.mark.parametrize("config", ALL_CONFIGS + TOPIC_BASED_CONFIGS)
def test_controller_type_matches_distro(config):
    controllers = load(config)["controller_manager"]["ros__parameters"]
    assert (
        controllers["robotiq_gripper_controller"]["type"]
        == EXPECTED_CONTROLLER_TYPES[config]
    )


@pytest.mark.parametrize(
    "distro,expected",
    [
        ("humble", HUMBLE_CONFIG),
        ("jazzy", JAZZY_CONFIG),
        ("lyrical", JAZZY_CONFIG),
        (None, JAZZY_CONFIG),  # ROS_DISTRO unset: newest layout is the default
    ],
)
def test_launch_selects_the_config_for_the_distro(distro, expected):
    launch_module = load_launch_module()
    selected = launch_module.controllers_file_for_distro(distro)
    assert selected == expected.name
    assert (CONFIG_DIR / selected).is_file()

    topic_based = launch_module.controllers_file_for_distro(distro, topic_based=True)
    assert topic_based == TOPIC_BASED_OF[expected].name
    assert (CONFIG_DIR / topic_based).is_file()


requires_launch = pytest.mark.skipif(
    importlib.util.find_spec("launch_ros") is None, reason="launch_ros not importable"
)


def spawned_controllers(distro, monkeypatch, **launch_arguments):
    from launch import LaunchContext
    from launch.utilities import perform_substitutions
    from launch_ros.actions import Node

    if distro is None:
        monkeypatch.delenv("ROS_DISTRO", raising=False)
    else:
        monkeypatch.setenv("ROS_DISTRO", distro)
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "sim_topic_based": "false",
            "gripper_joint": JOINT,
            **launch_arguments,
        }
    )

    spawned = {}
    for action in load_launch_module().generate_launch_description().entities:
        if not isinstance(action, Node) or action.node_executable != "spawner":
            continue
        if action.condition is not None and not action.condition.evaluate(context):
            continue
        cmd = [perform_substitutions(context, part) for part in action.cmd]
        spawned[cmd[1]] = Path(cmd[cmd.index("--param-file") + 1]).read_text()
    return spawned


def resolved(config, joint=JOINT):
    return config.read_text().replace(JOINT_PLACEHOLDER, joint)


@requires_launch
@pytest.mark.parametrize(
    "distro,hardware_config,sim_config",
    [
        ("humble", HUMBLE_CONFIG, HUMBLE_TOPIC_BASED_CONFIG),
        ("jazzy", JAZZY_CONFIG, JAZZY_TOPIC_BASED_CONFIG),
        (None, JAZZY_CONFIG, JAZZY_TOPIC_BASED_CONFIG),
    ],
)
def test_launch_spawns_per_plugin(distro, hardware_config, sim_config, monkeypatch):
    # Every spawner must be handed the same config the controller_manager loaded,
    # and the activation controller only exists where its GPIO does.
    hardware = spawned_controllers(distro, monkeypatch)
    assert hardware == {
        "joint_state_broadcaster": resolved(hardware_config),
        "robotiq_gripper_controller": resolved(hardware_config),
        "robotiq_activation_controller": resolved(hardware_config),
    }

    topic_based = spawned_controllers(distro, monkeypatch, sim_topic_based="true")
    assert topic_based == {
        "joint_state_broadcaster": resolved(sim_config),
        "robotiq_gripper_controller": resolved(sim_config),
    }


@pytest.mark.parametrize("config", TOPIC_BASED_CONFIGS)
def test_sim_configs_claim_no_driver_only_command_interfaces(config):
    # TopicBasedSystem exports the standard joint interfaces only; claiming
    # set_gripper_max_* here is why the sim_topic_based path never activated.
    params = gripper_controller_params(config)
    assert "max_effort_interface" not in params
    assert "max_velocity_interface" not in params


@pytest.mark.parametrize("config", TOPIC_BASED_CONFIGS)
def test_sim_configs_spawn_no_activation_controller(config):
    # No reactivate_gripper GPIO is declared under sim_topic_based (see
    # 2f_85.ros2_control.xacro), so the controller would have nothing to claim.
    controllers = load(config)["controller_manager"]["ros__parameters"]
    assert set(controllers) == {
        "update_rate",
        "joint_state_broadcaster",
        "robotiq_gripper_controller",
    }
    assert controllers["update_rate"] == UPDATE_RATE_HZ
    assert "robotiq_activation_controller" not in load(config)


@pytest.mark.parametrize("config", ALL_CONFIGS)
def test_sim_config_keeps_the_action_surface_of_its_distro(config):
    # Same controller name, type, joint and goal tolerance as the hardware config
    # for that distro: an action client must not notice which plugin is behind.
    # Stall handling is the one deliberate difference: a simulator that publishes
    # no velocities would otherwise pass every failed grasp as a successful stall.
    hardware = gripper_controller_params(config)
    sim = gripper_controller_params(TOPIC_BASED_OF[config])
    assert sim["joint"] == hardware["joint"]
    assert sim["goal_tolerance"] == hardware["goal_tolerance"]
    assert hardware["allow_stalling"] is True
    assert sim["allow_stalling"] is False
    assert (
        load(TOPIC_BASED_OF[config])["controller_manager"]["ros__parameters"][
            "robotiq_gripper_controller"
        ]
        == load(config)["controller_manager"]["ros__parameters"][
            "robotiq_gripper_controller"
        ]
    )


@pytest.mark.parametrize("config", ALL_CONFIGS)
def test_controller_names_and_update_rate_are_identical_across_distros(config):
    # The controller names are the action/service namespaces users depend on
    # (/robotiq_gripper_controller/gripper_cmd), so they must not drift between
    # the two configs.
    controllers = load(config)["controller_manager"]["ros__parameters"]
    assert set(controllers) == {
        "update_rate",
        "joint_state_broadcaster",
        "robotiq_gripper_controller",
        "robotiq_activation_controller",
    }
    assert controllers["update_rate"] == UPDATE_RATE_HZ
    assert (
        controllers["robotiq_activation_controller"]["type"]
        == "robotiq_controllers/RobotiqActivationController"
    )
    assert (
        controllers["joint_state_broadcaster"]["type"]
        == "joint_state_broadcaster/JointStateBroadcaster"
    )
