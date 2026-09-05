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

# The simulated plugin (sim_isaac) exports neither the set_gripper_max_* command
# interfaces nor the reactivate_gripper GPIO, so it gets a config of its own per
# distro, selected by the same launch file.
JAZZY_SIM_CONFIG = CONFIG_DIR / "robotiq_controllers.sim.yaml"
HUMBLE_SIM_CONFIG = CONFIG_DIR / "robotiq_controllers.sim.humble.yaml"
SIM_CONFIGS = (JAZZY_SIM_CONFIG, HUMBLE_SIM_CONFIG)
SIM_OF = {JAZZY_CONFIG: JAZZY_SIM_CONFIG, HUMBLE_CONFIG: HUMBLE_SIM_CONFIG}

# Names exported by robotiq_driver's hardware interface for the joint it is
# given. Kept in sync by robotiq_driver's test_robotiq_gripper_hardware_interface,
# which asserts the hardware exports exactly these.
JOINT = "robotiq_85_left_knuckle_joint"
GRIPPER_JOINTS = (JOINT, "finger_joint")
UPDATE_RATE_HZ = 500

# Every key a controller_manager block may carry besides the controllers, per
# config. A misspelled setting is silently ignored by the controller_manager, so
# the blocks are compared exhaustively against these.
CONTROLLER_MANAGER_SETTINGS = {
    JAZZY_CONFIG: {"update_rate", "hardware_components_initial_state"},
    HUMBLE_CONFIG: {"update_rate"},
    JAZZY_SIM_CONFIG: {"update_rate"},
    HUMBLE_SIM_CONFIG: {"update_rate"},
}


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
    {SIM_OF[config]: plugin for config, plugin in EXPECTED_CONTROLLER_TYPES.items()}
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


@pytest.mark.parametrize("config", ALL_CONFIGS + SIM_CONFIGS)
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
    "use_fake_hardware,sim_isaac",
    [("false", "false"), ("true", "false"), ("false", "true")],
)
def test_launch_accepts_at_most_one_hardware_flag(use_fake_hardware, sim_isaac):
    launch_module = load_launch_module()
    context = LaunchContext()
    context.launch_configurations.update(
        use_fake_hardware=use_fake_hardware, sim_isaac=sim_isaac
    )
    launch_module.reject_conflicting_hardware_flags(context)


def test_launch_rejects_fake_hardware_together_with_sim_isaac():
    # Both flags set makes the xacro emit two <plugin> elements while the launch
    # picks the sim config; neither half can work, so fail before anything starts.
    launch_module = load_launch_module()
    context = LaunchContext()
    context.launch_configurations.update(use_fake_hardware="true", sim_isaac="true")
    with pytest.raises(RuntimeError, match="use_fake_hardware and sim_isaac"):
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
        sim_isaac="false",
        isaac_joint_commands="/isaac_joint_commands",
        isaac_joint_states="/isaac_joint_states",
    )
    rendered = "".join(s.perform(context) for s in command.command)
    assert "gripper_model:=2f_140" in rendered


@pytest.mark.parametrize("config", ALL_CONFIGS + SIM_CONFIGS)
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

    simulated = launch_module.controllers_file_for_distro(distro, simulated=True)
    assert simulated == SIM_OF[expected].name
    assert (CONFIG_DIR / simulated).is_file()


requires_launch = pytest.mark.skipif(
    importlib.util.find_spec("launch_ros") is None, reason="launch_ros not importable"
)


def launch_entities(distro, monkeypatch, **launch_arguments):
    """The launch's entities, and a context with its arguments resolved.

    `launch_arguments` override the declared defaults.
    """
    if distro is None:
        monkeypatch.delenv("ROS_DISTRO", raising=False)
    else:
        monkeypatch.setenv("ROS_DISTRO", distro)
    context = LaunchContext()
    context.launch_configurations.update(launch_arguments)
    entities = load_launch_module().generate_launch_description().entities
    for entity in entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    return entities, context


def starts(action, context):
    return action.condition is None or action.condition.evaluate(context)


def nodes_running(entities, context, executable):
    from launch_ros.actions import Node

    return [
        e
        for e in entities
        if isinstance(e, Node) and e.node_executable == executable
        if starts(e, context)
    ]


def spawner_commands(distro, monkeypatch, **launch_arguments):
    """Yield (controller, resolved command line) for every spawner that starts.

    A generator so the entities, and with them the ParameterFile's temporary
    file, outlive the caller's look at each command.
    """
    from launch.utilities import perform_substitutions

    entities, context = launch_entities(distro, monkeypatch, **launch_arguments)
    for spawner in nodes_running(entities, context, "spawner"):
        cmd = [perform_substitutions(context, part) for part in spawner.cmd]
        yield cmd[1], cmd


def spawned_controllers(distro, monkeypatch, **launch_arguments):
    return {
        name: Path(cmd[cmd.index("--param-file") + 1]).read_text()
        for name, cmd in spawner_commands(distro, monkeypatch, **launch_arguments)
    }


def resolved(config, joint=JOINT):
    return config.read_text().replace(JOINT_PLACEHOLDER, joint)


@requires_launch
@pytest.mark.parametrize(
    "distro,hardware_config,sim_config",
    [
        ("humble", HUMBLE_CONFIG, HUMBLE_SIM_CONFIG),
        ("jazzy", JAZZY_CONFIG, JAZZY_SIM_CONFIG),
        (None, JAZZY_CONFIG, JAZZY_SIM_CONFIG),
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

    simulated = spawned_controllers(distro, monkeypatch, sim_isaac="true")
    assert simulated == {
        "joint_state_broadcaster": resolved(sim_config),
        "robotiq_gripper_controller": resolved(sim_config),
    }


@pytest.mark.parametrize("config", SIM_CONFIGS)
def test_sim_configs_claim_no_driver_only_command_interfaces(config):
    # TopicBasedSystem exports the standard joint interfaces only; claiming
    # set_gripper_max_* here is why the sim path never activated.
    params = gripper_controller_params(config)
    assert "max_effort_interface" not in params
    assert "max_velocity_interface" not in params


@pytest.mark.parametrize("config", SIM_CONFIGS)
def test_sim_configs_spawn_no_activation_controller(config):
    # No reactivate_gripper GPIO is declared under sim_isaac (see
    # 2f_85.ros2_control.xacro), so the controller would have nothing to claim.
    controllers = load(config)["controller_manager"]["ros__parameters"]
    assert set(controllers) == CONTROLLER_MANAGER_SETTINGS[config] | {
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
    sim = gripper_controller_params(SIM_OF[config])
    assert sim["joint"] == hardware["joint"]
    assert sim["goal_tolerance"] == hardware["goal_tolerance"]
    assert hardware["allow_stalling"] is True
    assert sim["allow_stalling"] is False
    assert (
        load(SIM_OF[config])["controller_manager"]["ros__parameters"][
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
    assert set(controllers) == CONTROLLER_MANAGER_SETTINGS[config] | {
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


def test_jazzy_config_keeps_the_node_alive_without_a_gripper():
    controllers = load(JAZZY_CONFIG)["controller_manager"]["ros__parameters"]
    initial_state = controllers["hardware_components_initial_state"]
    assert initial_state["shutdown_on_initial_state_failure"] is False


def test_humble_config_declares_no_initial_state_policy():
    controllers = load(HUMBLE_CONFIG)["controller_manager"]["ros__parameters"]
    assert "hardware_components_initial_state" not in controllers


@requires_launch
def test_launch_gives_a_slow_gripper_ten_seconds_to_reach_the_controller_manager(
    monkeypatch,
):
    # Bounded so a controller_manager that never answers ends the bringup, wide
    # enough that a gripper still recovering from a fault is not taken for missing.
    for _, cmd in spawner_commands("jazzy", monkeypatch):
        assert cmd[cmd.index("--controller-manager-timeout") + 1] == "10"


PROCESS = {"name": "p", "cmd": ["p"], "cwd": None, "env": None, "pid": 1}


def dispatch(entities, context, event):
    from launch.actions import RegisterEventHandler

    emitted = []
    for entity in entities:
        if not isinstance(entity, RegisterEventHandler) or not starts(entity, context):
            continue
        if entity.event_handler.matches(event):
            emitted.extend(entity.event_handler.handle(event, context) or [])
    return emitted


def drive_spawner_exits(monkeypatch, returncodes, distro="jazzy", **launch_arguments):
    """Exit each spawner that starts, in order, with the given returncodes.

    Returns what the launch's handlers emitted after each exit.
    """
    from launch.events.process import ProcessExited

    entities, context = launch_entities(distro, monkeypatch, **launch_arguments)
    spawners = nodes_running(entities, context, "spawner")
    assert len(spawners) == len(returncodes)
    return [
        dispatch(entities, context, ProcessExited(action=s, returncode=rc, **PROCESS))
        for s, rc in zip(spawners, returncodes)
    ]


def hints(emitted_per_exit):
    from launch.actions import LogInfo

    return [
        [perform_text(e.msg) for e in emitted if isinstance(e, LogInfo)]
        for emitted in emitted_per_exit
    ]


def shutdowns(emitted_per_exit):
    from launch.actions import Shutdown

    return [
        [e for e in emitted if isinstance(e, Shutdown)] for emitted in emitted_per_exit
    ]


SPAWNER_RERUN = (
    "ros2 run controller_manager spawner "
    "joint_state_broadcaster robotiq_gripper_controller robotiq_activation_controller"
)


@requires_launch
@pytest.mark.parametrize("returncodes", [[1, 1, 1], [0, 1, 0], [1, 0, 0]])
def test_launch_hints_once_after_the_last_spawner_exits(monkeypatch, returncodes):
    # Verified on a 2F-85: bringing the hardware back leaves the controllers
    # loaded but inactive, and re-running the spawners is what activates them.
    emitted = hints(drive_spawner_exits(monkeypatch, returncodes))
    assert emitted[:-1] == [[], []]
    assert len(emitted[-1]) == 1
    assert SPAWNER_RERUN in emitted[-1][0]
    assert shutdowns(drive_spawner_exits(monkeypatch, returncodes)) == [[], [], []]


@requires_launch
def test_launch_stays_quiet_when_every_spawner_succeeds(monkeypatch):
    assert drive_spawner_exits(monkeypatch, [0, 0, 0]) == [[], [], []]


@requires_launch
def test_launch_stays_quiet_when_the_spawners_are_killed(monkeypatch):
    # SIGINT from a Ctrl-C or from the launch's own Shutdown is not a failure.
    assert drive_spawner_exits(monkeypatch, [-2, -2, -2]) == [[], [], []]


@requires_launch
def test_launch_stays_quiet_once_shutting_down(monkeypatch):
    from launch.events.process import ProcessExited

    entities, context = launch_entities("jazzy", monkeypatch)
    context._set_is_shutdown(True)
    for spawner in nodes_running(entities, context, "spawner"):
        event = ProcessExited(action=spawner, returncode=1, **PROCESS)
        assert dispatch(entities, context, event) == []


@requires_launch
@pytest.mark.parametrize(
    "launch_arguments,returncodes",
    [({"sim_isaac": "true"}, [1, 1]), ({"use_fake_hardware": "true"}, [1, 1, 1])],
)
def test_launch_gives_no_reconnect_advice_without_a_gripper(
    monkeypatch, launch_arguments, returncodes
):
    emitted = drive_spawner_exits(monkeypatch, returncodes, **launch_arguments)
    assert emitted == [[] for _ in returncodes]


@requires_launch
def test_launch_ends_on_humble_where_the_node_cannot_survive(monkeypatch):
    # Humble's controller_manager aborts, or wedges, on a failed connect, so the
    # spawners' failure is the only exit signal the launch can act on there.
    emitted = drive_spawner_exits(monkeypatch, [1, 1, 1], distro="humble")
    assert hints(emitted)[:-1] == [[], []]
    assert "relaunch" in hints(emitted)[-1][0]
    assert SPAWNER_RERUN not in hints(emitted)[-1][0]
    assert [len(s) for s in shutdowns(emitted)] == [0, 0, 1]


@requires_launch
def test_launch_keeps_running_on_humble_when_asked(monkeypatch):
    emitted = drive_spawner_exits(
        monkeypatch, [1, 1, 1], distro="humble", shutdown_on_failure="false"
    )
    assert [len(h) for h in hints(emitted)] == [0, 0, 1]
    assert shutdowns(emitted) == [[], [], []]


def control_node_exit(monkeypatch, **launch_arguments):
    from launch.events.process import ProcessExited

    entities, context = launch_entities("jazzy", monkeypatch, **launch_arguments)
    (control_node,) = nodes_running(entities, context, "ros2_control_node")
    event = ProcessExited(action=control_node, returncode=-6, **PROCESS)
    return dispatch(entities, context, event)


@requires_launch
def test_launch_ends_when_the_control_node_exits(monkeypatch):
    from launch.actions import Shutdown

    emitted = control_node_exit(monkeypatch)
    assert len(emitted) == 1
    assert isinstance(emitted[0], Shutdown)


@requires_launch
def test_launch_can_outlive_the_control_node_for_includers(monkeypatch):
    # gripper_tactile_viz.launch.py includes this file next to the tactile
    # stack, which has no reason to stop streaming when the gripper is gone.
    assert control_node_exit(monkeypatch, shutdown_on_failure="false") == []


def perform_text(msg):
    return "".join(s.perform(LaunchContext()) for s in msg)
