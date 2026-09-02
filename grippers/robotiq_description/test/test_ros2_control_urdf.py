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

"""Expand the gripper xacro and check the ros2_control interfaces it declares.

The two hardware plugins need different interface sets, and getting it wrong is
silent until a controller fails to activate at runtime:

* The real driver exports set_gripper_max_effort / set_gripper_max_velocity from
  export_command_interfaces() rather than from the URDF, and its on_init rejects a
  joint that declares more than one command interface.
* Mock hardware exposes only what the URDF declares, so those two interfaces have
  to be declared there or parallel_gripper_action_controller cannot claim them and
  never activates under use_fake_hardware:=true.
* motor_current and object_status are the reverse case: only the real driver
  writes them.
"""

import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

PKG_DIR = Path(__file__).parents[1]
URDF_DIR = PKG_DIR / "urdf"
CONFIG = PKG_DIR / "config" / "robotiq_controllers.yaml"

# Top-level xacro -> the joint carrying the gripper's position command.
MODELS = {
    "robotiq_2f_85_gripper.urdf.xacro": "robotiq_85_left_knuckle_joint",
    "robotiq_2f_140_gripper.urdf.xacro": "finger_joint",
}

MOCK_PLUGIN = "mock_components/GenericSystem"
REAL_PLUGIN = "robotiq_driver/RobotiqGripperHardwareInterface"

# Exported by the driver at runtime; needed from the URDF under mock hardware.
EXTRA_MOCK_COMMAND_INTERFACES = {"set_gripper_max_velocity", "set_gripper_max_effort"}

HARDWARE_ONLY_STATE_INTERFACES = {"motor_current", "object_status"}

requires_xacro = pytest.mark.skipif(
    shutil.which("xacro") is None, reason="xacro not on PATH"
)


def expand(model, use_fake_hardware, *extra_args):
    """Run xacro on `model` and return the parsed <ros2_control> element."""
    result = subprocess.run(
        [
            "xacro",
            str(URDF_DIR / model),
            f"use_fake_hardware:={str(use_fake_hardware).lower()}",
            *extra_args,
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    ros2_control = ET.fromstring(result.stdout).find("ros2_control")
    assert ros2_control is not None, f"{model} declares no <ros2_control> block"
    return ros2_control


def plugin_of(ros2_control):
    return ros2_control.find("hardware/plugin").text.strip()


def hardware_param(ros2_control, name):
    param = ros2_control.find(f"hardware/param[@name='{name}']")
    return None if param is None else param.text.strip()


def command_interfaces_of(ros2_control, joint_name):
    joint = ros2_control.find(f"joint[@name='{joint_name}']")
    assert joint is not None, f"joint {joint_name} missing from <ros2_control>"
    return {ci.get("name") for ci in joint.findall("command_interface")}


def state_interfaces_of(ros2_control, joint_name):
    joint = ros2_control.find(f"joint[@name='{joint_name}']")
    assert joint is not None, f"joint {joint_name} missing from <ros2_control>"
    return {si.get("name") for si in joint.findall("state_interface")}


def joints_of(ros2_control):
    return {j.get("name") for j in ros2_control.findall("joint")}


@requires_xacro
@pytest.mark.parametrize("model,joint", MODELS.items())
def test_mock_declares_the_interfaces_the_gripper_controller_claims(model, joint):
    ros2_control = expand(model, use_fake_hardware=True)
    assert plugin_of(ros2_control) == MOCK_PLUGIN
    assert (
        command_interfaces_of(ros2_control, joint)
        == {"position"} | EXTRA_MOCK_COMMAND_INTERFACES
    )


@requires_xacro
@pytest.mark.parametrize("model,joint", MODELS.items())
def test_real_hardware_declares_exactly_one_command_interface(model, joint):
    # RobotiqGripperHardwareInterface::on_init fails the component outright if the
    # joint declares anything other than a single position command interface.
    ros2_control = expand(model, use_fake_hardware=False)
    assert plugin_of(ros2_control) == REAL_PLUGIN
    assert command_interfaces_of(ros2_control, joint) == {"position"}


@requires_xacro
@pytest.mark.parametrize("model,joint", MODELS.items())
def test_mock_publishes_the_same_joints_as_hardware(model, joint):
    # The mimic joints must NOT be declared under mock: mock_components would then
    # own them and publish them in /joint_states, which Humble reports as 0.0 while
    # Jazzy and Lyrical apply the URDF multipliers. Leaving them out makes
    # robot_state_publisher derive them from the <mimic> tags on every distro,
    # exactly as it already does for real hardware.
    mock = joints_of(expand(model, use_fake_hardware=True))
    real = joints_of(expand(model, use_fake_hardware=False))
    assert mock == real == {joint}


@requires_xacro
def test_gripper_controller_config_interfaces_exist_under_mock():
    # Ties config/robotiq_controllers.yaml to the mock URDF: these two parameters
    # name "<joint>/<interface>" pairs the controller claims, and an unmatched name
    # is why the controller used to fail to activate with use_fake_hardware:=true.
    params = yaml.safe_load(CONFIG.read_text())["robotiq_gripper_controller"][
        "ros__parameters"
    ]
    ros2_control = expand("robotiq_2f_85_gripper.urdf.xacro", use_fake_hardware=True)
    joint = MODELS["robotiq_2f_85_gripper.urdf.xacro"]
    claimed = {
        params[k].replace("$(var gripper_joint)", joint)
        for k in ("max_effort_interface", "max_velocity_interface")
    }
    available = {
        f"{joint}/{name}" for name in command_interfaces_of(ros2_control, joint)
    }

    assert (
        claimed <= available
    ), f"controller claims {claimed - available}, which the mock URDF does not declare"


@requires_xacro
@pytest.mark.parametrize("model,joint", MODELS.items())
def test_only_real_hardware_declares_the_status_block_interfaces(model, joint):
    real = state_interfaces_of(expand(model, use_fake_hardware=False), joint)
    mock = state_interfaces_of(expand(model, use_fake_hardware=True), joint)

    assert HARDWARE_ONLY_STATE_INTERFACES <= real
    assert HARDWARE_ONLY_STATE_INTERFACES.isdisjoint(mock)
    assert real - HARDWARE_ONLY_STATE_INTERFACES == mock == {"position", "velocity"}


@requires_xacro
@pytest.mark.parametrize("model", MODELS)
def test_baudrate_reaches_the_driver(model):
    # The launch argument crosses three xacro files to get here, and a half
    # updated chain fails at runtime with `Invalid parameter "baudrate"` rather
    # than at expansion.
    ros2_control = expand(model, False, "baudrate:=57600")
    assert hardware_param(ros2_control, "baudrate") == "57600"

    assert hardware_param(expand(model, False), "baudrate") == "115200"
    assert hardware_param(expand(model, True), "baudrate") is None
