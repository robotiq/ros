import pytest

from gripper_mcp.robot_description import command_joint

LIMIT_2F_85_RAD = 0.8
LIMIT_2F_140_RAD = 0.7
DRIVER_CLOSED_2F_85_RAD = 0.7929

REAL_HARDWARE = """
      <hardware>
        <param name="use_dummy">false</param>
        <plugin>robotiq_driver/RobotiqGripperHardwareInterface</plugin>
        <param name="gripper_closed_position">0.7929</param>
        <param name="COM_port">/dev/ttyUSB0</param>
      </hardware>"""
FAKE_HARDWARE = """
      <hardware>
        <param name="use_dummy">false</param>
        <plugin>mock_components/GenericSystem</plugin>
        <param name="state_following_offset">0.0</param>
      </hardware>"""


def ros2_control(hardware, joint="robotiq_85_left_knuckle_joint"):
    return f"""
    <ros2_control name="RobotiqGripperHardwareInterface" type="system">{hardware}
      <joint name="{joint}">
        <command_interface name="position"/>
        <state_interface name="position">
          <param name="initial_value">0.7929</param>
        </state_interface>
      </joint>
    </ros2_control>"""


def gripper_urdf(
    prefix="", lower="0.0", upper=LIMIT_2F_85_RAD, with_limit=True, control=""
):
    limit = f'<limit lower="{lower}" upper="{upper}" effort="50" velocity="0.5"/>'
    return f"""
    <robot name="cell">{control}
      <joint name="{prefix}robotiq_85_base_joint" type="fixed"/>
      <joint name="{prefix}robotiq_85_left_knuckle_joint" type="revolute">
        {limit if with_limit else ""}
      </joint>
      <joint name="{prefix}robotiq_85_right_knuckle_joint" type="revolute">
        <limit lower="-0.8" upper="0.0" effort="50" velocity="0.5"/>
        <mimic joint="{prefix}robotiq_85_left_knuckle_joint" multiplier="-1"/>
      </joint>
      <joint name="{prefix}robotiq_85_left_finger_tip_joint" type="revolute">
        <limit lower="-0.8" upper="0.0" effort="50" velocity="0.5"/>
        <mimic joint="{prefix}robotiq_85_left_knuckle_joint" multiplier="-1"/>
      </joint>
    </robot>
    """


def test_the_command_joint_is_the_one_the_fingers_mimic():
    joint = command_joint(gripper_urdf())

    assert joint.name == "robotiq_85_left_knuckle_joint"
    assert joint.rad_open == 0.0


def test_the_closed_angle_is_the_drivers_not_the_joint_limit():
    joint = command_joint(gripper_urdf(control=ros2_control(REAL_HARDWARE)))

    assert joint.rad_closed == DRIVER_CLOSED_2F_85_RAD


def test_a_control_block_emitted_after_the_joints_does_not_shadow_them():
    control_last = gripper_urdf().replace(
        "</robot>", ros2_control(REAL_HARDWARE) + "</robot>"
    )

    joint = command_joint(control_last)

    assert joint.rad_open == 0.0
    assert joint.rad_closed == DRIVER_CLOSED_2F_85_RAD


def test_fake_hardware_falls_back_to_the_joint_limit():
    joint = command_joint(gripper_urdf(control=ros2_control(FAKE_HARDWARE)))

    assert joint.rad_closed == LIMIT_2F_85_RAD


def test_a_control_block_for_another_joint_is_ignored():
    arm = ros2_control(REAL_HARDWARE, joint="shoulder_pan_joint")

    joint = command_joint(gripper_urdf(control=arm))

    assert joint.rad_closed == LIMIT_2F_85_RAD


def test_a_description_without_ros2_control_uses_the_limit():
    assert command_joint(gripper_urdf()).rad_closed == LIMIT_2F_85_RAD


def test_a_prefixed_cell_resolves_to_the_prefixed_name():
    joint = command_joint(gripper_urdf(prefix="left_"))

    assert joint.name == "left_robotiq_85_left_knuckle_joint"


def test_a_2f_140_style_range_comes_through():
    joint = command_joint(gripper_urdf(upper=LIMIT_2F_140_RAD))

    assert joint.rad_closed == LIMIT_2F_140_RAD


def test_an_arm_only_description_is_refused_by_name():
    arm = '<robot name="arm"><joint name="shoulder" type="revolute"/></robot>'

    with pytest.raises(ValueError, match="found 0"):
        command_joint(arm)


def test_two_grippers_in_one_description_are_refused_with_both_named():
    two = gripper_urdf(prefix="left_").replace("</robot>", "") + gripper_urdf(
        prefix="right_"
    ).replace('<robot name="cell">', "")

    with pytest.raises(ValueError, match="left_robotiq.*right_robotiq"):
        command_joint(two)


def test_a_command_joint_without_limits_is_refused():
    with pytest.raises(ValueError, match="no <limit"):
        command_joint(gripper_urdf(with_limit=False))
