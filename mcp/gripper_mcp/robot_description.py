"""Find the gripper's command joint in a robot description, with no ROS imports.

A Robotiq 2F is one actuated joint plus finger joints that `<mimic>` it, so the
command joint is the one the others point at. That holds whatever `prefix` the
xacro was expanded with, which is why the name is read here rather than kept in
a datasheet. The joint's lower limit is the open angle. The closed angle is the
driver's, not the joint's: the hardware interface scales the register to
`gripper_closed_position` (0.7929 on a 2F-85, against a limit of 0.8), which
the description carries as a `<hardware>` param of the `<ros2_control>` block
that lists the joint. Where that block has no such param (the fake hardware,
the topic-based sim) the mock mirrors commands and the limit is as good as
anything, so it is the fallback.
"""

from xml.etree import ElementTree

from gripper_mcp.units import JointGeometry


def command_joint(urdf_xml: str) -> JointGeometry:
    robot = ElementTree.fromstring(urdf_xml)
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
    name = mimicked_joint(robot, set(joints))
    rad_open, limit_upper = limits(joints[name])
    return JointGeometry(name, rad_open, closed_angle(robot, name, limit_upper))


def mimicked_joint(robot: ElementTree.Element, joint_names: set[str]) -> str:
    mimicked = sorted(
        {mimic.attrib["joint"] for mimic in robot.iter("mimic")} & joint_names
    )
    if len(mimicked) != 1:
        raise ValueError(
            "Expected exactly one joint mimicked by the finger joints, found "
            f"{len(mimicked)}: {', '.join(mimicked) or '(none)'}. Is this the "
            "description of a single Robotiq 2F gripper?"
        )
    return mimicked[0]


def closed_angle(robot: ElementTree.Element, joint: str, limit_upper: float) -> float:
    for block in robot.iter("ros2_control"):
        if block.find(f"joint[@name='{joint}']") is None:
            continue
        param = block.find("hardware/param[@name='gripper_closed_position']")
        if param is not None and param.text:
            return float(param.text)
    return limit_upper


def limits(joint: ElementTree.Element) -> tuple[float, float]:
    limit = joint.find("limit")
    try:
        return float(limit.attrib["lower"]), float(limit.attrib["upper"])
    except (AttributeError, KeyError) as error:
        raise ValueError(
            f"Joint '{joint.attrib['name']}' has no <limit lower= upper=> in the URDF"
        ) from error
