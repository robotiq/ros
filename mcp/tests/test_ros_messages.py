from gripper_mcp.ros_messages import (
    STATUS_ABORTED,
    STATUS_CANCELED,
    STATUS_SUCCEEDED,
    advertised_type,
    cancel_note,
    motion_from_result,
    position_of,
    refused,
    timed_out,
)

ACTION = "/left/robotiq_gripper_controller/gripper_cmd"
PARALLEL = "control_msgs/action/ParallelGripperCommand"
HUMBLE = "control_msgs/action/GripperCommand"
KNOWN = {PARALLEL: object(), HUMBLE: object()}

HALFWAY_RAD = 0.4


def test_a_goal_that_succeeds_reached_its_position():
    motion = motion_from_result(
        STATUS_SUCCEEDED, HALFWAY_RAD, stalled=False, reached_goal=True
    )

    assert motion.reached_goal is True
    assert motion.stalled is False
    assert motion.final_position_rad == HALFWAY_RAD
    assert motion.timed_out is False


def test_an_aborted_goal_that_stalled_is_a_stall_not_a_failure():
    motion = motion_from_result(
        STATUS_ABORTED, HALFWAY_RAD, stalled=True, reached_goal=False
    )

    assert motion.stalled is True
    assert motion.reached_goal is False
    assert motion.refused is False
    assert motion.timed_out is False
    assert "aborted" in motion.detail


def test_a_succeeded_goal_that_stalled_is_still_a_stall():
    motion = motion_from_result(
        STATUS_SUCCEEDED, HALFWAY_RAD, stalled=True, reached_goal=False
    )

    assert motion.stalled is True
    assert motion.reached_goal is False


def test_a_canceled_goal_timed_out():
    motion = motion_from_result(
        STATUS_CANCELED, HALFWAY_RAD, stalled=False, reached_goal=False
    )

    assert motion.timed_out is True
    assert motion.stalled is False


def test_a_refusal_keeps_the_fingers_where_they_were():
    motion = refused(HALFWAY_RAD, "no server")

    assert motion.refused is True
    assert motion.final_position_rad == HALFWAY_RAD
    assert motion.detail == "no server"


def test_a_timeout_names_the_stage_and_the_budget():
    motion = timed_out(HALFWAY_RAD, 10.0, "result")

    assert motion.timed_out is True
    assert "result" in motion.detail
    assert "10.0 s" in motion.detail


def test_the_action_type_is_whatever_the_server_advertises():
    graph = [("/left/joint_states", ["sensor_msgs/msg/JointState"]), (ACTION, [HUMBLE])]

    assert advertised_type(graph, ACTION, KNOWN) == HUMBLE


def test_a_server_under_another_namespace_does_not_count():
    graph = [("/right/robotiq_gripper_controller/gripper_cmd", [PARALLEL])]

    assert advertised_type(graph, ACTION, KNOWN) is None


def test_an_unknown_action_type_is_not_picked():
    graph = [(ACTION, ["some_pkg/action/Other"])]

    assert advertised_type(graph, ACTION, KNOWN) is None


KNUCKLE_RAD = 0.4
FALLBACK_RAD = 9.0


class FakeState:
    def __init__(self, names, positions):
        self.name = names
        self.position = positions


def test_the_position_is_looked_up_by_joint_name_not_index():
    state = FakeState(["finger_joint", "knuckle_joint"], [0.1, KNUCKLE_RAD])

    assert position_of(state, "knuckle_joint", FALLBACK_RAD) == KNUCKLE_RAD


def test_a_state_without_the_joint_yields_the_fallback():
    state = FakeState(["finger_joint"], [0.1])

    assert position_of(state, "knuckle_joint", FALLBACK_RAD) == FALLBACK_RAD


class FakeCancelResponse:
    def __init__(self, return_code):
        self.return_code = return_code


def test_an_unanswered_cancel_warns_the_gripper_may_still_move():
    assert "unanswered" in cancel_note(None)
    assert "still be moving" in cancel_note(None)


def test_a_rejected_cancel_warns_the_gripper_may_still_move():
    assert "rejected" in cancel_note(FakeCancelResponse(return_code=1))


def test_an_accepted_cancel_says_so():
    assert cancel_note(FakeCancelResponse(return_code=0)) == " The goal was canceled."


def test_a_timeout_carries_the_cancel_note():
    motion = timed_out(HALFWAY_RAD, 1.0, "result", cancel_note(None))

    assert motion.detail.startswith("No result from the controller within 1.0 s.")
    assert motion.detail.endswith("may still be moving.")
