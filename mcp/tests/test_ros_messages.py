from gripper_mcp.ros_messages import (
    STATUS_ABORTED,
    STATUS_CANCELED,
    STATUS_SUCCEEDED,
    motion_from_result,
    refused,
    timed_out,
)

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
