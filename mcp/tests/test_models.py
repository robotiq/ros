import pytest
from pydantic import ValidationError

from gripper_mcp.models import GripperMotionResult


def motion_result(**overrides):
    fields = dict(
        robot_name="left",
        commanded_opening_mm=0.0,
        achieved_opening_mm=31.2,
        reached_goal=False,
        stalled=True,
        object_grasped=True,
        outcome="grasped",
        detail="stalled on object",
        backend="ros",
    )
    fields.update(overrides)
    return GripperMotionResult(**fields)


def test_a_result_names_the_backend_it_came_from():
    with pytest.raises(ValidationError):
        motion_result(backend="sdk")


def test_a_result_only_reports_known_outcomes():
    with pytest.raises(ValidationError):
        motion_result(outcome="probably_fine")


def test_a_result_survives_the_wire():
    result = motion_result()

    assert GripperMotionResult.model_validate_json(result.model_dump_json()) == result
