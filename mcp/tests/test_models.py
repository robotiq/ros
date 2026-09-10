from datetime import datetime, timezone

import pytest
from pydantic import ValidationError

from gripper_mcp.models import GripperMotionResult, GripperState


def motion_result(**overrides):
    fields = dict(
        gripper_name="left",
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


def test_a_state_keeps_its_timezone_across_the_wire():
    state = GripperState(
        gripper_name="left",
        opening_mm=42.5,
        opening_fraction=0.5,
        knuckle_rad=0.4,
        backend="mock",
        measured_at=datetime(2026, 9, 8, 14, 0, tzinfo=timezone.utc),
    )

    parsed = GripperState.model_validate_json(state.model_dump_json())

    assert parsed.measured_at == state.measured_at
    assert parsed.measured_at.tzinfo is not None
