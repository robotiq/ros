import pytest

from gripper_mcp.backend import BackendMotion
from gripper_mcp.config import SPEC_DIR, GripperConfig, load_model_specs
from gripper_mcp.mock_backend import MockGripperBackend
from gripper_mcp.service import GripperService, UnknownGripperError, classify

NARROW = "robotiq_2f_85"
WIDE = "robotiq_2f_140"
ARM = "left"
CUBE_WIDTH_MM = 40.0
FULLY_OPEN_MM = 85.0
FULLY_CLOSED_MM = 0.0
HALF_OPEN_MM = 42.5
DATASHEET_EFFORT_N = 50.0
GENTLE_EFFORT_N = 20.0


def service_with(**backend_kwargs) -> tuple[GripperService, MockGripperBackend]:
    specs = load_model_specs(SPEC_DIR, {NARROW})
    backend = MockGripperBackend(sleep_fn=lambda _seconds: None, **backend_kwargs)
    service = GripperService(
        specs=specs,
        configs={ARM: GripperConfig(name=ARM, model=NARROW, description="left")},
        backends={ARM: backend},
    )
    return service, backend


def motion(**overrides) -> BackendMotion:
    fields = dict(final_position_rad=0.0, reached_goal=True, stalled=False)
    fields.update(overrides)
    return BackendMotion(**fields)


def test_a_fresh_gripper_reads_fully_open():
    service, _ = service_with()

    state = service.get_state(ARM)

    assert state.opening_mm == pytest.approx(FULLY_OPEN_MM)
    assert state.opening_fraction == pytest.approx(1.0)
    assert state.knuckle_rad == pytest.approx(0.0)


def test_an_unknown_gripper_is_named_with_the_available_ones():
    service, _ = service_with()

    with pytest.raises(UnknownGripperError, match=ARM):
        service.get_state("nonexistent")


def test_closing_on_empty_space_reaches_the_target():
    service, _ = service_with()

    result = service.move_to_opening(ARM, FULLY_CLOSED_MM)

    assert result.outcome == "reached"
    assert result.achieved_opening_mm == pytest.approx(FULLY_CLOSED_MM)


def test_the_state_follows_the_commanded_opening():
    service, _ = service_with()

    service.move_to_opening(ARM, HALF_OPEN_MM)

    assert service.get_state(ARM).opening_mm == pytest.approx(HALF_OPEN_MM)


def test_a_grasp_that_stops_on_an_object_is_a_success_with_reached_goal_false():
    service, _ = service_with(object_width_mm=CUBE_WIDTH_MM)

    result = service.grasp(ARM)

    assert result.outcome == "grasped"
    assert result.object_grasped is True
    assert result.reached_goal is False
    assert result.stalled is True
    assert result.achieved_opening_mm == pytest.approx(CUBE_WIDTH_MM)


def test_a_grasp_on_empty_space_closes_without_an_object():
    service, _ = service_with()

    result = service.grasp(ARM)

    assert result.outcome == "closed_without_object"
    assert result.object_grasped is False


def test_a_plain_close_that_stalls_is_unexpected():
    service, _ = service_with(object_width_mm=CUBE_WIDTH_MM)

    result = service.close_fully(ARM)

    assert result.outcome == "stalled_unexpectedly"
    assert result.object_grasped is None


def test_a_grasp_uses_the_datasheet_effort_unless_told_otherwise():
    service, backend = service_with(object_width_mm=CUBE_WIDTH_MM)

    service.grasp(ARM)
    default_force = backend.read_state().force_n
    service.open_fully(ARM)
    service.grasp(ARM, max_effort_n=GENTLE_EFFORT_N)

    assert default_force == DATASHEET_EFFORT_N
    assert backend.read_state().force_n == GENTLE_EFFORT_N


def test_an_opening_beyond_the_stroke_is_clamped():
    service, _ = service_with()

    result = service.move_to_opening(ARM, 500.0)

    assert result.commanded_opening_mm == pytest.approx(FULLY_OPEN_MM)


def test_listed_grippers_report_their_backend():
    service, _ = service_with()

    (entry,) = service.list_grippers()

    assert (entry.name, entry.model, entry.backend) == (ARM, NARROW, "mock")


def test_health_passes_the_backend_verdict_through():
    service, _ = service_with()

    health = service.get_health(ARM)

    assert health.reachable is True
    assert health.controller_active is True
    assert health.backend == "mock"


def test_each_gripper_opens_to_its_own_model_width():
    specs = load_model_specs(SPEC_DIR, {NARROW, WIDE})
    configs = {
        "narrow": GripperConfig(name="narrow", model=NARROW),
        "wide": GripperConfig(name="wide", model=WIDE),
    }
    backends = {
        name: MockGripperBackend(
            sleep_fn=lambda _seconds: None, geometry=specs[config.model].geometry
        )
        for name, config in configs.items()
    }
    service = GripperService(specs=specs, configs=configs, backends=backends)

    assert service.open_fully("narrow").commanded_opening_mm == pytest.approx(85.0)
    assert service.open_fully("wide").commanded_opening_mm == pytest.approx(140.0)


@pytest.mark.parametrize(
    ("result", "stopped_on_object", "is_grasp", "outcome"),
    [
        (motion(refused=True), False, True, "refused"),
        (motion(reached_goal=False, timed_out=True), False, True, "incomplete"),
        (motion(reached_goal=False, stalled=True), True, True, "grasped"),
        (
            motion(reached_goal=False, stalled=True),
            False,
            True,
            "closed_without_object",
        ),
        (motion(reached_goal=True), False, True, "closed_without_object"),
        (motion(reached_goal=False, stalled=True), True, False, "stalled_unexpectedly"),
        (motion(reached_goal=True), False, False, "reached"),
        (motion(reached_goal=False), False, False, "incomplete"),
    ],
)
def test_classify_covers_every_way_a_motion_can_end(
    result, stopped_on_object, is_grasp, outcome
):
    assert classify(result, stopped_on_object, is_grasp) == outcome
