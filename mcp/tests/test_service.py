import pytest
from fakes.gripper import MockGripperBackend

from gripper_mcp.backend import BackendMotion
from gripper_mcp.config import SPEC_DIR, GripperConfig, load_model_specs
from gripper_mcp.service import (
    GripperService,
    UnknownGripperError,
    classify,
    stopped_on_something,
)
from gripper_mcp.units import Stroke

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
        configs={
            ARM: GripperConfig(
                name=ARM, model=NARROW, namespace="/left", description="left"
            )
        },
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


@pytest.mark.parametrize(
    "call",
    [
        GripperService.get_state,
        GripperService.open_fully,
        GripperService.close_fully,
        GripperService.get_health,
        lambda service, name: service.move_to_opening(name, HALF_OPEN_MM),
    ],
)
def test_an_unknown_gripper_is_named_with_the_available_ones_from_every_entry_point(
    call,
):
    service, _ = service_with()

    with pytest.raises(UnknownGripperError, match=ARM):
        call(service, "nonexistent")


def test_closing_on_empty_space_reaches_the_target():
    service, _ = service_with()

    result = service.move_to_opening(ARM, FULLY_CLOSED_MM)

    assert result.outcome == "reached"
    assert result.achieved_opening_mm == pytest.approx(FULLY_CLOSED_MM)


def test_the_state_follows_the_commanded_opening():
    service, _ = service_with()

    service.move_to_opening(ARM, HALF_OPEN_MM)

    assert service.get_state(ARM).opening_mm == pytest.approx(HALF_OPEN_MM)


def test_a_close_that_stops_on_an_object_reports_it_with_reached_goal_false():
    service, _ = service_with(object_width_mm=CUBE_WIDTH_MM)

    result = service.close_fully(ARM)

    assert result.outcome == "stopped_on_object"
    assert result.object_detected is True
    assert result.reached_goal is False
    assert result.stalled is True
    assert result.achieved_opening_mm == pytest.approx(CUBE_WIDTH_MM)


def test_a_close_on_empty_space_reaches_the_stop_with_nothing_detected():
    service, _ = service_with()

    result = service.close_fully(ARM)

    assert result.outcome == "reached"
    assert result.object_detected is False


def test_a_close_uses_the_datasheet_effort_unless_told_otherwise():
    service, backend = service_with(object_width_mm=CUBE_WIDTH_MM)

    service.close_fully(ARM)
    default_force = backend.read_state().force_n
    service.open_fully(ARM)
    service.close_fully(ARM, max_effort_n=GENTLE_EFFORT_N)

    assert default_force == DATASHEET_EFFORT_N
    assert backend.read_state().force_n == GENTLE_EFFORT_N


def test_an_opening_beyond_the_stroke_is_clamped():
    service, _ = service_with()

    result = service.move_to_opening(ARM, 500.0)

    assert result.commanded_opening_mm == pytest.approx(FULLY_OPEN_MM)


def test_listed_grippers_report_their_backend():
    service, _ = service_with()

    (entry,) = service.list_grippers()

    assert (entry.gripper_name, entry.model, entry.backend) == (ARM, NARROW, "mock")
    assert entry.max_opening_mm == pytest.approx(FULLY_OPEN_MM)


def test_health_passes_the_backend_verdict_through():
    service, _ = service_with()

    health = service.get_health(ARM)

    assert health.reachable is True
    assert health.controller_active is True
    assert health.backend == "mock"


def test_each_gripper_opens_to_its_own_model_width():
    specs = load_model_specs(SPEC_DIR, {NARROW, WIDE})
    configs = {
        "narrow": GripperConfig(name="narrow", model=NARROW, namespace="/narrow"),
        "wide": GripperConfig(name="wide", model=WIDE, namespace="/wide"),
    }
    backends = {
        name: MockGripperBackend(
            sleep_fn=lambda _seconds: None, stroke=specs[config.model].stroke
        )
        for name, config in configs.items()
    }
    service = GripperService(specs=specs, configs=configs, backends=backends)

    assert service.open_fully("narrow").commanded_opening_mm == pytest.approx(85.0)
    assert service.open_fully("wide").commanded_opening_mm == pytest.approx(140.0)


STROKE_2F_140 = Stroke(max_opening_mm=140.0, closed_tolerance_mm=1.5)
ONE_COUNT_2F_140_MM = 140.0 / 227
TSF_CLOSED_MM = 0.75


@pytest.mark.parametrize(
    ("result", "commanded_mm", "achieved_mm", "outcome"),
    [
        (motion(refused=True), 0.0, 85.0, "refused"),
        (motion(reached_goal=False, timed_out=True), 0.0, 60.0, "incomplete"),
        (motion(reached_goal=False, stalled=True), 0.0, 40.0, "stopped_on_object"),
        (motion(reached_goal=False, stalled=True), 0.0, 0.0, "reached"),
        (motion(reached_goal=True), 0.0, 0.0, "reached"),
        (motion(reached_goal=False, stalled=True), 85.0, 49.0, "stopped_on_object"),
        (motion(reached_goal=True), 30.0, 30.0, "reached"),
        (motion(reached_goal=False, stalled=True), 30.0, 30.0, "reached"),
    ],
    ids=[
        "refused",
        "timed out",
        "close stopped on something",
        "close met the stop, stalled",
        "close met the stop, reached",
        "open blocked part-way",
        "move landed",
        "move landed but the driver says stalled (#29)",
    ],
)
def test_classify_decides_from_position_not_the_stall_flag(
    result, commanded_mm, achieved_mm, outcome
):
    assert classify(result, commanded_mm, achieved_mm, STROKE_2F_140) == outcome


@pytest.mark.parametrize(
    ("achieved_mm", "stopped"),
    [
        (ONE_COUNT_2F_140_MM, False),
        (TSF_CLOSED_MM, False),
        (40.0, True),
    ],
    ids=[
        "a count short of the stop is an empty close",
        "thicker fingers meeting early is an empty close",
        "stopping well before the stop is an object",
    ],
)
def test_the_closed_tolerance_decides_an_empty_close(achieved_mm, stopped):
    assert stopped_on_something(0.0, achieved_mm, STROKE_2F_140) is stopped


def test_a_move_within_the_tolerance_reached_its_target():
    assert not stopped_on_something(30.0, 30.0 + ONE_COUNT_2F_140_MM, STROKE_2F_140)
    assert stopped_on_something(30.0, 32.0, STROKE_2F_140)


def test_an_out_of_range_request_says_so_in_the_detail():
    service, _ = service_with()

    result = service.move_to_opening(ARM, 500.0)

    assert result.detail.startswith("Requested 500.0 mm, clamped to 85.0 mm. ")


def test_an_in_range_request_keeps_the_backend_detail_verbatim():
    service, _ = service_with()

    result = service.move_to_opening(ARM, HALF_OPEN_MM)

    assert result.detail == "Reached the commanded position."
