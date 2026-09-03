import pytest

from gripper_mcp.backend import GripperBackend
from gripper_mcp.mock_backend import MOCK_GEOMETRY, MockGripperBackend

CLOSED = MOCK_GEOMETRY.knuckle_rad_closed
OPEN = MOCK_GEOMETRY.knuckle_rad_open
MAX_EFFORT_N = 50.0
GENEROUS_TIMEOUT_S = 30.0
CUBE_WIDTH_MM = 40.0


def instant_mock(**kwargs) -> MockGripperBackend:
    return MockGripperBackend(sleep_fn=lambda _seconds: None, **kwargs)


def test_the_mock_satisfies_the_backend_protocol():
    assert isinstance(instant_mock(), GripperBackend)


def test_a_fresh_mock_starts_open():
    assert instant_mock().read_state().position_rad == OPEN


def test_closing_on_empty_space_reaches_the_goal():
    motion = instant_mock().move_to(CLOSED, MAX_EFFORT_N, GENEROUS_TIMEOUT_S)

    assert motion.reached_goal is True
    assert motion.stalled is False
    assert motion.final_position_rad == CLOSED


def test_closing_on_a_virtual_object_stalls():
    backend = instant_mock(object_width_mm=CUBE_WIDTH_MM)

    motion = backend.move_to(CLOSED, MAX_EFFORT_N, GENEROUS_TIMEOUT_S)

    assert motion.stalled is True
    assert motion.reached_goal is False


def test_a_stall_stops_at_the_object_width():
    backend = instant_mock(object_width_mm=CUBE_WIDTH_MM)

    motion = backend.move_to(CLOSED, MAX_EFFORT_N, GENEROUS_TIMEOUT_S)

    assert backend.opening_mm_for(motion.final_position_rad) == pytest.approx(
        CUBE_WIDTH_MM
    )


def test_a_stall_reports_the_commanded_effort_as_grip_force():
    backend = instant_mock(object_width_mm=CUBE_WIDTH_MM)

    backend.move_to(CLOSED, MAX_EFFORT_N, GENEROUS_TIMEOUT_S)

    assert backend.read_state().force_n == MAX_EFFORT_N


def test_an_object_narrower_than_the_target_does_not_stall():
    backend = instant_mock(object_width_mm=10.0)

    motion = backend.move_to(
        backend.position_rad_for(50.0), MAX_EFFORT_N, GENEROUS_TIMEOUT_S
    )

    assert motion.reached_goal is True
    assert motion.stalled is False


def test_opening_after_a_grasp_releases_without_stalling():
    backend = instant_mock(object_width_mm=CUBE_WIDTH_MM)
    backend.move_to(CLOSED, MAX_EFFORT_N, GENEROUS_TIMEOUT_S)

    motion = backend.move_to(OPEN, MAX_EFFORT_N, GENEROUS_TIMEOUT_S)

    assert motion.reached_goal is True
    assert motion.final_position_rad == OPEN
    assert backend.read_state().force_n == 0.0


def test_a_target_past_the_stroke_is_clamped():
    motion = instant_mock().move_to(CLOSED + 1.0, MAX_EFFORT_N, GENEROUS_TIMEOUT_S)

    assert motion.final_position_rad == CLOSED


def test_a_move_slower_than_its_timeout_reports_timed_out_and_stays_put():
    backend = instant_mock(travel_speed_mm_s=100.0)

    motion = backend.move_to(CLOSED, MAX_EFFORT_N, timeout_s=0.001)

    assert motion.timed_out is True
    assert motion.reached_goal is False
    assert backend.read_state().position_rad == OPEN


def test_travel_time_grows_with_distance():
    slept: list[float] = []
    backend = MockGripperBackend(sleep_fn=slept.append)

    backend.move_to(backend.position_rad_for(60.0), MAX_EFFORT_N, GENEROUS_TIMEOUT_S)
    short = sum(slept)
    slept.clear()
    backend.move_to(CLOSED, MAX_EFFORT_N, GENEROUS_TIMEOUT_S)
    long = sum(slept)

    assert long > short
