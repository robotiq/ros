import pytest

from gripper_mcp.config import SPEC_DIR, GripperConfig, load_model_specs
from gripper_mcp.mock_backend import MockGripperBackend
from gripper_mcp.mock_tactile_backend import REST_COUNTS, MockTactileBackend
from gripper_mcp.service import GripperService, UnknownGripperError
from gripper_mcp.tactile_service import (
    TactileService,
    TactileUnavailableError,
    judge_grasp,
)

MODEL = "robotiq_2f_85"
ARM = "left"
CUBE_WIDTH_MM = 40.0
FULLY_CLOSED_MM = 0.0
SPEC = load_model_specs(SPEC_DIR, {MODEL})[MODEL]


def services_with(
    gripper_object_mm: float | None, pad_object_mm: float | None
) -> tuple[GripperService, TactileService]:
    gripper = MockGripperBackend(
        object_width_mm=gripper_object_mm, sleep_fn=lambda _seconds: None
    )
    pads = MockTactileBackend(
        read_opening_mm=lambda: gripper.opening_mm_for(
            gripper.read_state().position_rad
        ),
        object_width_mm=pad_object_mm,
        layout=SPEC.tactile.layout,
    )
    grippers = GripperService(
        specs={MODEL: SPEC},
        configs={ARM: GripperConfig(name=ARM, model=MODEL, tactile="mock")},
        backends={ARM: gripper},
        tactile_sources={ARM: pads.name},
    )
    return grippers, TactileService(grippers, {ARM: pads}, {ARM: SPEC.tactile})


def services_without_pads() -> tuple[GripperService, TactileService]:
    grippers = GripperService(
        specs={MODEL: SPEC},
        configs={ARM: GripperConfig(name=ARM, model=MODEL)},
        backends={ARM: MockGripperBackend(sleep_fn=lambda _seconds: None)},
    )
    return grippers, TactileService(grippers, {}, {})


def test_taring_averages_the_datasheet_sample_count_at_rest():
    _, tactile = services_with(CUBE_WIDTH_MM, CUBE_WIDTH_MM)

    result = tactile.tare(ARM)

    assert result.samples == SPEC.tactile.baseline_samples
    assert result.rest_counts_mean == pytest.approx(REST_COUNTS)
    assert result.tactile_backend == "mock"


def test_an_open_gripper_reads_no_contact():
    _, tactile = services_with(CUBE_WIDTH_MM, CUBE_WIDTH_MM)

    reading = tactile.read(ARM)

    assert reading.contact is False
    assert reading.contact_signal == pytest.approx(0.0)
    assert reading.threshold == SPEC.tactile.contact_threshold


def test_a_grasp_on_the_cube_presses_both_pads():
    grippers, tactile = services_with(CUBE_WIDTH_MM, CUBE_WIDTH_MM)
    tactile.tare(ARM)

    grippers.grasp(ARM)
    reading = tactile.read(ARM)

    assert reading.contact is True
    assert set(reading.pad_signals) == set(SPEC.tactile.pads)
    assert all(signal > 0.0 for signal in reading.pad_signals.values())
    assert reading.peak_taxel_counts > 0.0


def test_verifying_after_a_grasp_on_the_cube_says_held():
    grippers, tactile = services_with(CUBE_WIDTH_MM, CUBE_WIDTH_MM)
    tactile.tare(ARM)

    grippers.grasp(ARM)
    verification = tactile.verify_grasp(ARM)

    assert verification.verdict == "held"
    assert verification.object_held is True
    assert verification.opening_mm == pytest.approx(CUBE_WIDTH_MM)


def test_verifying_an_open_gripper_says_no_contact():
    _, tactile = services_with(CUBE_WIDTH_MM, CUBE_WIDTH_MM)

    verification = tactile.verify_grasp(ARM)

    assert verification.verdict == "no_contact"
    assert verification.object_held is False


def test_verifying_after_closing_on_nothing_says_so():
    grippers, tactile = services_with(None, None)
    tactile.tare(ARM)

    grippers.grasp(ARM)
    verification = tactile.verify_grasp(ARM)

    assert verification.verdict == "closed_on_nothing"
    assert verification.opening_mm == pytest.approx(FULLY_CLOSED_MM)


def test_a_stall_outside_the_pads_is_not_a_hold():
    grippers, tactile = services_with(CUBE_WIDTH_MM, None)
    tactile.tare(ARM)

    grasp = grippers.grasp(ARM)
    verification = tactile.verify_grasp(ARM)

    assert grasp.outcome == "grasped"
    assert verification.verdict == "no_contact"


def test_the_first_read_takes_its_baseline_from_an_open_gripper():
    grippers, tactile = services_with(CUBE_WIDTH_MM, CUBE_WIDTH_MM)

    tactile.read(ARM)
    grippers.grasp(ARM)

    assert tactile.read(ARM).contact is True


def test_reading_with_the_fingers_closed_and_no_baseline_asks_for_a_tare():
    grippers, tactile = services_with(None, None)
    grippers.close_fully(ARM)

    with pytest.raises(TactileUnavailableError, match="gripper_tare_tactile"):
        tactile.read(ARM)


def test_a_gripper_without_pads_refuses_tactile_tools():
    _, tactile = services_without_pads()

    with pytest.raises(TactileUnavailableError, match=ARM):
        tactile.read(ARM)


def test_an_unknown_gripper_is_still_reported_as_unknown():
    _, tactile = services_with(None, None)

    with pytest.raises(UnknownGripperError):
        tactile.verify_grasp("nonexistent")


def test_the_listing_shows_which_grippers_have_pads():
    (with_pads,) = services_with(None, None)[0].list_grippers()
    (without_pads,) = services_without_pads()[0].list_grippers()

    assert with_pads.tactile == "mock"
    assert without_pads.tactile is None


@pytest.mark.parametrize(
    ("contact", "fingers_met", "verdict"),
    [
        (True, False, "held"),
        (False, False, "no_contact"),
        (True, True, "closed_on_nothing"),
        (False, True, "closed_on_nothing"),
    ],
)
def test_judge_grasp_covers_every_combination(contact, fingers_met, verdict):
    assert judge_grasp(contact, fingers_met) == verdict
