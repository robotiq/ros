import pytest

from gripper_mcp.config import SPEC_DIR, GripperConfig, load_model_specs
from gripper_mcp.contact_grasp import grasp_until_contact
from gripper_mcp.mock_backend import MockGripperBackend
from gripper_mcp.mock_tactile_backend import MockTactileBackend
from gripper_mcp.service import GripperService
from gripper_mcp.tactile_service import TactileService, TactileUnavailableError

MODEL = "robotiq_2f_85"
ARM = "left"
FULLY_OPEN_MM = 85.0
FULLY_CLOSED_MM = 0.0
RIGID_CORE_MM = 40.0
SOFT_SURFACE_MM = 46.0
FIRM_THRESHOLD = 0.25
EXPIRED_TIMEOUT_S = 0.0
SPEC = load_model_specs(SPEC_DIR, {MODEL})[MODEL]


def services_with(
    gripper_object_mm: float | None,
    pad_object_mm: float | None,
    contact_timeout_s: float = SPEC.tactile.contact_timeout_s,
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
    tactile_spec = SPEC.tactile.model_copy(
        update={"contact_timeout_s": contact_timeout_s}
    )
    grippers = GripperService(
        specs={MODEL: SPEC},
        configs={ARM: GripperConfig(name=ARM, model=MODEL, tactile="mock")},
        backends={ARM: gripper},
        tactile_sources={ARM: pads.name},
    )
    return grippers, TactileService(grippers, {ARM: pads}, {ARM: tactile_spec})


def test_an_object_in_the_jaws_is_found_by_touch():
    grippers, tactile = services_with(RIGID_CORE_MM, SOFT_SURFACE_MM)

    result = grasp_until_contact(grippers, tactile, ARM)

    assert result.outcome == "contact_detected"
    assert result.object_grasped is True
    assert result.contact_signal >= result.threshold


def test_contact_stops_the_fingers_on_the_surface_not_the_core():
    grippers, tactile = services_with(RIGID_CORE_MM, SOFT_SURFACE_MM)

    result = grasp_until_contact(grippers, tactile, ARM)

    assert RIGID_CORE_MM < result.opening_mm <= SOFT_SURFACE_MM
    assert result.steps > 0


def test_a_firmer_threshold_closes_further():
    gentle = grasp_until_contact(*services_with(RIGID_CORE_MM, SOFT_SURFACE_MM), ARM)
    firm = grasp_until_contact(
        *services_with(RIGID_CORE_MM, SOFT_SURFACE_MM), ARM, threshold=FIRM_THRESHOLD
    )

    assert firm.outcome == "contact_detected"
    assert firm.opening_mm < gentle.opening_mm


def test_empty_jaws_close_without_contact():
    grippers, tactile = services_with(None, None)

    result = grasp_until_contact(grippers, tactile, ARM)

    assert result.outcome == "closed_without_contact"
    assert result.object_grasped is False
    assert result.opening_mm == pytest.approx(FULLY_CLOSED_MM)


def test_a_stall_with_quiet_pads_is_reported_not_trusted():
    grippers, tactile = services_with(RIGID_CORE_MM, None)

    result = grasp_until_contact(grippers, tactile, ARM)

    assert result.outcome == "stalled_before_contact"
    assert result.object_grasped is False
    assert result.opening_mm == pytest.approx(RIGID_CORE_MM)


def test_an_expired_timeout_stops_the_loop_before_it_moves():
    grippers, tactile = services_with(RIGID_CORE_MM, SOFT_SURFACE_MM, EXPIRED_TIMEOUT_S)

    result = grasp_until_contact(grippers, tactile, ARM)

    assert result.outcome == "incomplete"
    assert result.steps == 0
    assert result.opening_mm == pytest.approx(FULLY_OPEN_MM)


def test_the_result_names_both_sources():
    result = grasp_until_contact(*services_with(RIGID_CORE_MM, SOFT_SURFACE_MM), ARM)

    assert (result.backend, result.tactile_backend) == ("mock", "mock")


def test_a_gripper_without_pads_cannot_grasp_by_touch():
    grippers = GripperService(
        specs={MODEL: SPEC},
        configs={ARM: GripperConfig(name=ARM, model=MODEL)},
        backends={ARM: MockGripperBackend(sleep_fn=lambda _seconds: None)},
    )

    with pytest.raises(TactileUnavailableError, match=ARM):
        grasp_until_contact(grippers, TactileService(grippers, {}, {}), ARM)
