import pytest
from fakes.gripper import MockGripperBackend
from fakes.tactile import REST_COUNTS, MockTactileBackend

from gripper_mcp.config import SPEC_DIR, GripperConfig, load_model_specs
from gripper_mcp.service import GripperService, UnknownGripperError
from gripper_mcp.tactile_backend import TactilePad, TactileReading
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

BENCH_REST_COUNTS = 12540
BENCH_REST_NOISE_SIGNAL = 0.035
BENCH_EXTERNAL_HOLD_SIGNAL = 0.8


class ScriptedPads:
    name = "mock"

    def __init__(self, rest_frames: list[TactileReading]) -> None:
        self._rest_frames = rest_frames
        self.live: TactileReading = rest_frames[0]

    def read_tactile(self) -> TactileReading:
        return self.live

    def sample(self, count: int) -> list[TactileReading]:
        return [self._rest_frames[i % len(self._rest_frames)] for i in range(count)]


def uniform_frame(counts: int) -> TactileReading:
    layout = SPEC.tactile.layout
    return TactileReading(
        pads=tuple(
            TactilePad(name=name, taxels=(counts,) * layout.taxels_per_pad)
            for name in layout.pad_names
        ),
        layout=layout,
    )


def frame_at_signal(signal: float, above: TactileReading) -> TactileReading:
    taxel_count = sum(len(pad.taxels) for pad in above.pads)
    rise = round(signal * SPEC.tactile.full_scale_counts / taxel_count)
    return uniform_frame(above.pads[0].taxels[0] + rise)


def bench_services() -> tuple[GripperService, TactileService, ScriptedPads]:
    quiet = uniform_frame(BENCH_REST_COUNTS)
    loud = frame_at_signal(2 * BENCH_REST_NOISE_SIGNAL, quiet)
    pads = ScriptedPads([quiet, loud])
    gripper = MockGripperBackend(
        object_width_mm=CUBE_WIDTH_MM, sleep_fn=lambda _seconds: None
    )
    grippers = GripperService(
        specs={MODEL: SPEC},
        configs={
            ARM: GripperConfig(name=ARM, model=MODEL, namespace="/left", tactile="ros")
        },
        backends={ARM: gripper},
        tactile_sources={ARM: pads.name},
    )
    return grippers, TactileService(grippers, {ARM: (pads, SPEC.tactile)}), pads


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
        configs={
            ARM: GripperConfig(name=ARM, model=MODEL, namespace="/left", tactile="ros")
        },
        backends={ARM: gripper},
        tactile_sources={ARM: pads.name},
    )
    return grippers, TactileService(grippers, {ARM: (pads, SPEC.tactile)})


def services_without_pads() -> tuple[GripperService, TactileService]:
    grippers = GripperService(
        specs={MODEL: SPEC},
        configs={ARM: GripperConfig(name=ARM, model=MODEL, namespace="/left")},
        backends={ARM: MockGripperBackend(sleep_fn=lambda _seconds: None)},
    )
    return grippers, TactileService(grippers, {})


def test_taring_averages_the_datasheet_sample_count_at_rest():
    _, tactile = services_with(CUBE_WIDTH_MM, CUBE_WIDTH_MM)

    result = tactile.tare(ARM)

    assert result.samples == SPEC.tactile.baseline_samples
    assert result.rest_counts_mean == pytest.approx(REST_COUNTS)
    assert result.tactile_backend == "mock"


def test_quiet_pads_keep_the_datasheet_threshold():
    _, tactile = services_with(CUBE_WIDTH_MM, CUBE_WIDTH_MM)

    result = tactile.tare(ARM)

    assert result.noise_floor == pytest.approx(0.0)
    assert result.threshold == SPEC.tactile.contact_threshold


def test_noisy_pads_raise_the_threshold_above_their_own_noise():
    _, tactile, _ = bench_services()

    result = tactile.tare(ARM)

    assert result.noise_floor == pytest.approx(BENCH_REST_NOISE_SIGNAL, abs=0.002)
    assert result.threshold == pytest.approx(
        SPEC.tactile.noise_margin * BENCH_REST_NOISE_SIGNAL, abs=0.004
    )
    assert result.threshold > SPEC.tactile.contact_threshold


def test_an_empty_open_gripper_at_bench_noise_is_not_a_hold():
    _, tactile, pads = bench_services()
    tactile.tare(ARM)
    pads.live = frame_at_signal(
        BENCH_REST_NOISE_SIGNAL, uniform_frame(BENCH_REST_COUNTS)
    )

    reading = tactile.read(ARM)
    verification = tactile.verify_grasp(ARM)

    assert reading.contact is False
    assert verification.verdict == "no_contact"


def test_a_bench_external_hold_is_still_held():
    grippers, tactile, pads = bench_services()
    tactile.tare(ARM)
    grippers.close_fully(ARM)
    pads.live = frame_at_signal(
        BENCH_EXTERNAL_HOLD_SIGNAL, uniform_frame(BENCH_REST_COUNTS)
    )

    verification = tactile.verify_grasp(ARM)

    assert verification.verdict == "held"
    assert verification.contact_signal > verification.threshold


def test_an_open_gripper_reads_no_contact():
    _, tactile = services_with(CUBE_WIDTH_MM, CUBE_WIDTH_MM)

    reading = tactile.read(ARM)

    assert reading.contact is False
    assert reading.contact_signal == pytest.approx(0.0)
    assert reading.threshold == SPEC.tactile.contact_threshold


def test_a_grasp_on_the_cube_presses_both_pads():
    grippers, tactile = services_with(CUBE_WIDTH_MM, CUBE_WIDTH_MM)
    tactile.tare(ARM)

    grippers.close_fully(ARM)
    reading = tactile.read(ARM)

    assert reading.contact is True
    assert set(reading.pad_signals) == set(SPEC.tactile.pads)
    assert all(signal > 0.0 for signal in reading.pad_signals.values())
    assert sum(reading.pad_signals.values()) == pytest.approx(
        reading.contact_signal * len(SPEC.tactile.pads), abs=1e-4
    )
    assert reading.peak_taxel_counts > 0.0


def test_verifying_after_a_grasp_on_the_cube_says_held():
    grippers, tactile = services_with(CUBE_WIDTH_MM, CUBE_WIDTH_MM)
    tactile.tare(ARM)

    grippers.close_fully(ARM)
    verification = tactile.verify_grasp(ARM)

    assert verification.verdict == "held"
    assert verification.object_held is True
    assert verification.opening_mm == pytest.approx(CUBE_WIDTH_MM)


def test_a_verification_reads_the_gripper_state_once():
    grippers, tactile = services_with(CUBE_WIDTH_MM, CUBE_WIDTH_MM)
    reads = []
    get_state = grippers.get_state
    grippers.get_state = lambda name: reads.append(name) or get_state(name)

    verification = tactile.verify_grasp(ARM)

    assert verification.verdict == "no_contact"
    assert reads == [ARM]


def test_verifying_an_open_gripper_says_no_contact():
    _, tactile = services_with(CUBE_WIDTH_MM, CUBE_WIDTH_MM)

    verification = tactile.verify_grasp(ARM)

    assert verification.verdict == "no_contact"
    assert verification.object_held is False


def test_verifying_after_closing_on_nothing_says_so():
    grippers, tactile = services_with(None, None)
    tactile.tare(ARM)

    grippers.close_fully(ARM)
    verification = tactile.verify_grasp(ARM)

    assert verification.verdict == "closed_on_nothing"
    assert verification.opening_mm == pytest.approx(FULLY_CLOSED_MM)


def test_a_stall_outside_the_pads_is_not_a_hold():
    grippers, tactile = services_with(CUBE_WIDTH_MM, None)
    tactile.tare(ARM)

    close = grippers.close_fully(ARM)
    verification = tactile.verify_grasp(ARM)

    assert close.outcome == "stopped_on_object"
    assert verification.verdict == "no_contact"


def test_the_first_read_takes_its_baseline_from_an_open_gripper():
    grippers, tactile = services_with(CUBE_WIDTH_MM, CUBE_WIDTH_MM)

    tactile.read(ARM)
    grippers.close_fully(ARM)

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
