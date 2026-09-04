from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError

from gripper_mcp.config import (
    SPEC_DIR,
    load_gripper_configs,
    load_model_spec,
    load_model_specs,
)

MCP_DIR = Path(__file__).resolve().parent.parent
EXAMPLE_WIRING = MCP_DIR / "grippers.yaml.example"

MODELS = {
    "robotiq_2f_85": ("robotiq_85_left_knuckle_joint", 85.0, 0.8),
    "robotiq_2f_140": ("finger_joint", 140.0, 0.7),
}
TSF_85_TAXELS_PER_PAD = 28


def wiring_file(tmp_path, entries):
    path = tmp_path / "grippers.yaml"
    path.write_text(yaml.safe_dump(entries))
    return path


@pytest.mark.parametrize("model", sorted(MODELS))
def test_each_datasheet_matches_the_urdf(model):
    joint, max_opening_mm, knuckle_rad_closed = MODELS[model]

    spec = load_model_spec(SPEC_DIR / f"{model}.yaml")

    assert spec.model == model
    assert spec.command_joint == joint
    assert spec.geometry.max_opening_mm == max_opening_mm
    assert spec.geometry.knuckle_rad_closed == knuckle_rad_closed


def test_every_shipped_datasheet_loads():
    models = {path.stem for path in SPEC_DIR.glob("*.yaml")}

    specs = load_model_specs(SPEC_DIR, models)

    assert set(specs) == set(MODELS)


def test_an_unknown_model_names_the_available_ones():
    with pytest.raises(FileNotFoundError, match="robotiq_2f_85"):
        load_model_specs(SPEC_DIR, {"robotiq_3f"})


def test_a_datasheet_missing_a_field_is_rejected(tmp_path):
    incomplete = yaml.safe_load((SPEC_DIR / "robotiq_2f_85.yaml").read_text())
    del incomplete["command_joint"]
    path = tmp_path / "robotiq_2f_85.yaml"
    path.write_text(yaml.safe_dump(incomplete))

    with pytest.raises(ValidationError, match="command_joint"):
        load_model_spec(path)


def test_the_2f_85_datasheet_describes_the_tsf_85_pads():
    spec = load_model_spec(SPEC_DIR / "robotiq_2f_85.yaml")

    assert spec.tactile.layout.taxels_per_pad == TSF_85_TAXELS_PER_PAD
    assert spec.tactile.layout.pad_names == ("left", "right")
    assert 0.0 < spec.tactile.contact_threshold < 1.0


def test_the_2f_140_datasheet_has_no_tactile_option():
    spec = load_model_spec(SPEC_DIR / "robotiq_2f_140.yaml")

    assert spec.tactile is None


def test_the_example_wiring_loads_and_covers_both_backends():
    configs = load_gripper_configs(EXAMPLE_WIRING)

    assert {config.backend for config in configs} == {"ros", "mock"}
    assert {config.model for config in configs} <= set(MODELS)
    assert {config.tactile for config in configs} == {None, "mock"}


def test_an_unknown_tactile_source_is_rejected(tmp_path):
    path = wiring_file(
        tmp_path, [{"name": "left", "model": "robotiq_2f_85", "tactile": "sdk"}]
    )

    with pytest.raises(ValidationError, match="tactile"):
        load_gripper_configs(path)


def test_a_minimal_entry_defaults_to_the_mock(tmp_path):
    path = wiring_file(tmp_path, [{"name": "left", "model": "robotiq_2f_85"}])

    (config,) = load_gripper_configs(path)

    assert config.backend == "mock"
    assert config.namespace == ""
    assert config.object_width_mm is None


def test_an_unknown_backend_is_rejected(tmp_path):
    path = wiring_file(
        tmp_path, [{"name": "left", "model": "robotiq_2f_85", "backend": "sdk"}]
    )

    with pytest.raises(ValidationError, match="backend"):
        load_gripper_configs(path)


def test_duplicate_gripper_names_are_rejected(tmp_path):
    path = wiring_file(
        tmp_path,
        [
            {"name": "left", "model": "robotiq_2f_85"},
            {"name": "left", "model": "robotiq_2f_140"},
        ],
    )

    with pytest.raises(ValueError, match="left"):
        load_gripper_configs(path)


def test_a_missing_wiring_file_names_the_path(tmp_path):
    missing = tmp_path / "grippers.yaml"

    with pytest.raises(FileNotFoundError, match=str(missing)):
        load_gripper_configs(missing)
