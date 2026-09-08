from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError

import gripper_mcp
from gripper_mcp.config import (
    SPEC_DIR,
    load_gripper_configs,
    load_model_spec,
    load_model_specs,
)

MCP_DIR = Path(__file__).resolve().parent.parent
EXAMPLE_WIRING = MCP_DIR / "grippers.yaml.example"

MODELS = {
    "robotiq_2f_85": (85.0, 20.0, 235.0),
    "robotiq_2f_140": (140.0, 10.0, 125.0),
}


def wiring_file(tmp_path, entries):
    path = tmp_path / "grippers.yaml"
    path.write_text(yaml.safe_dump(entries))
    return path


def datasheet_2f_85():
    return yaml.safe_load((SPEC_DIR / "robotiq_2f_85.yaml").read_text())


def write_datasheet(tmp_path, datasheet):
    path = tmp_path / "robotiq_2f_85.yaml"
    path.write_text(yaml.safe_dump(datasheet))
    return path


@pytest.mark.parametrize("model", sorted(MODELS))
def test_each_datasheet_matches_the_product_sheet(model):
    max_opening_mm, min_force_n, max_force_n = MODELS[model]

    spec = load_model_spec(SPEC_DIR / f"{model}.yaml")

    assert spec.model == model
    assert spec.stroke.max_opening_mm == max_opening_mm
    assert spec.stroke.min_opening_mm == 0.0
    assert (spec.grip_force.min_n, spec.grip_force.max_n) == (min_force_n, max_force_n)


def test_the_datasheets_ship_inside_the_package():
    package_dir = Path(gripper_mcp.__file__).resolve().parent

    assert SPEC_DIR.is_relative_to(package_dir)
    assert set(SPEC_DIR.glob("*.yaml"))


def test_every_shipped_datasheet_loads():
    models = {path.stem for path in SPEC_DIR.glob("*.yaml")}

    specs = load_model_specs(SPEC_DIR, models)

    assert set(specs) == set(MODELS)


def test_an_unknown_model_names_the_available_ones():
    with pytest.raises(FileNotFoundError, match="robotiq_2f_85"):
        load_model_specs(SPEC_DIR, {"robotiq_3f"})


def test_a_datasheet_missing_a_field_is_rejected(tmp_path):
    incomplete = datasheet_2f_85()
    del incomplete["stroke"]

    with pytest.raises(ValidationError, match="stroke"):
        load_model_spec(write_datasheet(tmp_path, incomplete))


def test_a_datasheet_no_longer_carries_the_urdf_joint(tmp_path):
    stale = datasheet_2f_85()
    stale["command_joint"] = "robotiq_85_left_knuckle_joint"

    with pytest.raises(ValidationError, match="command_joint"):
        load_model_spec(write_datasheet(tmp_path, stale))


def test_a_default_effort_outside_the_rated_grip_force_is_rejected(tmp_path):
    too_strong = datasheet_2f_85()
    too_strong["defaults"]["max_effort_n"] = 300.0

    with pytest.raises(ValidationError, match="outside the rated grip_force"):
        load_model_spec(write_datasheet(tmp_path, too_strong))


def test_the_example_wiring_loads_and_covers_both_backends():
    configs = load_gripper_configs(EXAMPLE_WIRING)

    assert {config.backend for config in configs} == {"ros", "mock"}
    assert {config.model for config in configs} <= set(MODELS)


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


def test_a_misspelled_key_is_rejected_by_name(tmp_path):
    path = wiring_file(
        tmp_path, [{"name": "left", "model": "robotiq_2f_85", "namespcae": "/left"}]
    )

    with pytest.raises(ValidationError, match="namespcae"):
        load_gripper_configs(path)


def test_a_single_entry_written_without_the_list_dash_is_explained(tmp_path):
    path = tmp_path / "grippers.yaml"
    path.write_text("name: left\nmodel: robotiq_2f_85\n")

    with pytest.raises(ValueError, match="list of gripper entries"):
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
