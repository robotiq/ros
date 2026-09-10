from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError

import gripper_mcp
from gripper_mcp.config import (
    GripperConfig,
    SPEC_DIR,
    load_gripper_configs,
    load_model_spec,
    load_model_specs,
    load_tactile_spec,
)

MCP_DIR = Path(__file__).resolve().parent.parent
EXAMPLE_WIRING = MCP_DIR / "grippers.yaml.example"

MODELS = {
    "robotiq_2f_85": (85.0, 20.0, 235.0),
    "robotiq_2f_140": (140.0, 10.0, 125.0),
}
TSF_85_TAXELS_PER_PAD = 28
TACTILE_SPEC_DIR = SPEC_DIR / "tactile"


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


def test_the_2f_85_datasheet_names_the_tsf_85_pads():
    spec = load_model_spec(SPEC_DIR / "robotiq_2f_85.yaml")

    assert spec.tactile_model == "robotiq_tsf_85"
    assert spec.tactile.layout.taxels_per_pad == TSF_85_TAXELS_PER_PAD
    assert spec.tactile.layout.pad_names == ("left", "right")
    assert 0.0 < spec.tactile.contact_threshold < 1.0
    assert spec.tactile.step_mm > 0.0
    assert spec.tactile.contact_timeout_s > 0.0


def test_the_2f_140_datasheet_has_no_tactile_option():
    spec = load_model_spec(SPEC_DIR / "robotiq_2f_140.yaml")

    assert spec.tactile_model is None
    assert spec.tactile is None


def test_every_shipped_tactile_datasheet_is_named_by_a_gripper():
    named = {load_model_spec(path).tactile_model for path in SPEC_DIR.glob("*.yaml")}

    assert {path.stem for path in TACTILE_SPEC_DIR.glob("*.yaml")} <= named


def test_a_tactile_block_inside_a_gripper_datasheet_is_rejected(tmp_path):
    inlined = datasheet_2f_85()
    inlined["tactile"] = yaml.safe_load(
        (TACTILE_SPEC_DIR / "robotiq_tsf_85.yaml").read_text()
    )

    with pytest.raises(ValidationError, match="tactile_model"):
        load_model_spec(write_datasheet(tmp_path, inlined))


def test_a_missing_tactile_datasheet_is_named(tmp_path):
    orphan = datasheet_2f_85()
    orphan["tactile_model"] = "robotiq_tsf_140"

    with pytest.raises(FileNotFoundError, match="robotiq_tsf_140"):
        load_model_spec(write_datasheet(tmp_path, orphan))


def test_the_example_wiring_loads_and_names_a_namespace_per_gripper():
    configs = load_gripper_configs(EXAMPLE_WIRING)

    assert {config.model for config in configs} <= set(MODELS)
    assert all(config.namespace.startswith("/") for config in configs)
    assert {config.tactile for config in configs} == {None, "ros"}


def test_a_mock_tactile_source_is_not_a_wiring_option(tmp_path):
    path = wiring_file(
        tmp_path,
        [
            {
                "name": "left",
                "model": "robotiq_2f_85",
                "namespace": "/left",
                "tactile": "mock",
            }
        ],
    )

    with pytest.raises(ValidationError, match="tactile"):
        load_gripper_configs(path)


def test_a_gripper_without_a_namespace_is_rejected(tmp_path):
    path = wiring_file(tmp_path, [{"name": "left", "model": "robotiq_2f_85"}])

    with pytest.raises(ValidationError, match="namespace"):
        load_gripper_configs(path)


def test_an_empty_namespace_is_rejected_by_gripper_name(tmp_path):
    wiring = tmp_path / "grippers.yaml"
    wiring.write_text("- {name: left, model: robotiq_2f_85, namespace: ''}\n")

    with pytest.raises(ValidationError, match="'left' has no namespace"):
        load_gripper_configs(wiring)


def test_a_mock_backend_is_no_longer_a_wiring_option(tmp_path):
    path = wiring_file(
        tmp_path,
        [{"name": "bench", "model": "robotiq_2f_85", "backend": "mock"}],
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
            {"name": "left", "model": "robotiq_2f_85", "namespace": "/left"},
            {"name": "left", "model": "robotiq_2f_140", "namespace": "/left"},
        ],
    )

    with pytest.raises(ValueError, match="left"):
        load_gripper_configs(path)


def test_a_missing_wiring_file_names_the_path(tmp_path):
    missing = tmp_path / "grippers.yaml"

    with pytest.raises(FileNotFoundError, match=str(missing.resolve())):
        load_gripper_configs(missing)


def test_a_tactile_datasheet_with_no_samples_to_average_is_rejected(tmp_path):
    sheet = tmp_path / "robotiq_tsf_85.yaml"
    sheet.write_text(
        (TACTILE_SPEC_DIR / "robotiq_tsf_85.yaml")
        .read_text()
        .replace("baseline_samples: 1000", "baseline_samples: 0")
    )

    with pytest.raises(ValidationError, match="baseline_samples"):
        load_tactile_spec(sheet)


def test_a_tactile_datasheet_whose_margin_would_lower_the_noise_is_rejected(tmp_path):
    sheet = tmp_path / "robotiq_tsf_85.yaml"
    sheet.write_text(
        (TACTILE_SPEC_DIR / "robotiq_tsf_85.yaml")
        .read_text()
        .replace("noise_margin: 2.0", "noise_margin: 0.5")
    )

    with pytest.raises(ValidationError, match="noise_margin"):
        load_tactile_spec(sheet)


def test_the_pads_default_to_the_gripper_namespace_unless_told_otherwise():
    same = GripperConfig(name="left", model="robotiq_2f_85", namespace="/left")
    apart = GripperConfig(
        name="left",
        model="robotiq_2f_85",
        namespace="/left",
        tactile="ros",
        tactile_namespace="/left_tsf",
    )

    assert same.tactile_namespace is None
    assert apart.tactile_namespace == "/left_tsf"
