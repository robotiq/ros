"""Configuration loading: gripper model datasheets and per-cell wiring.

Two kinds of files, deliberately split:

- `gripper_mcp/datasheets/<model>.yaml`: one datasheet per Robotiq
  model, shipped inside the package so an installed wheel finds it. It carries
  only what no robot description can tell us: the stroke in mm, the grip force
  the product is rated for, and timing defaults. The command joint's name and
  range come from the robot at runtime. Adding a model is adding a file.
- `grippers.yaml`: the cell's wiring, which grippers exist, their model and
  the ROS namespace their driver runs under. Host-specific, so it ships as an
  example to copy.

The only backend is the ROS driver. Running without hardware is the driver's
job too, on the SDK's fake gripper (`use_dummy`), so the wiring has no mock
entry to offer.

Every model refuses unknown keys: these files are hand-edited per host, and a
typo that silently dropped a field would point the server at the wrong robot.
"""

from importlib.resources import files
from pathlib import Path

import yaml
from pydantic import BaseModel, ConfigDict, model_validator

from gripper_mcp.units import Stroke

SPEC_DIR = Path(str(files("gripper_mcp").joinpath("datasheets")))


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid")


class GripperConfig(StrictModel):
    name: str
    model: str
    namespace: str
    description: str = ""

    @model_validator(mode="after")
    def _needs_a_namespace(self) -> "GripperConfig":
        if not self.namespace:
            raise ValueError(
                f"gripper '{self.name}' has no namespace; "
                "an empty namespace would address the root graph"
            )
        return self


class GripForce(StrictModel):
    min_n: float
    max_n: float

    @model_validator(mode="after")
    def _ordered(self) -> "GripForce":
        if self.max_n <= self.min_n:
            raise ValueError("max_n must exceed min_n")
        return self


class Defaults(StrictModel):
    max_effort_n: float
    motion_timeout_s: float


class GripperModelSpec(StrictModel):
    model: str
    stroke: Stroke
    grip_force: GripForce
    defaults: Defaults

    @model_validator(mode="after")
    def _default_effort_within_the_rated_range(self) -> "GripperModelSpec":
        effort = self.defaults.max_effort_n
        if not self.grip_force.min_n <= effort <= self.grip_force.max_n:
            raise ValueError(
                f"defaults.max_effort_n {effort} is outside the rated grip_force "
                f"{self.grip_force.min_n}-{self.grip_force.max_n} N"
            )
        return self


def load_model_spec(path: Path) -> GripperModelSpec:
    return GripperModelSpec.model_validate(yaml.safe_load(path.read_text()))


def load_model_specs(spec_dir: Path, models: set[str]) -> dict[str, GripperModelSpec]:
    specs = {}
    for model in sorted(models):
        path = spec_dir / f"{model}.yaml"
        if not path.exists():
            available = ", ".join(sorted(p.stem for p in spec_dir.glob("*.yaml")))
            raise FileNotFoundError(
                f"No datasheet for gripper model '{model}' (expected {path}). "
                f"Available models: {available or '(none)'}"
            )
        specs[model] = load_model_spec(path)
    return specs


def load_gripper_configs(path: Path) -> list[GripperConfig]:
    if not path.exists():
        raise FileNotFoundError(
            f"Gripper wiring file not found: {path.resolve()}. "
            "Copy grippers.yaml.example next to it and edit."
        )
    entries = yaml.safe_load(path.read_text()) or []
    if not isinstance(entries, list):
        raise ValueError(
            f"{path} must be a list of gripper entries, each starting with '- ' "
            f"(got a {type(entries).__name__})"
        )
    configs = [GripperConfig.model_validate(entry) for entry in entries]

    names = [config.name for config in configs]
    duplicates = sorted({name for name in names if names.count(name) > 1})
    if duplicates:
        raise ValueError(f"Duplicate gripper names in {path}: {', '.join(duplicates)}")
    return configs
