"""Configuration loading: gripper model datasheets and per-cell wiring.

Two kinds of files, deliberately split:

- `robot_specifications/<model>.yaml`: one datasheet per Robotiq model, the same
  on every host. Adding a model is adding a file.
- `grippers.yaml`: the cell's wiring, which grippers exist, their model, backend
  and ROS namespace. Host-specific, so it ships as an example to copy.
"""

from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel

from gripper_mcp.tactile_backend import TactileLayout
from gripper_mcp.units import GripperGeometry

SPEC_DIR = Path(__file__).resolve().parent.parent / "robot_specifications"


class GripperConfig(BaseModel):
    name: str
    model: str
    backend: Literal["ros", "mock"] = "mock"
    namespace: str = ""
    description: str = ""
    object_width_mm: float | None = None
    tactile: Literal["mock"] | None = None


class Defaults(BaseModel):
    max_effort_n: float
    motion_timeout_s: float


class TactileSpec(BaseModel):
    rows: int
    cols: int
    pads: list[str]
    full_scale_counts: float
    contact_threshold: float
    baseline_samples: int

    @property
    def layout(self) -> TactileLayout:
        return TactileLayout(rows=self.rows, cols=self.cols, pad_names=tuple(self.pads))


class GripperModelSpec(BaseModel):
    model: str
    command_joint: str
    geometry: GripperGeometry
    defaults: Defaults
    tactile: TactileSpec | None = None


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
            f"Gripper wiring file not found: {path}. "
            "Copy grippers.yaml.example next to it and edit."
        )
    entries = yaml.safe_load(path.read_text()) or []
    configs = [GripperConfig.model_validate(entry) for entry in entries]

    names = [config.name for config in configs]
    duplicates = sorted({name for name in names if names.count(name) > 1})
    if duplicates:
        raise ValueError(f"Duplicate gripper names in {path}: {', '.join(duplicates)}")
    return configs
