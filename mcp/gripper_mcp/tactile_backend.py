"""What a tactile source has to provide.

Deliberately separate from `GripperBackend`. The TSF-85 pads are their own
device with their own driver (`robotiq_tsf`), a bare 2F-85 has no pads at all,
and a simulator may model the fingers without modelling touch. Tactile is an
optional capability wired alongside a gripper, never a method every gripper
backend has to refuse.

Taxel counts are raw and uncalibrated, exactly as the sensor reports them. They
only mean something once a baseline is subtracted; that arithmetic lives in
`tactile.py`, in one place, so no source does it differently.
"""

from dataclasses import dataclass
from typing import Protocol, runtime_checkable


@dataclass(frozen=True)
class TactileLayout:
    rows: int
    cols: int
    pad_names: tuple[str, ...]

    @property
    def taxels_per_pad(self) -> int:
        return self.rows * self.cols


@dataclass(frozen=True)
class TactilePad:
    name: str
    taxels: tuple[int, ...]
    dynamic: int = 0


@dataclass(frozen=True)
class TactileReading:
    pads: tuple[TactilePad, ...]
    layout: TactileLayout


@runtime_checkable
class TactileBackend(Protocol):
    name: str

    def read_tactile(self) -> TactileReading: ...
