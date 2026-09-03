"""Pure-Python tactile source, the CI default alongside the mock gripper.

Pressure is derived from how far the fingers have closed past a virtual object,
the same device the mock gripper uses to decide when to stall. Nothing here
models the sensor's physics; it models its behaviour well enough to drive and
test a contact loop: no signal until contact, a rise that grows with
penetration, a centre that reads harder than the edges, and saturation.

Its `object_width_mm` is configured independently of the mock gripper's. That
is the point: a gripper that stalls while the pads stay quiet is the "stopped on
something outside the pads" case, and it has to be reachable in a test.
"""

from typing import Callable

from gripper_mcp.tactile_backend import TactileLayout, TactilePad, TactileReading

TSF_85_LAYOUT = TactileLayout(rows=7, cols=4, pad_names=("left", "right"))

REST_COUNTS = 9
TAXEL_MAX_COUNTS = 110
STIFFNESS_COUNTS_PER_MM = 6.9


class MockTactileBackend:
    name = "mock"

    def __init__(
        self,
        read_opening_mm: Callable[[], float],
        object_width_mm: float | None = None,
        layout: TactileLayout = TSF_85_LAYOUT,
        rest_counts: int = REST_COUNTS,
        stiffness_counts_per_mm: float = STIFFNESS_COUNTS_PER_MM,
    ) -> None:
        self._read_opening_mm = read_opening_mm
        self._object_width_mm = object_width_mm
        self._layout = layout
        self._rest_counts = rest_counts
        self._stiffness_counts_per_mm = stiffness_counts_per_mm
        self._weights = contact_patch_weights(layout.rows, layout.cols)

    def read_tactile(self) -> TactileReading:
        taxels = self._taxels_for(self._penetration_mm())

        return TactileReading(
            pads=tuple(
                TactilePad(name=name, taxels=taxels) for name in self._layout.pad_names
            ),
            layout=self._layout,
        )

    def _penetration_mm(self) -> float:
        if self._object_width_mm is None:
            return 0.0
        return max(0.0, self._object_width_mm - self._read_opening_mm())

    def _taxels_for(self, penetration_mm: float) -> tuple[int, ...]:
        rise = penetration_mm * self._stiffness_counts_per_mm

        return tuple(
            min(TAXEL_MAX_COUNTS, self._rest_counts + round(rise * weight))
            for weight in self._weights
        )


def contact_patch_weights(rows: int, cols: int) -> tuple[float, ...]:
    weights = [
        axis_weight(row, rows) * axis_weight(col, cols)
        for row in range(rows)
        for col in range(cols)
    ]
    peak = max(weights)

    return tuple(weight / peak for weight in weights)


def axis_weight(index: int, count: int) -> float:
    centre = (count - 1) / 2.0

    return 1.0 - abs(index - centre) / count
