"""Shaping robotiq_tsf messages into a TactileReading, with no ROS imports.

`TactileSensor/StaticData` carries `Taxels[2] taxels`, each `uint16[28] values`:
the two 7x4 pads as raw counts. `TactileSensor/Dynamic` carries `Data1[2] data`,
one int16 per pad. Counts stay raw, exactly as the driver reports them; the
baseline arithmetic lives in tactile.py and nowhere else.
"""

from gripper_mcp.tactile_backend import TactileLayout, TactilePad, TactileReading

STATIC_TOPIC = "TactileSensor/StaticData"
DYNAMIC_TOPIC = "TactileSensor/Dynamic"


class TactileMessageMismatch(Exception):
    pass


def reading_from_counts(
    pads_counts: list[list[int]], dynamics: list[int], layout: TactileLayout
) -> TactileReading:
    if len(pads_counts) != len(layout.pad_names):
        raise TactileMessageMismatch(
            f"{STATIC_TOPIC} carried {len(pads_counts)} pad(s), expected "
            f"{len(layout.pad_names)}. Wrong topic, or a message type mismatch."
        )
    for index, counts in enumerate(pads_counts):
        if len(counts) != layout.taxels_per_pad:
            raise TactileMessageMismatch(
                f"{STATIC_TOPIC} pad {index} carried {len(counts)} taxel(s), expected "
                f"{layout.taxels_per_pad} ({layout.rows}x{layout.cols}). The driver "
                "and the datasheet disagree on the sensor grid."
            )
    if dynamics and len(dynamics) != len(layout.pad_names):
        raise TactileMessageMismatch(
            f"{DYNAMIC_TOPIC} carried {len(dynamics)} value(s), expected one per pad."
        )

    padded_dynamics = dynamics or [0] * len(layout.pad_names)
    return TactileReading(
        pads=tuple(
            TactilePad(name=name, taxels=tuple(int(c) for c in counts), dynamic=dynamic)
            for name, counts, dynamic in zip(
                layout.pad_names, pads_counts, padded_dynamics
            )
        ),
        layout=layout,
    )
