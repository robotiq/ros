"""Shaping robotiq_tsf messages into a TactileReading, with no ROS imports.

`TactileSensor/StaticData` carries `Taxels[2] taxels`, each `uint16[28] values`:
the two 7x4 pads as raw counts. Counts stay raw, exactly as the driver reports
them; the baseline arithmetic lives in tactile.py and nowhere else.
"""

from gripper_mcp.tactile_backend import TactileLayout, TactilePad, TactileReading

STATIC_TOPIC = "TactileSensor/StaticData"


class TactileMessageMismatch(Exception):
    pass


def reading_from_counts(
    pads_counts: list[list[int]], layout: TactileLayout
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

    return TactileReading(
        pads=tuple(
            TactilePad(name=name, taxels=tuple(int(c) for c in counts))
            for name, counts in zip(layout.pad_names, pads_counts)
        ),
        layout=layout,
    )


def stale_frame_message(age_s: float, limit_s: float) -> str:
    return (
        f"The last {STATIC_TOPIC} frame is {age_s:.1f} s old (limit {limit_s:.0f} s); "
        "the robotiq_tsf driver has stopped publishing."
    )


def stalled_sample_message(collected: int, wanted: int, limit_s: float) -> str:
    return (
        f"No new {STATIC_TOPIC} frame for {limit_s:.0f} s after {collected} of "
        f"{wanted} samples; the robotiq_tsf driver has stopped publishing."
    )
