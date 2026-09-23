"""Tactile readings from a TSF-85 through this repo's robotiq_tsf driver.

One node per gripper, in the namespace the wiring gives the pads (the gripper's
own unless `tactile_namespace` says otherwise), so `TactileSensor/StaticData`
resolves to the pads on that gripper. It shares the executor thread with the
gripper backends (RosGraph).

The driver publishes with the sensor-data QoS (best effort), so the
subscription must too: a reliable subscriber never matches a best-effort
publisher and simply receives nothing, with no error anywhere.

Reads return the latest message received and never block on the topic rate:
the sensor publishes at about 2 kHz, and a contact loop reads far more often
than it moves. Because of that, a read remembers when the last frame arrived
and refuses to answer from a frame older than `STALE_FRAME_S`; otherwise a
driver that stopped publishing would keep reporting its last frame forever,
and a frozen "no contact" reads as "keep closing" to a tactile-guided close.

`sample` is the exception: the subscription callback keeps every frame that
arrives during the tare window while the caller sleeps through it, so each
frame is a distinct sensor update and the two threads never hand off per
frame. The window is a duration rather than a frame count because the frame
rate is the driver's to choose. A window that received no frame at all means
the driver has stopped publishing.
"""

import time

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robotiq_tsf.msg import StaticData

from gripper_mcp.ros_backend import RosGraph, poll_until
from gripper_mcp.ros_tactile_messages import (
    STATIC_TOPIC,
    reading_from_counts,
    stale_frame_message,
    stalled_sample_message,
)
from gripper_mcp.tactile_backend import TactileLayout, TactileReading

STATIC_WAIT_S = 2.0
STALE_FRAME_S = 2.0


class RosTactileBackend:
    name = "ros"

    def __init__(
        self, gripper_name: str, namespace: str, layout: TactileLayout
    ) -> None:
        self._layout = layout
        executor = RosGraph.executor()
        self._node = Node(
            f"gripper_mcp_tactile_{gripper_name}", namespace=namespace or "/"
        )
        self._static: StaticData | None = None
        self._latest_at = 0.0
        self._collecting = False
        self._collected: list[StaticData] = []
        self._node.create_subscription(
            StaticData, STATIC_TOPIC, self._on_static, qos_profile_sensor_data
        )
        executor.add_node(self._node)

    def read_tactile(self) -> TactileReading:
        static = poll_until(lambda: self._static, STATIC_WAIT_S)
        if static is None:
            raise RuntimeError(
                f"No {STATIC_TOPIC} under '{self._node.get_namespace()}' within "
                f"{STATIC_WAIT_S:.0f} s; is the robotiq_tsf driver running?"
            )
        age_s = time.monotonic() - self._latest_at
        if age_s > STALE_FRAME_S:
            raise RuntimeError(stale_frame_message(age_s, STALE_FRAME_S))
        return self._reading(static)

    def sample(self, duration_s: float) -> list[TactileReading]:
        self._collected = []
        self._collecting = True
        time.sleep(duration_s)
        self._collecting = False
        frames = list(self._collected)
        if not frames:
            raise RuntimeError(stalled_sample_message(duration_s))
        return [self._reading(message) for message in frames]

    def _reading(self, static: StaticData) -> TactileReading:
        return reading_from_counts(
            [list(pad.values) for pad in static.taxels], self._layout
        )

    def _on_static(self, message: StaticData) -> None:
        self._static = message
        self._latest_at = time.monotonic()
        if self._collecting:
            self._collected.append(message)
