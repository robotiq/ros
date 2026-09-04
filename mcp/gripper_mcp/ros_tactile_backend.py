"""Tactile readings from a TSF-85 through this repo's robotiq_tsf driver.

One node per gripper, in the gripper's namespace, so `TactileSensor/StaticData`
and `TactileSensor/Dynamic` resolve to the pads on that gripper. It shares the
executor thread with the gripper backends (RosGraph). Reads return the latest
message received, never block on the topic rate: the sensor publishes at 60 Hz,
and a contact loop reads far more often than it moves.
"""

import time

from rclpy.node import Node
from robotiq_tsf.msg import Dynamic, StaticData

from gripper_mcp.ros_backend import POLL_S, STATE_WAIT_S, RosGraph
from gripper_mcp.ros_tactile_messages import (
    DYNAMIC_TOPIC,
    STATIC_TOPIC,
    reading_from_counts,
)
from gripper_mcp.tactile_backend import TactileLayout, TactileReading


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
        self._dynamic: Dynamic | None = None
        self._node.create_subscription(StaticData, STATIC_TOPIC, self._on_static, 10)
        self._node.create_subscription(Dynamic, DYNAMIC_TOPIC, self._on_dynamic, 10)
        executor.add_node(self._node)

    def read_tactile(self) -> TactileReading:
        if not self._await_static():
            raise RuntimeError(
                f"No {STATIC_TOPIC} under '{self._node.get_namespace()}' within "
                f"{STATE_WAIT_S:.0f} s; is the robotiq_tsf driver running?"
            )
        dynamics = (
            [] if self._dynamic is None else [d.value for d in self._dynamic.data]
        )
        return reading_from_counts(
            [list(pad.values) for pad in self._static.taxels], dynamics, self._layout
        )

    def _on_static(self, message: StaticData) -> None:
        self._static = message

    def _on_dynamic(self, message: Dynamic) -> None:
        self._dynamic = message

    def _await_static(self) -> bool:
        deadline = time.monotonic() + STATE_WAIT_S
        while self._static is None and time.monotonic() < deadline:
            time.sleep(POLL_S)
        return self._static is not None
