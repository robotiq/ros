"""Gripper backend on the ROS 2 driver, through rclpy.

One node per gripper, created inside the gripper's namespace so that
`robotiq_gripper_controller/gripper_cmd` and `joint_states` resolve to that
gripper's controller. All nodes share one executor spinning in a background
thread; the MCP tools run in their own threads and wait on futures.

The action type is whatever the running controller advertises for `gripper_cmd`:
`ParallelGripperCommand` from Jazzy's parallel_gripper_action_controller,
`GripperCommand` from Humble's position_controllers one. It is read off the graph
rather than guessed from the distro or from what imports, because recent
control_msgs releases ship both types on every distro.
"""

import math
import threading
import time

import rclpy
from control_msgs.action import GripperCommand, ParallelGripperCommand
from rclpy.action import ActionClient
from rclpy.action.graph import get_action_names_and_types
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from gripper_mcp.backend import BackendHealth, BackendMotion, BackendState
from gripper_mcp.ros_messages import (
    advertised_type,
    motion_from_result,
    refused,
    timed_out,
)

ACTION_NAME = "robotiq_gripper_controller/gripper_cmd"
ACTION_TYPES = {
    "control_msgs/action/ParallelGripperCommand": ParallelGripperCommand,
    "control_msgs/action/GripperCommand": GripperCommand,
}
JOINT_STATES_TOPIC = "joint_states"
SERVER_WAIT_S = 2.0
STATE_WAIT_S = 2.0
STALE_STATE_S = 2.0
POLL_S = 0.05


class RosGraph:
    _lock = threading.Lock()
    _executor: MultiThreadedExecutor | None = None

    @classmethod
    def executor(cls) -> MultiThreadedExecutor:
        with cls._lock:
            if cls._executor is None:
                rclpy.init()
                cls._executor = MultiThreadedExecutor()
                threading.Thread(
                    target=cls._executor.spin, name="rclpy-spin", daemon=True
                ).start()
            return cls._executor

    @classmethod
    def shutdown(cls) -> None:
        with cls._lock:
            if cls._executor is not None:
                cls._executor.shutdown()
                rclpy.shutdown()
                cls._executor = None


def wait_for(future, timeout_s: float):
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    if not done.wait(timeout_s):
        return None
    return future.result()


def measured_effort(efforts, index: int) -> float | None:
    if len(efforts) <= index or math.isnan(efforts[index]):
        return None
    return efforts[index]


class RosGripperBackend:
    name = "ros"

    def __init__(self, gripper_name: str, namespace: str, command_joint: str) -> None:
        self._joint = command_joint
        executor = RosGraph.executor()
        self._node = Node(f"gripper_mcp_{gripper_name}", namespace=namespace or "/")
        self._latest: JointState | None = None
        self._latest_at = 0.0
        self._node.create_subscription(
            JointState, JOINT_STATES_TOPIC, self._on_joint_states, 10
        )
        self._client: ActionClient | None = None
        self._action_type = None
        executor.add_node(self._node)

    def read_state(self) -> BackendState:
        if not self._await_state():
            raise RuntimeError(
                f"No {JOINT_STATES_TOPIC} under '{self._node.get_namespace()}' "
                f"within {STATE_WAIT_S:.0f} s; is the controller running?"
            )
        index = list(self._latest.name).index(self._joint)
        return BackendState(
            position_rad=self._latest.position[index],
            force_n=measured_effort(self._latest.effort, index),
        )

    def move_to(
        self, position_rad: float, max_effort_n: float, timeout_s: float
    ) -> BackendMotion:
        here = self._position_or(position_rad)
        client = self._ready_client()
        if client is None:
            return refused(
                here,
                f"No {ACTION_NAME} action server under "
                f"'{self._node.get_namespace()}'; is the controller active?",
            )

        goal = self._goal(position_rad, max_effort_n)
        handle = wait_for(client.send_goal_async(goal), timeout_s)
        if handle is None:
            return timed_out(here, timeout_s, "goal response")
        if not handle.accepted:
            return refused(here, "The controller rejected the goal.")

        wrapped = wait_for(handle.get_result_async(), timeout_s)
        if wrapped is None:
            handle.cancel_goal_async()
            return timed_out(self._position_or(here), timeout_s, "result")

        return motion_from_result(
            wrapped.status,
            self._result_position(wrapped.result, position_rad),
            wrapped.result.stalled,
            wrapped.result.reached_goal,
        )

    def health(self) -> BackendHealth:
        client = self._ready_client()
        state_fresh = (
            self._await_state() and time.monotonic() - self._latest_at < STALE_STATE_S
        )
        return BackendHealth(
            reachable=client is not None or state_fresh,
            controller_active=client is not None,
            detail=(
                f"{ACTION_NAME} {self._describe(client)}, "
                f"{JOINT_STATES_TOPIC} {'fresh' if state_fresh else 'stale'} "
                f"under '{self._node.get_namespace()}'."
            ),
        )

    def _on_joint_states(self, message: JointState) -> None:
        if self._joint in message.name:
            self._latest = message
            self._latest_at = time.monotonic()

    def _position_or(self, fallback: float) -> float:
        if self._latest is None:
            return fallback
        return self._latest.position[list(self._latest.name).index(self._joint)]

    def _await_state(self) -> bool:
        deadline = time.monotonic() + STATE_WAIT_S
        while self._latest is None and time.monotonic() < deadline:
            time.sleep(POLL_S)
        return self._latest is not None

    def _ready_client(self) -> ActionClient | None:
        if self._client is None:
            type_name = self._await_advertised_type()
            if type_name is None:
                return None
            self._action_type = ACTION_TYPES[type_name]
            self._client = ActionClient(self._node, self._action_type, ACTION_NAME)
        if not self._client.wait_for_server(timeout_sec=SERVER_WAIT_S):
            return None
        return self._client

    def _await_advertised_type(self) -> str | None:
        deadline = time.monotonic() + SERVER_WAIT_S
        while True:
            type_name = advertised_type(
                get_action_names_and_types(self._node),
                self._resolved_action_name(),
                ACTION_TYPES,
            )
            if type_name is not None or time.monotonic() >= deadline:
                return type_name
            time.sleep(POLL_S)

    def _resolved_action_name(self) -> str:
        return self._node.get_namespace().rstrip("/") + "/" + ACTION_NAME

    def _describe(self, client: ActionClient | None) -> str:
        if client is None:
            return "absent"
        return f"ready ({self._action_type.__name__})"

    def _goal(self, position_rad: float, max_effort_n: float):
        goal = self._action_type.Goal()
        if self._action_type is ParallelGripperCommand:
            goal.command.name = [self._joint]
            goal.command.position = [position_rad]
            goal.command.effort = [max_effort_n]
        else:
            goal.command.position = position_rad
            goal.command.max_effort = max_effort_n
        return goal

    def _result_position(self, result, fallback: float) -> float:
        if self._action_type is ParallelGripperCommand:
            return result.state.position[0] if result.state.position else fallback
        return result.position
