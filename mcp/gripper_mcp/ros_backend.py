"""Gripper backend on the ROS 2 driver, through rclpy.

One node per gripper, created inside the gripper's namespace so that
`robotiq_gripper_controller/gripper_cmd` and `joint_states` resolve to that
gripper's controller. All nodes share one executor spinning in a background
thread; the MCP tools run in their own threads and wait on futures.

The action type follows the distro the way the launch file does: Jazzy and later
ship `ParallelGripperCommand`, Humble only has `GripperCommand`. Whichever
imports is the one the controller speaks.
"""

import math
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from gripper_mcp.backend import BackendHealth, BackendMotion, BackendState
from gripper_mcp.ros_messages import motion_from_result, refused, timed_out

try:
    from control_msgs.action import ParallelGripperCommand as GripperAction

    PARALLEL_GRIPPER_ACTION = True
except ImportError:
    from control_msgs.action import GripperCommand as GripperAction

    PARALLEL_GRIPPER_ACTION = False

ACTION_NAME = "robotiq_gripper_controller/gripper_cmd"
JOINT_STATES_TOPIC = "joint_states"
SERVER_WAIT_S = 2.0
STATE_WAIT_S = 2.0
STALE_STATE_S = 2.0


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
        self._client = ActionClient(self._node, GripperAction, ACTION_NAME)
        executor.add_node(self._node)

    def read_state(self) -> BackendState:
        deadline = time.monotonic() + STATE_WAIT_S
        while self._latest is None and time.monotonic() < deadline:
            time.sleep(0.05)
        if self._latest is None:
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
        if not self._client.wait_for_server(timeout_sec=SERVER_WAIT_S):
            return refused(
                here,
                f"No {ACTION_NAME} action server under "
                f"'{self._node.get_namespace()}'; is the controller active?",
            )

        handle = wait_for(
            self._client.send_goal_async(self._goal(position_rad, max_effort_n)),
            timeout_s,
        )
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
        server_ready = self._client.wait_for_server(timeout_sec=SERVER_WAIT_S)
        state_fresh = time.monotonic() - self._latest_at < STALE_STATE_S
        return BackendHealth(
            reachable=server_ready or state_fresh,
            controller_active=server_ready,
            detail=(
                f"{ACTION_NAME} {'ready' if server_ready else 'absent'}, "
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

    def _goal(self, position_rad: float, max_effort_n: float):
        goal = GripperAction.Goal()
        if PARALLEL_GRIPPER_ACTION:
            goal.command.name = [self._joint]
            goal.command.position = [position_rad]
            goal.command.effort = [max_effort_n]
        else:
            goal.command.position = position_rad
            goal.command.max_effort = max_effort_n
        return goal

    def _result_position(self, result, fallback: float) -> float:
        if PARALLEL_GRIPPER_ACTION:
            return result.state.position[0] if result.state.position else fallback
        return result.position
