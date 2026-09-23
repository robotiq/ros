"""Gripper backend on the ROS 2 driver, through rclpy.

One node per gripper, created inside the gripper's namespace so that
`robotiq_gripper_controller/gripper_cmd`, `joint_states` and `robot_description`
resolve to that gripper's controller. Each node spins on its own single-threaded
executor in a background thread; the MCP tools run in their own threads and
wait on futures. Single-threaded because rclpy's MultiThreadedExecutor takes a
whole core to follow the driver's 500 Hz joint_states, still drops most of them,
and starves the tool threads of the GIL. One per node because an executor serves
its nodes in no fixed order, and the pads' 2 kHz frames sharing one would, on some
starts, keep the gripper's own topics from ever being read.

The command joint's name and range are read from `robot_description`, the URDF
robot_state_publisher latches for the cell, so a prefixed cell resolves by itself
and nothing is copied from a datasheet. It is resolved on first use and kept.

The action type is whatever the running controller advertises for `gripper_cmd`:
`ParallelGripperCommand` from Jazzy's parallel_gripper_action_controller,
`GripperCommand` from Humble's position_controllers one. It is read off the graph
rather than guessed from the distro or from what imports, because recent
control_msgs releases ship both types on every distro.

Calls on one gripper are serialised: FastMCP runs tools on worker threads and an
agent may issue two moves at once, and two goals in flight on one controller
resolve in whatever order it likes. One deadline covers the whole call, from goal
response to result, so the tool-level ceiling is the datasheet's timeout. A goal
that outlives it is cancelled and the cancel's answer is reported, because a
rejected cancel means the gripper is still moving after the tool said it failed.

The effort is 0.0 to 1.0, the gripper's minimum to maximum force, the way the
gripper SDK commands it: the force register has no newton equivalent, since the
force applied depends on the speed, the fingers and the object. The controller's
goal still carries `max_effort` in nominal newtons and the driver divides it by
its `gripper_max_force` parameter, 235 by default, to fill the register, so the
effort is sent as effort x 235. Temporary: robotiq/ros#70 has the driver take
the SDK's effort directly, and this scaling goes with it.

A speed, when the call names one, goes on the goal as `command.velocity` in
m/s; the controller forwards it to the driver's `set_gripper_max_velocity`
interface and the driver maps it onto the gripper's speed register. Humble's
`GripperCommand` has no speed field, so a move that names a speed is refused
there before anything moves: running a "slowly" request at full speed is the
dangerous way to fail. A move that names none runs at the controller's own.

`holding_effort` is never filled. `joint_states.effort` would be a joint torque
in N m, and no shipped description exports an effort interface anyway; the
gripper reports motor current (gCU), and turning that into a fingertip force
needs a calibration nobody has done.
"""

import threading
import time
from collections.abc import Callable

import rclpy
from control_msgs.action import GripperCommand, ParallelGripperCommand
from rclpy.action import ActionClient
from rclpy.action.graph import get_action_names_and_types
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from gripper_mcp.backend import BackendHealth, BackendMotion, BackendState
from gripper_mcp.robot_description import command_joint
from gripper_mcp.units import JointGeometry
from gripper_mcp.ros_messages import (
    JOINT_STATES_TOPIC,
    advertised_type,
    cancel_note,
    motion_from_result,
    no_speed_message,
    no_state_message,
    position_of,
    refused,
    timed_out,
)

ACTION_NAME = "robotiq_gripper_controller/gripper_cmd"
ACTION_TYPES = {
    "control_msgs/action/ParallelGripperCommand": ParallelGripperCommand,
    "control_msgs/action/GripperCommand": GripperCommand,
}
DESCRIPTION_TOPIC = "robot_description"
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
DRIVER_FULL_SCALE_EFFORT_N = 235.0
SERVER_WAIT_S = 2.0
DESCRIPTION_WAIT_S = 2.0
STATE_WAIT_S = 2.0
STALE_STATE_S = 2.0
POLL_S = 0.05


class RosGraph:
    _lock = threading.Lock()
    _executors: list[SingleThreadedExecutor] = []

    @classmethod
    def node(cls, name: str, namespace: str) -> Node:
        with cls._lock:
            if not rclpy.ok():
                rclpy.init()
        return Node(name, namespace=namespace or "/")

    @classmethod
    def spin(cls, node: Node) -> None:
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        with cls._lock:
            cls._executors.append(executor)
        threading.Thread(
            target=executor.spin, name=f"rclpy-spin-{node.get_name()}", daemon=True
        ).start()

    @classmethod
    def shutdown(cls) -> None:
        with cls._lock:
            for executor in cls._executors:
                executor.shutdown()
            if cls._executors:
                rclpy.shutdown()
            cls._executors = []


def wait_for(future, timeout_s: float):
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    if not done.wait(timeout_s):
        return None
    return future.result()


def poll_until(probe: Callable[[], object], timeout_s: float):
    deadline = time.monotonic() + timeout_s
    while True:
        found = probe()
        if found is not None or time.monotonic() >= deadline:
            return found
        time.sleep(POLL_S)


def remaining(deadline: float) -> float:
    return max(0.0, deadline - time.monotonic())


def cancel_when_accepted(goal_future) -> None:
    def cancel(done) -> None:
        handle = done.result()
        if handle.accepted:
            handle.cancel_goal_async()

    goal_future.add_done_callback(cancel)


class JointStateFeed:
    def __init__(self, node: Node) -> None:
        self.latest: JointState | None = None
        self.latest_at = 0.0
        self._joint_name: str | None = None
        node.create_subscription(
            JointState, JOINT_STATES_TOPIC, self._on_joint_states, 10
        )

    def follow(self, joint_name: str) -> None:
        self._joint_name = joint_name

    def age_s(self) -> float | None:
        if self.latest is None:
            return None
        return time.monotonic() - self.latest_at

    def fresh(self) -> bool:
        age = self.age_s()
        return age is not None and age < STALE_STATE_S

    def _on_joint_states(self, message: JointState) -> None:
        if self._joint_name is not None and self._joint_name in message.name:
            self.latest_at = time.monotonic()
            self.latest = message


class RosGripperBackend:
    name = "ros"

    def __init__(self, gripper_name: str, namespace: str) -> None:
        self._node = RosGraph.node(f"gripper_mcp_{gripper_name}", namespace)
        self._description: str | None = None
        self._node.create_subscription(
            String, DESCRIPTION_TOPIC, self._on_description, LATCHED
        )
        self._states = JointStateFeed(self._node)
        self._joint: JointGeometry | None = None
        self._client: ActionClient | None = None
        self._action_type = None
        self._client_lock = threading.Lock()
        self._motion_lock = threading.Lock()
        RosGraph.spin(self._node)

    def joint_geometry(self) -> JointGeometry:
        if self._joint is None:
            if not self._await_description():
                raise RuntimeError(
                    f"No {DESCRIPTION_TOPIC} under '{self._node.get_namespace()}' "
                    f"within {DESCRIPTION_WAIT_S:.0f} s; is robot_state_publisher "
                    "running there?"
                )
            self._joint = command_joint(self._description)
            self._states.follow(self._joint.name)
        return self._joint

    def read_state(self) -> BackendState:
        joint = self.joint_geometry().name
        if not self._await_state():
            age = self._states.age_s()
            if age is None or age >= STALE_STATE_S:
                raise RuntimeError(
                    no_state_message(
                        age,
                        joint,
                        self._node.get_namespace(),
                        STATE_WAIT_S,
                        STALE_STATE_S,
                    )
                )
        return BackendState(position_rad=position_of(self._states.latest, joint, 0.0))

    def move_to(
        self,
        position_rad: float,
        effort: float,
        timeout_s: float,
        speed_m_s: float | None = None,
    ) -> BackendMotion:
        with self._motion_lock:
            return self._move_to(position_rad, effort, timeout_s, speed_m_s)

    def _move_to(
        self,
        position_rad: float,
        effort: float,
        timeout_s: float,
        speed_m_s: float | None,
    ) -> BackendMotion:
        deadline = time.monotonic() + timeout_s
        self.joint_geometry()
        here = self._position_or(position_rad)
        client = self._ready_client()
        if client is None:
            return refused(
                here,
                f"No {ACTION_NAME} action server under "
                f"'{self._node.get_namespace()}'; is the controller active?",
            )

        if speed_m_s is not None and self._action_type is not ParallelGripperCommand:
            return refused(here, no_speed_message(self._action_type.__name__))

        goal = self._goal(position_rad, effort, speed_m_s)
        sent = client.send_goal_async(goal)
        handle = wait_for(sent, remaining(deadline))
        if handle is None:
            cancel_when_accepted(sent)
            return timed_out(here, timeout_s, "goal response")
        if not handle.accepted:
            return refused(here, "The controller rejected the goal.")

        wrapped = wait_for(handle.get_result_async(), remaining(deadline))
        if wrapped is None:
            cancelled = wait_for(handle.cancel_goal_async(), STATE_WAIT_S)
            return timed_out(
                self._position_or(here), timeout_s, "result", cancel_note(cancelled)
            )

        here = self._position_or(here)
        return motion_from_result(
            wrapped.status,
            self._result_position(wrapped.result, here),
            wrapped.result.stalled,
            wrapped.result.reached_goal,
        )

    def health(self) -> BackendHealth:
        client = self._ready_client()
        state_fresh = self._joint_known() and self._await_state()
        return BackendHealth(
            reachable=client is not None or state_fresh,
            controller_active=client is not None,
            detail=(
                f"{ACTION_NAME} {self._describe(client)}, "
                f"{JOINT_STATES_TOPIC} {'fresh' if state_fresh else 'stale'}, "
                f"{DESCRIPTION_TOPIC} "
                f"{'received' if self._description else 'absent'} "
                f"under '{self._node.get_namespace()}'."
            ),
        )

    def _joint_known(self) -> bool:
        try:
            self.joint_geometry()
        except (RuntimeError, ValueError):
            return False
        return True

    def _on_description(self, message: String) -> None:
        self._description = message.data

    def _position_or(self, fallback: float) -> float:
        latest = self._states.latest
        if latest is None:
            return fallback
        return position_of(latest, self._joint.name, fallback)

    def _await_description(self) -> bool:
        return poll_until(lambda: self._description, DESCRIPTION_WAIT_S) is not None

    def _await_state(self) -> bool:
        return (
            poll_until(lambda: self._states.fresh() or None, STATE_WAIT_S) is not None
        )

    def _ready_client(self) -> ActionClient | None:
        with self._client_lock:
            if self._client is None:
                type_name = poll_until(self._advertised_type, SERVER_WAIT_S)
                if type_name is None:
                    return None
                self._action_type = ACTION_TYPES[type_name]
                self._client = ActionClient(self._node, self._action_type, ACTION_NAME)
        if not self._client.wait_for_server(timeout_sec=SERVER_WAIT_S):
            return None
        return self._client

    def _advertised_type(self) -> str | None:
        return advertised_type(
            get_action_names_and_types(self._node),
            self._resolved_action_name(),
            ACTION_TYPES,
        )

    def _resolved_action_name(self) -> str:
        return self._node.get_namespace().rstrip("/") + "/" + ACTION_NAME

    def _describe(self, client: ActionClient | None) -> str:
        if client is None:
            return "absent"
        return f"ready ({self._action_type.__name__})"

    def _goal(self, position_rad: float, effort: float, speed_m_s: float | None):
        goal = self._action_type.Goal()
        nominal_n = effort * DRIVER_FULL_SCALE_EFFORT_N
        if self._action_type is ParallelGripperCommand:
            goal.command.name = [self._joint.name]
            goal.command.position = [position_rad]
            goal.command.effort = [nominal_n]
            if speed_m_s is not None:
                goal.command.velocity = [speed_m_s]
        else:
            goal.command.position = position_rad
            goal.command.max_effort = nominal_n
        return goal

    def _result_position(self, result, fallback: float) -> float:
        if self._action_type is ParallelGripperCommand:
            return position_of(result.state, self._joint.name, fallback)
        return result.position
