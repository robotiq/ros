"""What a finished `gripper_cmd` goal means, with no ROS imports so it is testable anywhere.

The action's terminal status and its result fields are folded into one
BackendMotion. The case that matters: with `allow_stalling: false` (the sim
controller configs) the controller ABORTS a goal that stalls, and the result
still says `stalled: true`. That abort is how the driver reports fingers stopped
on an object, so it comes through as a stall, not as a failure.
"""

from gripper_mcp.backend import BackendMotion

STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6

STATUS_NAMES = {
    STATUS_SUCCEEDED: "succeeded",
    STATUS_CANCELED: "canceled",
    STATUS_ABORTED: "aborted",
}


def motion_from_result(
    status: int, position_rad: float, stalled: bool, reached_goal: bool
) -> BackendMotion:
    name = STATUS_NAMES.get(status, f"status {status}")
    if status == STATUS_CANCELED:
        return BackendMotion(
            final_position_rad=position_rad,
            reached_goal=False,
            stalled=False,
            timed_out=True,
            detail="Goal canceled before it finished.",
        )
    return BackendMotion(
        final_position_rad=position_rad,
        reached_goal=reached_goal,
        stalled=stalled,
        detail=f"Goal {name}: stalled={stalled}, reached_goal={reached_goal}.",
    )


def refused(position_rad: float, detail: str) -> BackendMotion:
    return BackendMotion(
        final_position_rad=position_rad,
        reached_goal=False,
        stalled=False,
        refused=True,
        detail=detail,
    )


def timed_out(position_rad: float, timeout_s: float, stage: str) -> BackendMotion:
    return BackendMotion(
        final_position_rad=position_rad,
        reached_goal=False,
        stalled=False,
        timed_out=True,
        detail=f"No {stage} from the controller within {timeout_s:.1f} s.",
    )


def advertised_type(names_and_types, action_name: str, known: dict) -> str | None:
    for name, type_names in names_and_types:
        if name != action_name:
            continue
        for type_name in type_names:
            if type_name in known:
                return type_name
    return None
