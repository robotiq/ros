"""Close until the pads feel something: the one loop that issues repeated motion.

The gripper's own object detection is a motor stall, so it fires only on real
mechanical resistance; foam, a paper cup or a cable can be fully squashed while
the driver still reports nothing. The pads register first touch instead, so the
loop closes in small steps and reads them between steps.

It is the only place in the server that moves more than once per call, and it
ends on exactly one of: contact, fingers met, a stall, a refusal, or the
datasheet's hard timeout. The timeout is a safety limit, not a performance one:
a loop that can exit only on a sensor threshold is one bad threshold away from
closing all the way onto a part.
"""

import time
from dataclasses import dataclass

from gripper_mcp.models import (
    ContactGraspResult,
    ContactOutcome,
    GripperMotionResult,
    TactileReadingResult,
)
from gripper_mcp.service import GripperService
from gripper_mcp.tactile_service import TactileService

OUTCOME_DETAIL: dict[ContactOutcome, str] = {
    "contact_detected": "The pads registered contact; closing stopped.",
    "closed_without_contact": (
        "The fingers closed fully without the pads registering contact; "
        "nothing is held."
    ),
    "stalled_before_contact": (
        "The fingers stalled but the pads stayed quiet: obstructed outside the "
        "pads, or the sensor is not reporting. Do not treat this as a grasp."
    ),
    "incomplete": "The contact grasp hit its timeout; closing stopped where it was.",
    "refused": "The controller refused a closing step.",
}


@dataclass(frozen=True)
class ContactRun:
    limit: float
    step_mm: float
    deadline: float


def grasp_until_contact(
    grippers: GripperService,
    tactile: TactileService,
    robot_name: str,
    threshold: float | None = None,
    max_effort_n: float | None = None,
) -> ContactGraspResult:
    spec = tactile.spec_of(robot_name)
    run = ContactRun(
        limit=spec.contact_threshold if threshold is None else threshold,
        step_mm=spec.step_mm,
        deadline=time.monotonic() + spec.contact_timeout_s,
    )

    steps = 0
    reading = tactile.read(robot_name)
    outcome = stop_before_step(reading, grippers.fingers_met(robot_name), run)
    while outcome is None:
        target_mm = grippers.get_state(robot_name).opening_mm - run.step_mm
        motion = grippers.move_to_opening(robot_name, target_mm, max_effort_n)
        steps += 1
        reading = tactile.read(robot_name)
        outcome = stop_after_step(motion, reading, run) or stop_before_step(
            reading, grippers.fingers_met(robot_name), run
        )

    state = grippers.get_state(robot_name)
    return ContactGraspResult(
        robot_name=robot_name,
        outcome=outcome,
        object_grasped=outcome == "contact_detected",
        opening_mm=state.opening_mm,
        contact_signal=reading.contact_signal,
        threshold=run.limit,
        steps=steps,
        detail=OUTCOME_DETAIL[outcome],
        backend=state.backend,
        tactile_backend=reading.tactile_backend,
    )


def stop_before_step(
    reading: TactileReadingResult, fingers_met: bool, run: ContactRun
) -> ContactOutcome | None:
    if reading.contact_signal >= run.limit:
        return "contact_detected"
    if fingers_met:
        return "closed_without_contact"
    if time.monotonic() >= run.deadline:
        return "incomplete"
    return None


def stop_after_step(
    motion: GripperMotionResult, reading: TactileReadingResult, run: ContactRun
) -> ContactOutcome | None:
    if motion.outcome == "refused":
        return "refused"
    if motion.stalled:
        if reading.contact_signal >= run.limit:
            return "contact_detected"
        return "stalled_before_contact"
    return None
