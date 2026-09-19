# What the gripper reports about itself, decoded. The driver reads these out of
# the status block every exchange cycle; a broadcaster republishes them here so
# that a client off the controller manager can read them without re-deriving
# anything from a register.

std_msgs/Header header

# gOBJ. The gripper's own answer to "am I holding something", and the only
# authoritative one: a stall flag or a position comparison is a guess at this.
uint8 MOVING=0
uint8 DETECTED_WHILE_OPENING=1
uint8 DETECTED_WHILE_CLOSING=2
uint8 AT_REQUESTED_POSITION=3
uint8 object_detection

# gCU in amperes. Motor current, not grip force, and not convertible to one.
float64 motor_current

# gFLT, as the instruction manual numbers it. The register is four bits wide, so
# a code the manual does not list can still arrive: treat it as an undefined
# fault, which gripper_fault_severity reports as major.
#
# UNKNOWN is outside the register, and says the gripper did not report a fault
# this cycle, either because the hardware exports no fault interface or because
# the reading failed. It is not NONE: an unfaulted gripper and a gripper that
# is not reporting a fault state are different, and only one of them is safe to act on.
uint8 GRIPPER_FAULT_UNKNOWN=255
uint8 GRIPPER_FAULT_NONE=0
uint8 GRIPPER_FAULT_ACTION_DELAYED=5
uint8 GRIPPER_FAULT_ACTIVATION_REQUIRED=7
uint8 GRIPPER_FAULT_OVER_TEMPERATURE=8
uint8 GRIPPER_FAULT_NO_COMMUNICATION=9
uint8 GRIPPER_FAULT_UNDER_VOLTAGE=10
uint8 GRIPPER_FAULT_AUTOMATIC_RELEASE_IN_PROGRESS=11
uint8 GRIPPER_FAULT_INTERNAL_FAULT=12
uint8 GRIPPER_FAULT_ACTIVATION_FAULT=13
uint8 GRIPPER_FAULT_OVERCURRENT=14
uint8 GRIPPER_FAULT_AUTOMATIC_RELEASE_COMPLETE=15
uint8 gripper_fault 255

# How bad the gripper fault is. A major fault clears only on a reactivation;
# the manual calls the warning tier "priority faults". UNKNOWN accompanies
# GRIPPER_FAULT_UNKNOWN: the two fields are exported together or not at all.
uint8 GRIPPER_FAULT_SEVERITY_UNKNOWN=255
uint8 GRIPPER_FAULT_SEVERITY_NONE=0
uint8 GRIPPER_FAULT_SEVERITY_WARNING=1
uint8 GRIPPER_FAULT_SEVERITY_MINOR=2
uint8 GRIPPER_FAULT_SEVERITY_MAJOR=3
uint8 gripper_fault_severity 255
