class GripperState:
    """state"""

    POWER_ON = -1  #:
    CONNECTION_CHECKING = -2
    GRIPPER_RELEASING = -3
    GRIPPER_HOLDING = -4


class ArmCmd:
    """cmd"""

    RELEASE_CMD = -1  #:
    HOLD_CMD = -2
