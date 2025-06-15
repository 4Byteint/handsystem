from gripper_sub.subscriber_member_function import pubsub
import rclpy
from rclpy.executors import MultiThreadedExecutor

from utils.table import (
    GripperState,
    ArmCmd,
    GripperInfomation,
    CanData,
    CanId,
    Device,
    Status,
)


def main(args=None):
    rclpy.init(args=args)

    pubsub_instance = pubsub()

    user_input = input(" enter a to directly power on, enter b to wait for ArmCmd: ")
    if user_input == "a":
        pubsub_instance.claw.state = GripperState.STATE_POWER_ON
    elif user_input == "b":
        pass

    # 使用 MultiThreadedExecutor 運行節點
    executor = MultiThreadedExecutor()
    executor.add_node(pubsub_instance)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        pubsub_instance.destroy_node()
        pubsub_instance.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
