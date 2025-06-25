from .subscriber_member_function import pubsub
import rclpy
from rclpy.executors import MultiThreadedExecutor
from .ros_socket_receive import SocketReceiver
    
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

    Pubsub_instance = pubsub()
    Socket_instance = SocketReceiver()
    user_input = input(" enter a to directly power on, enter b to wait for ArmCmd: \n")
    if user_input == "a":
        Pubsub_instance.claw.state = GripperState.STATE_POWER_ON
    elif user_input == "b":
        pass

    # 使用 MultiThreadedExecutor 運行節點
    executor = MultiThreadedExecutor()
    executor.add_node(Pubsub_instance)
    executor.add_node(Socket_instance)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        Pubsub_instance.destroy_node()
        Pubsub_instance.shutdown()
        Socket_instance.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
