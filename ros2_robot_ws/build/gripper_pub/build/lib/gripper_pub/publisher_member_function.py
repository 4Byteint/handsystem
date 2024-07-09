import rclpy
from rclpy.node import Node
from robot_interfaces.msg import GripperCommand, GripperInfo
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from gripper_sub.table import (
    GripperState,
    ArmCmd,
    GripperInfomation,
    CanData,
    CanId,
    Device,
    Status,
)


class GripperPublisher(Node):
    def __init__(self):
        super().__init__("gripper_publisher")

        # 使用 ReentrantCallbackGroup 來允許回調並行執行
        self.callback_group = ReentrantCallbackGroup()

        # 發布者
        self.publisher_ = self.create_publisher(
            GripperCommand, "/gripper_command", 10, callback_group=self.callback_group
        )

        # 訂閱者
        self.subscription1 = self.create_subscription(
            GripperCommand,
            "/gripper_response",
            self.listener_callback1,
            10,
            callback_group=self.callback_group,
        )

        self.subscription2 = self.create_subscription(
            GripperInfo,
            "/gripper_info",
            self.listener_callback2,
            10,
            callback_group=self.callback_group,
        )

        self.Timer = self.create_timer(
            0.1, self.timer_CallBack, callback_group=self.callback_group
        )

        self.id = 4
        self.num = None

    def publish_message(self, id, num):
        msg = GripperCommand()
        msg.id = id
        msg.num = num
        self.publisher_.publish(msg)
        print(f"Published: id={msg.id}, num={msg.num}")

    def set_num(self, num):
        self.num = num

    def timer_CallBack(self):
        user_input = input(
            # "輸入 'a' 設定 num=1 / 'b' 設定 num=2: / 'c' 設定 num=-1: "
            "輸入 1~6 or 'e': "
        )
        if user_input == "1":
            self.publish_message(4, ArmCmd.CMD_RELEASE)
        elif user_input == "2":
            self.publish_message(4, ArmCmd.CMD_GRAB)
        elif user_input == "3":
            self.publish_message(4, ArmCmd.CMD_INIT)
        elif user_input == "4":
            self.publish_message(4, ArmCmd.CMD_POWERON)
        elif user_input == "5":
            self.publish_message(4, ArmCmd.CMD_POWEROFF)
        elif user_input == "6":
            self.publish_message(4, ArmCmd.CMD_STATE_CHECK)
        elif user_input == "e":
            self.publish_message(4, ArmCmd.CMD_ERROR)
        else:
            print("無效的輸入")

    def listener_callback1(self, msg):
        print("callback1")
        print(f"Received: id={msg.id}, num={msg.num}, resp={msg.resp}")

    def listener_callback2(self, msg):
        print("callback2")
        print(f"Received: result={msg.result}")


def main(args=None):
    rclpy.init(args=args)
    gripper_publisher = GripperPublisher()

    # 使用 MultiThreadedExecutor 運行節點
    executor = MultiThreadedExecutor()
    executor.add_node(gripper_publisher)

    try:
        executor.spin()
        # while rclpy.ok():
        #     user_input = input(
        #         # "輸入 'a' 設定 num=1 / 'b' 設定 num=2: / 'c' 設定 num=-1: "
        #         "輸入 1~6 or 'e': "
        #     )
        #     if user_input == "1":
        #         gripper_publisher.publish_message(4, ArmCmd.CMD_RELEASE)
        #     elif user_input == "2":
        #         gripper_publisher.publish_message(4, ArmCmd.CMD_GRAB)
        #     elif user_input == "3":
        #         gripper_publisher.publish_message(4, ArmCmd.CMD_INIT)
        #     elif user_input == "4":
        #         gripper_publisher.publish_message(4, ArmCmd.CMD_POWERON)
        #     elif user_input == "5":
        #         gripper_publisher.publish_message(4, ArmCmd.CMD_POWEROFF)
        #     elif user_input == "6":
        #         gripper_publisher.publish_message(4, ArmCmd.CMD_STATE_CHECK)
        #     elif user_input == "e":
        #         gripper_publisher.publish_message(4, ArmCmd.CMD_ERROR)
        #     else:
        #         print("無效的輸入")

        #     # 处理一次ROS2的回调，确保订阅消息的处理
        #     rclpy.spin_once(gripper_publisher, timeout_sec=10)

    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        gripper_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
