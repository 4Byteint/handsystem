import rclpy
from rclpy.node import Node

from robot_interfaces.msg import GripperCommand                            # CHANGE


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(GripperCommand, '/gripper_command', 10)  # CHANGE
        # 只發送一次
        msg = GripperCommand()                                                # CHANGE
        msg.id = 4
        msg.num = 1                                           # CHANGE
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: ID={msg.id}, Num={msg.num}")       # CHANGE
        # 移除關閉節點的程式碼
        # self.destroy_node()
        # rclpy.shutdown()  
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)# 保持節點運行

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()