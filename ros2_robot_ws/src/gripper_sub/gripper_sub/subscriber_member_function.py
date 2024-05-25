import rclpy
from rclpy.node import Node
from robot_interfaces.msg import GripperCommand                        # CHANGE
import os
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            GripperCommand,                                               # CHANGE
            '/gripper_command',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
            self.get_logger().info(f'Received: id={msg.id} num={msg.num}')  # CHANGE
            os.environ['ARM_ID'] = int(msg.id)
            os.environ['ARM_NUM'] = int(msg.num)
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()