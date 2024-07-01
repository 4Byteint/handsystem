import rclpy
from rclpy.node import Node
from robot_interfaces.msg import GripperCommand
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class GripperPublisher(Node):
    def __init__(self):
        super().__init__('gripper_publisher')

        # 使用 ReentrantCallbackGroup 來允許回調並行執行
        self.callback_group = ReentrantCallbackGroup()

        # 發布者
        self.publisher_ = self.create_publisher(
            GripperCommand, 
            '/gripper_command', 
            10,
            callback_group=self.callback_group
        )

        # 訂閱者
        self.subscription = self.create_subscription(
            GripperCommand,
            '/gripper_response',
            self.listener_callback,
            10,
            callback_group=self.callback_group
        )

        self.id = 4
        self.num = None

    def publish_message(self, id, num):
        msg = GripperCommand()
        msg.id = id
        msg.num = num
        self.publisher_.publish(msg)
        print(f'Published: id={msg.id}, num={msg.num}')

    def set_num(self, num):
        self.num = num

    def listener_callback(self, msg):
        print(f'Received: id={msg.id}, num={msg.num}, resp={msg.resp}')

def main(args=None):
    rclpy.init(args=args)
    gripper_publisher = GripperPublisher()

    # 使用 MultiThreadedExecutor 運行節點
    executor = MultiThreadedExecutor()
    executor.add_node(gripper_publisher)

    try:
        while rclpy.ok():
            user_input = input("輸入 'a' 設定 num=1 / 'b' 設定 num=2: / 'c' 設定 num=-1: ")
            if user_input == 'a':
                gripper_publisher.publish_message(4, 1)
            elif user_input == 'b':
                gripper_publisher.publish_message(4, 2)
            elif user_input == 'c':
                gripper_publisher.publish_message(4, -1)
            else:
                print("無效的輸入，請輸入 'a', 'b', 'c'")
            
            # 处理一次ROS2的回调，确保订阅消息的处理
            rclpy.spin_once(gripper_publisher, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        gripper_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
