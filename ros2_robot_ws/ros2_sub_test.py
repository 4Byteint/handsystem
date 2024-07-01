import rclpy
from rclpy.node import Node
from robot_interfaces.msg import GripperCommand
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time
class MinimalPublisherSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_publisher_subscriber')

        # 使用 ReentrantCallbackGroup
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        # 訂閱者
        self.subscription = self.create_subscription(
            GripperCommand,
            '/gripper_command',
            self.listener_callback,
            10,
            callback_group=self.callback_group)
        self.subscription

        # 發布者
        self.publisher = self.create_publisher(
            GripperCommand, 
            '/gripper_response', # 請確認回傳的 topic 名稱
            10,
            callback_group=self.callback_group) 

        # 狀態變數
        self.gripper_state = None
        self.claw_callback_timer = self.create_timer(0, self.claw_callback, callback_group=self.callback_group)

    def listener_callback(self, msg):
        print(f'I heard: "{msg.num}"')
        
        # 更新狀態變數
        try:
            if msg.num == 1 :
                self.gripper_state = 'grip'
                self.claw_action()
                #self.action_executed = True  # 設置標志位，表示動作已執行
            elif msg.num == 2 :
                self.gripper_state = 'release'
                self.claw_action()
                #self.action_executed = True  # 設置標志位，表示動作已執行
            # 要新增一個例外處理並把 self.action_executed = False
            elif msg.num == -1 :
                raise ValueError('例外發生')
        except ValueError as e:
            print(f'Exception occurred: {e}')

    def claw_action(self):
        if self.gripper_state == 'grip':
            print("開始夾取流程")
            # 執行夾取流程的邏輯
            # 發布夾取狀態
            msg = GripperCommand()
            msg.id = 5
            msg.num = 1 
            msg.resp = 1
            self.publisher.publish(msg)
            self.gripper_state = None  # 重置狀態
        elif self.gripper_state == 'release':
            print("開始放下流程")
            # 執行放下流程的邏輯
            # 發布放下狀態
            msg = GripperCommand()
            msg.id = 5
            msg.num = 2 
            msg.resp = 2
            self.publisher.publish(msg)
            self.gripper_state = None  # 重置狀態

    def claw_callback(self):
        print("claw")
        time.sleep(2)
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher_subscriber = MinimalPublisherSubscriber()

    # 使用 MultiThreadedExecutor 運行節點
    executor = MultiThreadedExecutor()
    executor.add_node(minimal_publisher_subscriber)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        minimal_publisher_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()