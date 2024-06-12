import rclpy
from rclpy.node import Node
from robot_interfaces.msg import GripperCommand   
from canlib import canlib, Frame                  # CHANGE

class Cansendmsg:
    # def __init__(self):
    #     #self.state = "PowerOn"  # 初始狀態
    #     self.ch = canlib.openChannel(
    #   	    channel=0,
    #   	    flags=canlib.Open.EXCLUSIVE,
    #   	    bitrate=canlib.canBITRATE_1M
    #     )
    #     self.ch.setBusOutputControl(canlib.Driver.NORMAL)
    #     self.ch.busOn()
    def send_gripper_command(self):
        #start_grab = Frame(id_=2, data=[0,1,1,4,50,0,0,0], dlc=8)
        #self.ch.write(start_grab)
        print("got it!")
# listener
class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            GripperCommand,                                               # CHANGE
            '/gripper_command',
            self.listener_callback,
            10)
        self.subscription
        self.can_sender = Cansendmsg()  # 創建 Cansendmsg 的實例
    def listener_callback(self, msg):
            self.get_logger().info(f'Received: id={msg.id} num={msg.num}')  # CHANGE
            if msg.id == 4 and msg.num == 1:
                self.can_sender.send_gripper_command()  # 使用實例調用方法
def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
     
if __name__ == '__main__':
    main()