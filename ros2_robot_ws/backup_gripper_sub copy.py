# for ROS2 function
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import GripperCommand   
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from canlib import canlib, Frame
import time
from threading import Thread

class Claw:
    def __init__(self):
        self.state = "PowerOn"  # 初始狀態
        self.ch = canlib.openChannel(
          	channel=0,
          	flags=canlib.Open.EXCLUSIVE,
          	bitrate=canlib.canBITRATE_1M
        )
        self.ch.setBusOutputControl(canlib.Driver.NORMAL)
        self.ch.busOn()
        self.current_msg = None
    def power_on(self):
        # 上電後的初始化操作，例如檢查電源、啟動系統、檢查各個device是否開啟
        # while True:
        #   # 在背景執行 process 
        #   process = subprocess.Popen(["python3", "digit_check.py"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        #   time.sleep(3)
        #   if process.poll() is None:
        #     print("camera is running.")
        #     self.state = "CheckConnection"
        #     break
        #   else:
        #     print("digit not found")

        self.state = "CheckConnection"
    def check_connection(self):
        ask_Boot = Frame(id_=1, data=[0,1,1,1,0,0,0,0] , dlc=8)
        while True:
            user_input = input("please enter a to start: ")
            if user_input == 'a':
                self.ch.write(ask_Boot)
                time.sleep(0.1)
                break
        count = 0
        U_flag = False
        STM_flag = False
        start_time = time.time()
        #while count<2 :
        while count<1:
            try:
                msg = self.ch.read(timeout=5000)                    # timeout 機制
                print(msg)
                # if (msg.data[2] == 2 ) and msg.data[3] == 1:        # STM
                #    count +=1
                #    STM_flag = True
                #elif (msg.data[2] == 3) and msg.data[3] == 1:       # UNO
                if (msg.data[2] == 3) and msg.data[3] == 1: 
                   count +=1
                   U_flag = True
                #if STM_flag and U_flag:
                if U_flag:
                   break
            except canlib.CanNoMsg:
                if time.time() - start_time > 5:
                    count = 0
                    if not U_flag:
                        U_flag = False
                        print("沒收到 UNO 開機，重新發送")
                    # if not STM_flag:
                    #     STM_flag = False
                    #     print("沒收到 STM 開機，重新發送")
                    self.ch.write(ask_Boot)
        # if STM_flag and U_flag:
        #     return True
        if U_flag:
            return True
    def initialization(self):
      # 1. 發送初始化指令
        U_init = Frame(id_=3, data=[0,1,1,2,0,0,0,0], dlc=8)  
        STM_init = Frame(id_=2, data=[0,1,1,3,10,0,0,0] , dlc=8) #初始值10 (for STM)
        self.ch.write(U_init)
        self.ch.write(STM_init)
        time.sleep(1)
      # 2. 等待初始化完成
        count = 0
        U_flag = False
        STM_flag = False
        start_time = time.time()
        #while count<2 :
        while count<1:
            try:
                msg = self.ch.read(timeout=5000)                    # timeout 機制
                print(msg)
                # if (msg.data[2] == 2) and msg.data[3] == 2:        # STM
                #    count +=1
                #    STM_flag = True
                # elif (msg.data[2] == 3) and msg.data[3] == 2:      # UNO
                if (msg.data[2] == 3) and msg.data[3] == 2:
                   count +=1
                   U_flag = True
                # if STM_flag and U_flag:
                #    break
                if U_flag:
                    break
            except canlib.CanNoMsg:
                if time.time() - start_time > 5:
                    count = 0
                    if not U_flag:
                        U_flag = False
                        print("沒收到 UNO 初始化，重新發送")
                        self.ch.write(U_init)
                    # if not STM_flag:
                    #     STM_flag = False
                    #     print("沒收到 STM 初始化，重新發送")
                    #     self.ch.write(STM_init)
        # if STM_flag and U_flag:
        #     return True
        if U_flag:
            return True
    # def wait_for_command(self):
    #     print("正在等待手臂指令")
    #     start_time = time.time()  # 設置循環開始時間
    #     arm_flag = False

    #     while time.time() - start_time <= 5:  # 檢查是否超過 5 秒
    #         if self.current_msg:
    #             if self.current_msg.id == 4 and self.current_msg.num == 1:
    #                 start_grab = Frame(id_=2, data=[0,1,1,4,50,0,0,0], dlc=8)
    #                 request_sensor = Frame(id_=2, data=[0,1,1,5,0,0,0,0], dlc=8)
    #                 self.ch.write(start_grab)
    #                 self.ch.write(request_sensor)
    #                 print("發送開始夾取 & 要求傳送sensor資料")
    #                 arm_flag = True
    #                 break  # 成功接收指令，跳出循環
    #         time.sleep(0.1)  # 避免忙等待

    #     if not arm_flag:
    #         print("沒收到手臂端指令，請重新發送")

    #     return arm_flag
    
    def grabbing(self):
        print("grabbing... ")
        grab_action = Frame(id_=2, data=[0,1,1,5,50,0,0,0], dlc=8)
        self.ch.write(grab_action)
        return True
    # def Hold(self):
    #     start_grab = Frame(id_=2, data=[0,1,1,4,50,0,0,0], dlc=8)
    #     self.ch.write(start_grab)
    def Release(self):
        start_release = Frame(id_=2, data=[0,1,1,5,10,0,0,0], dlc=8)
        self.ch.write(start_release)
    def shutdown(self):
        # 關閉機器
        self.ch.busOff()
        self.ch.close()

class GripperCommandSubscriber(Node):
    def __init__(self):
        super().__init__('gripper_command_subscriber')
        self.callback_group = ReentrantCallbackGroup()
        self.subscription = self.create_subscription(
            GripperCommand,
            '/gripper_command',
            self.listener_callback,
            10
        )
        
        #self.current_id = None
        #self.current_num = None

    def listener_callback(self, msg):
        print(f'Received: id={msg.id} num={msg.num}')
        id = msg.id
        num = msg.num
        
        if num == 1:
            start_grab = Frame(id_=2, data=[0,1,1,4,50,0,0,0], dlc=8)
            request_sensor = Frame(id_=2, data=[0,1,1,5,0,0,0,0], dlc=8)
            self.ch.write(start_grab)
            self.ch.write(request_sensor)
            print("發送開始夾取 & 要求傳送sensor資料")
            self.state == "Go_grabbing"
        elif num == 2:
            self.state = "Release"
        elif num == -1:
            self.state = "HandleException"
        else:
            print(f'No action defined for num = {num}')

def main(args=None):
    # 打開 ros2 通道
    rclpy.init(args=args)
    gripper_subscriber = GripperCommandSubscriber()
    # 執行器
    executor = MultiThreadedExecutor()
    executor.add_node(gripper_subscriber)
    # 在 thread 執行 gripper_subscriber
    executor_thread = Thread(target=executor.spin)
    executor_thread.start()

    # 使用 class_Claw
    claw_instance = Claw()
    claw_instance.power_on()
    if claw_instance.state == "CheckConnection":
        if claw_instance.check_connection():
            print("連線成功!")
            claw_instance.state = "Initialization"

    elif claw_instance.state == "Initialization":
        if claw_instance.initialization():
            print("初始化成功!")
            claw_instance.state = "wait_for_command"

    elif claw_instance.state == "wait_for_command":
            print("等待命令...")
            if gripper_subscriber.state == "go_grabbing":
                if claw_instance.grabbing():
                    print("夾ing...")
                    claw_instance.state == "release"
            
    elif claw_instance.state == "release":
        if claw_instance.Release():
            print("開始夾取!")
        else:
            pass

    claw_instance.shutdown()
    GripperCommandSubscriber.destroy_node()
    rclpy.shutdown()
     
if __name__ == '__main__':
    main()