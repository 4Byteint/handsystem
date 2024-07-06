# 6/30 整合
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import GripperCommand, GripperInfo
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time
from canlib import canlib, Frame
import threading


class Claw:
    def __init__(self):
        self.state = "PowerOn"  # 初始狀態
        self.ch = canlib.openChannel(
            channel=0, flags=canlib.Open.EXCLUSIVE, bitrate=canlib.canBITRATE_1M
        )
        self.ch.setBusOutputControl(canlib.Driver.NORMAL)
        self.ch.busOn()
        self.power_flag = False
        self.ros_flag = False
        self.sendSTM_flag = False
        self.sendUNO_flag = False

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
        ask_Boot = Frame(id_=1, data=[0, 1, 1, 1, 0, 0, 0, 0], dlc=8)
        while True:
            user_input = input("please enter a to start: ")
            if user_input == "a":
                self.ch.write(ask_Boot)
                time.sleep(0.1)
                break
        count = 0
        U_flag = False
        STM_flag = False
        start_time = time.time()
        while count < 2:
            try:
                msg = self.ch.read(timeout=5000)  # timeout 機制
                print(msg)
                if (msg.data[2] == 2) and msg.data[3] == 1:  # STM
                    count += 1
                    STM_flag = True
                elif (msg.data[2] == 3) and msg.data[3] == 1:  # UNO
                    count += 1
                    U_flag = True
                if STM_flag and U_flag:
                    break
            except canlib.CanNoMsg:
                if time.time() - start_time > 5:
                    count = 0
                    if not U_flag:
                        U_flag = False
                        print("沒收到 UNO 開機，重新發送")
                    if not STM_flag:
                        STM_flag = False
                        print("沒收到 STM 開機，重新發送")
                    self.ch.write(ask_Boot)
        if STM_flag and U_flag:
            return True

    def initialization(self):
        # 1. 發送初始化指令
        U_init = Frame(id_=3, data=[0, 1, 1, 2, 0, 0, 0, 0], dlc=8)
        STM_init = Frame(
            id_=2, data=[0, 1, 1, 3, 10, 0, 0, 0], dlc=8
        )  # 初始值10 (for STM)
        self.ch.write(U_init)
        self.ch.write(STM_init)
        # 2. 等待初始化完成
        count = 0
        U_flag = False
        STM_flag = False
        start_time = time.time()
        while count < 2:
            try:
                msg = self.ch.read(timeout=5000)  # timeout 機制
                print(msg)
                if (msg.data[2] == 2) and msg.data[3] == 2:  # STM
                    count += 1
                    STM_flag = True
                elif (msg.data[2] == 3) and msg.data[3] == 2:  # UNO
                    count += 1
                    U_flag = True
                if STM_flag and U_flag:
                    break
            except canlib.CanNoMsg:
                if time.time() - start_time > 5:
                    count = 0
                    if not U_flag:
                        U_flag = False
                        print("沒收到 UNO 初始化，重新發送")
                        self.ch.write(U_init)
                    if not STM_flag:
                        STM_flag = False
                        print("沒收到 STM 初始化，重新發送")
                        self.ch.write(STM_init)
        if STM_flag and U_flag:
            return True

    def grabbing(self):
        
        if  sendUNO_flag:
            sensor_request = Frame(id_=3, data=[0, 1, 1, 6, 0, 0, 0, 0], dlc=8)
            self.ch.write(sensor_request)
            sendUNO_flag=False
        
        if  sendSTM_flag:
            print("grabbing... ")
            grab_action = Frame(id_=2, data=[0, 1, 1, 4, 200, 0, 0, 0], dlc=8)
            self.ch.write(grab_action)
            sendSTM_flag=False
        else
            try:
                msg = self.ch.read(timeout=5000)  # timeout 機制
                print(msg)
                if (msg.data[2] == 2) and msg.data[3] == 4:  # STM
                    print("STM開始夾")
                    return True
                else
                    print("STM尚未開夾")
                    
                    return False
            except canlib.CanNoMsg:
                sendSTM_flag = True
                print("未收到STM是否開夾,重新發送")

    def Release(self):
        
        if sendSTM_flag:
            start_release = Frame(id_=2, data=[0, 1, 1, 5, 10, 0, 0, 0], dlc=8)
            self.ch.write(start_release)
        else:
            try:
                msg = self.ch.read(timeout=5000)  # timeout 機制
                print(msg)
                if (msg.data[2] == 2) and msg.data[3] == 5:  # STM
                    STM_flag = True
                    print("STM開始放開")
                    return True
                else:
                    print("STM尚未放開")
                    return False
            except canlib.CanNoMsg:
                sendSTM_flag = True
                print("未收到STM是否放開,重新發送")
                  
        

    def shutdown(self):
        # 關閉機器
        self.ch.busOff()
        self.ch.close()


#


class pubsub(Node, Claw):

    def __init__(self):
        super().__init__("pubsub")
        # 使用 ReentrantCallbackGroup
        self.callback_group1 = ReentrantCallbackGroup()
        self.callback_group2 = ReentrantCallbackGroup()
        self.callback_group3 = ReentrantCallbackGroup()
        self.callback_group4 = ReentrantCallbackGroup()
        # 訂閱者
        self.subscription = self.create_subscription(
            GripperCommand,
            "/gripper_command",
            self.listener_callback,
            10,
            callback_group=self.callback_group1,
        )
        self.subscription
        self.lock = threading.Lock()

        # 發布者:回傳 response
        self.publisher_resp = self.create_publisher(
            GripperCommand,
            "/gripper_response",  # 請確認回傳的 topic 名稱
            10,
            callback_group=self.callback_group2,
        )
        # 發布者:發布 result
        self.publisher_info = self.create_publisher(
            GripperInfo,
            "/gripper_info",  # 請確認回傳的 topic 名稱
            10,
            callback_group=self.callback_group3,
        )

        # 狀態變數 called every 0.01 sec
        self.gripper_state = None
        self.claw_callback_timer = self.create_timer(
            0.01, self.claw_callback, callback_group=self.callback_group4
        )
        self.claw = Claw()

    def listener_callback(self, msg):
        with self.lock:
            print(f'I heard: "{msg.num}"')

            # 更新狀態變數
            try:
                if msg.num == 1:
                    self.gripper_state = "grab"
                    self.Arm_action_grip()

                elif msg.num == 2:
                    self.gripper_state = "release"
                    self.Arm_action_grip()

                elif msg.num == -1:
                    raise ValueError("例外發生")
            except ValueError as e:
                print(f"Exception occurred: {e}")

    def shutdown(self):
        self.claw_callback_timer.cancel()
        self.claw.shutdown()

    def Arm_action_grip(self):
        if self.gripper_state == "grab":
            print("開始夾取流程")
            # 執行夾取流程的邏輯
            # 發布夾取狀態
            # msg = GripperCommand()
            # msg.id = 5
            # msg.num = 1
            # msg.resp = 1
            # self.publisher_resp.publish(msg)
            
            sendSTM_flag = True
            sendUNO_flag = True
            self.claw.state = "grabbing"
        elif self.gripper_state == "release":
            print("開始放下流程")
            # 執行放下流程的邏輯
            # 發布放下狀態
            # msg = GripperCommand()
            # msg.id = 5
            # msg.num = 2
            # msg.resp = 2
            # self.publisher_resp.publish(msg)
            sendSTM_flag = True
            self.claw.state = "releasing"

    def claw_callback(self):
        if not self.claw.power_flag:
            self.claw.power_on()
            self.claw.power_flag = True
        if self.claw.state == "CheckConnection":
            if self.claw.check_connection():
                print("連線成功!")
                self.claw.state = "Initialization"

        if self.claw.state == "Initialization":
            if self.claw.initialization():
                print("初始化成功!")
                self.claw.state = "wait_for_command"

        if self.claw.state == "wait_for_command":
            print("等待命令...")
            # time.sleep(1)

        if self.claw.state == "grabbing":
            # print("開始夾取流程")
            if self.claw.grabbing():
                msg = GripperInfo()
                msg.result=1
                self.publisher_info.publish(msg)
                print("回傳已夾取")

        if self.claw.state == "releasing":
            # print("開始放下流程")
            if self.claw.Release():
                msg = GripperInfo()
                msg.result=3
                self.publisher_info.publish(msg)
                print("回傳已放開")

        # elif not ros_flag:
        #             self.claw.state = "wait_for_command"

        # if self.claw.state == "hold":
        #     if self.claw.Hold():
        #         print("開始夾取!")
        #         pass


def main(args=None):
    rclpy.init(args=args)

    pubsub_instance = pubsub()
    # Claw_instance = Claw()
    # 使用 MultiThreadedExecutor 運行節點
    executor = MultiThreadedExecutor()
    executor.add_node(pubsub_instance)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        pubsub_instance.destroy_node()
        pubsub_instance.claw.shutdown()
        rclpy.shutdown()


# def testForGit():
#     print("放下完成")

if __name__ == "__main__":
    main()
