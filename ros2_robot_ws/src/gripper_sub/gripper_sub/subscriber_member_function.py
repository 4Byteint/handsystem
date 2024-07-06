# 6/30 整合
import time
import threading
from gripper_sub.claw import Claw
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from robot_interfaces.msg import GripperCommand, GripperInfo


#########################################################################
#########################################################################
#########################################################################
#########################################################################
#########################################################################


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

            self.claw.sendSTM_flag = True
            self.claw.sendUNO_flag = True
            print(self.claw.sendSTM_flag)
            print("開始傳")
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
            self.claw.sendSTM_flag = True
            self.claw.state = "releasing"
            self.sendingTimeStart_STM = time.time()

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
                msg.result = 1
                self.publisher_info.publish(msg)
                print("回傳已夾取")
                self.claw.state = "wait_for_command"

        if self.claw.state == "releasing":
            # print("開始放下流程")
            if self.claw.Release():
                msg = GripperInfo()
                msg.result = 3
                self.publisher_info.publish(msg)
                print("回傳已放開")
                self.claw.state = "wait_for_command"
            else:
                print("Release失敗")

        # elif not ros_flag:
        #             self.claw.state = "wait_for_command"

        # if self.claw.state == "hold":
        #     if self.claw.Hold():
        #         print("開始夾取!")
        #         pass
