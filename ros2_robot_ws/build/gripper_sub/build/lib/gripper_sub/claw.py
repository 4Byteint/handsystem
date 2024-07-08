from canlib import canlib, Frame
import time
from gripper_sub.table import GripperState
from gripper_sub.table import Device, CanId, CanData, Status


class Claw:
    def __init__(self):
        self.state = GripperState.STATE_POWER_OFF  # 初始狀態

        self.ch = canlib.openChannel(
            channel=0, flags=canlib.Open.EXCLUSIVE, bitrate=canlib.canBITRATE_1M
        )
        self.ch.setBusOutputControl(canlib.Driver.NORMAL)
        self.ch.busOn()

        # self.power_flag = False
        # self.ros_flag = False
        self.sendSTM_flag = False
        self.sendUNO_flag = False
        self.sendingTimeStart_STM = 0
        self.sendingTimeStart_UNO = 0
        self.canMsg = Frame(id_=0, data=[0, 0, 0, 0, 0, 0, 0, 0], dlc=8)
        self.firstTimeFlag = {Device.STM: False, Device.UNO: False}
        self.connectStatus = {Device.STM: Status.UNKNOWN, Device.UNO: Status.UNKNOWN}
        self.initStatus = {Device.STM: Status.UNKNOWN, Device.UNO: Status.UNKNOWN}

        # state task to do
        self.toDoTask = {
            GripperState.STATE_POWER_OFF: self.PowerOff,
            GripperState.STATE_POWER_ON: self.PowerOn,
            GripperState.STATE_INITIALIZING: self.Initialization,
            GripperState.STATE_RELEASING: self.Release,
            GripperState.STATE_GRABBING: self.Grab,
            GripperState.STATE_OFFLINE: self.OffLine,
            GripperState.STATE_GRABBING_MISS: self.GrabbingMiss,
            GripperState.STATE_RELEASING_MISS: self.ReleasingMiss,
        }

    def PowerOff(self):
        """tell STM to turn off motor"""
        print("power off")

        return Status.SUCCESS

    def PowerOn(self):
        """just check"""
        print("power on")
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

        # self.state = "CheckConnection"
        print("connectStatus")
        print(self.connectStatus[Device.UNO])
        print(self.connectStatus[Device.STM])
        if (
            self.connectStatus[Device.UNO] == Status.SUCCESS
            and self.connectStatus[Device.STM] == Status.SUCCESS
        ):
            self.state = GripperState.STATE_INITIALIZING
            return Status.SUCCESS

    # def CheckConnection(self):
    #     print("connection check")

    #     ask_Boot = Frame(id_=1, data=[0, 1, 1, 1, 0, 0, 0, 0], dlc=8)
    #     while True:
    #         user_input = input("please enter a to start: ")
    #         if user_input == "a":
    #             self.ch.write(ask_Boot)
    #             time.sleep(0.1)
    #             break
    #     count = 0
    #     U_flag = False
    #     STM_flag = False
    #     start_time = time.time()
    #     while count < 2:
    #         try:
    #             msg = self.ch.read(timeout=5000)  # timeout 機制
    #             print(msg)
    #             if (msg.data[2] == 2) and msg.data[3] == 1:  # STM
    #                 count += 1
    #                 STM_flag = True
    #             elif (msg.data[2] == 3) and msg.data[3] == 1:  # UNO
    #                 count += 1
    #                 U_flag = True
    #             if STM_flag and U_flag:
    #                 break
    #         except canlib.CanNoMsg:
    #             if time.time() - start_time > 5:
    #                 count = 0
    #                 if not U_flag:
    #                     U_flag = False
    #                     print("沒收到 UNO 開機，重新發送")
    #                 if not STM_flag:
    #                     STM_flag = False
    #                     print("沒收到 STM 開機，重新發送")
    #                 self.ch.write(ask_Boot)
    #     if STM_flag and U_flag:
    #         return True

    def Initialization(self):
        print("init")

        if self.firstTimeFlag[Device.UNO]:
            self.ch.write(
                Frame(
                    id_=CanId.CANID_PI_TO_UNO,
                    data=list(CanData.CMD_PI_UNO_INIT) + [0, 0, 0, 0],
                    dlc=8,
                )
            )
            self.firstTimeFlag[Device.UNO] = False
        else:
            try:
                if self.canMsg.data[0:4] == list(CanData.STATE_UNO_INIT_OK):
                    self.initStatus[Device.UNO] = Status.SUCCESS
                elif self.canMsg.data[0:4] == list(CanData.STATE_UNO_INIT_NOTOK):
                    self.initStatus[Device.UNO] = Status.FAILED
            except canlib.CanNoMsg:
                self.firstTimeFlag[Device.UNO] = True
                print("未收到UNO INIT,重新發送")
                # return False

        if self.firstTimeFlag[Device.STM]:
            self.ch.write(
                Frame(
                    id_=CanId.CANID_PI_TO_STM,
                    data=list(CanData.CMD_PI_STM_INIT) + [0, 0, 0, 0],
                    dlc=8,
                )
            )
            self.firstTimeFlag[Device.STM] = False
        else:
            try:
                if self.canMsg.data[0:4] == list(CanData.STATE_STM_INIT_OK):
                    self.initStatus[Device.STM] = Status.SUCCESS
                elif self.canMsg.data[0:4] == list(CanData.STATE_STM_INIT_NOTOK):
                    self.initStatus[Device.STM] = Status.FAILED
            except canlib.CanNoMsg:
                self.firstTimeFlag[Device.STM] = True
                print("未收到STM INIT,重新發送")
                # return False
        if (
            self.initStatus[Device.STM] == Status.SUCCESS
            and self.initStatus[Device.STM] == Status.SUCCESS
        ):
            return Status.SUCCESS
        elif (
            self.initStatus[Device.STM] == Status.FAILED
            and self.initStatus[Device.STM] == Status.FAILED
        ):
            return Status.FAILED
        else:
            return Status.UNKNOWN

        # # 1. 發送初始化指令
        # U_init = Frame(id_=3, data=[0, 1, 1, 2, 0, 0, 0, 0], dlc=8)
        # STM_init = Frame(
        #     id_=2, data=[0, 1, 1, 3, 10, 0, 0, 0], dlc=8
        # )  # 初始值10 (for STM)
        # self.ch.write(U_init)
        # self.ch.write(STM_init)
        # # 2. 等待初始化完成
        # count = 0
        # U_flag = False
        # STM_flag = False
        # start_time = time.time()
        # while count < 2:
        #     try:
        #         msg = self.ch.read(timeout=5000)  # timeout 機制
        #         print(msg)
        #         if (msg.data[2] == 2) and msg.data[3] == 2:  # STM
        #             count += 1
        #             STM_flag = True
        #         elif (msg.data[2] == 3) and msg.data[3] == 2:  # UNO
        #             count += 1
        #             U_flag = True
        #         if STM_flag and U_flag:
        #             break
        #     except canlib.CanNoMsg:
        #         if time.time() - start_time > 5:
        #             count = 0
        #             if not U_flag:
        #                 U_flag = False
        #                 print("沒收到 UNO 初始化，重新發送")
        #                 self.ch.write(U_init)
        #             if not STM_flag:
        #                 STM_flag = False
        #                 print("沒收到 STM 初始化，重新發送")
        #                 self.ch.write(STM_init)
        # if STM_flag and U_flag:
        #     return True

    def Grab(self):

        if self.firstTimeFlag[Device.UNO]:
            # sensor_request = Frame(id_=3, data=[0, 1, 1, 6, 0, 0, 0, 0], dlc=8)
            # self.ch.write(sensor_request)
            self.ch.write(
                Frame(
                    id_=CanId.CANID_PI_TO_UNO,
                    data=list(CanData.CMD_PI_SENSOR_REQUEST) + [0, 0, 0, 0],
                    dlc=8,
                )
            )
            self.firstTimeFlag[Device.UNO] = False

        #  remove if and keep else if you want to send STM cmd Constantly
        if self.firstTimeFlag[Device.STM]:
            # grab_action = Frame(id_=2, data=[0, 1, 1, 4, 200, 0, 0, 0], dlc=8)
            # self.ch.write(grab_action)
            self.ch.write(
                Frame(
                    id_=CanId.CANID_PI_TO_STM,
                    data=list(CanData.CMD_PI_GRABBING) + [200, 0, 0, 0],
                    dlc=8,
                )
            )
            self.firstTimeFlag[Device.STM] = False
            # self.sendingTimeStart_STM = time.time()
        else:
            try:
                # msg = self.ch.read(timeout=5000)  # timeout 機制
                # print(msg)

                if self.canMsg.data[0:4] == list(CanData.STATE_STM_START_GRABBING):
                    return Status.SUCCESS

                # if (msg.data[2] == 2) and msg.data[3] == 4:  # STM
                #     print("STM開始夾")
                #     return True
            except canlib.CanNoMsg:
                # if time.time() - self.sendingTimeStart_STM > 5:
                self.firstTimeFlag[Device.STM] = True
                print("未收到STM是否開夾,重新發送")
                return Status.UNKNOWN

    def Release(self):

        if self.firstTimeFlag[Device.STM]:
            # start_release = Frame(id_=2, data=[0, 1, 1, 5, 10, 0, 0, 0], dlc=8)
            # self.ch.write(start_release)
            self.ch.write(
                Frame(
                    id_=CanId.CANID_PI_TO_STM,
                    data=list(CanData.CMD_PI_RELEASING) + [10, 0, 0, 0],
                    dlc=8,
                )
            )
            # self.sendingTimeStart_STM = time.time()
            # print(time.time() - self.sendingTimeStart_STM)
        else:
            try:
                # msg = self.ch.read(timeout=5000)  # timeout 機制
                # print(msg)

                if self.canMsg.data[0:4] == list(CanData.STATE_STM_START_RELEASING):
                    return Status.SUCCESS

                # if (msg.data[2] == 2) and msg.data[3] == 5:  # STM
                #     print("c8 c8 c8 88c8 8c88 c88c 8c88 8c")
                #     print("STM開始放開")
                #     return True
                # else:
                #     print("STM尚未放開,重新發送")
                #     self.sendSTM_flag = True
                #     return False
            except canlib.CanNoMsg:
                # if time.time() - self.sendingTimeStart_STM > 5:
                self.firstTimeFlag[Device.STM] = True
                print("未收到STM是否放開,重新發送")
                return Status.UNKNOWN

    def OffLine():
        """nothing to do with STM,UNO"""
        print("offline")
        return Status.SUCCESS

    def GrabbingMiss():
        """nothing to do with STM,UNO"""
        print("grabing miss")
        return Status.SUCCESS

    def ReleasingMiss():
        """nothing to do with STM,UNO"""
        print("releasing miss")
        return Status.SUCCESS

    def NoTask():
        """nothing to do with STM,UNO"""
        print("no task")
        return Status.SUCCESS

    def shutdown(self):
        # 關閉機器
        self.ch.busOff()
        self.ch.close()
