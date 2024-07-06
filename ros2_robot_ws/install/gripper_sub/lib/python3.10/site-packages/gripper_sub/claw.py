from canlib import canlib, Frame
import time


class Claw:
    def __init__(self):
        self.state = "PowerOn"  # 初始狀態
        # self.ch = canlib.openChannel(
        #     channel=0, flags=canlib.Open.EXCLUSIVE, bitrate=canlib.canBITRATE_1M
        # )
        # self.ch.setBusOutputControl(canlib.Driver.NORMAL)
        # self.ch.busOn()
        self.power_flag = False
        self.ros_flag = False
        self.sendSTM_flag = False
        self.sendUNO_flag = False
        self.sendingTimeStart_STM = 0
        self.sendingTimeStart_UNO = 0

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

        if self.sendUNO_flag:
            sensor_request = Frame(id_=3, data=[0, 1, 1, 6, 0, 0, 0, 0], dlc=8)
            self.ch.write(sensor_request)
            self.sendUNO_flag = False

        if self.sendSTM_flag:
            print("grabbing... ")
            grab_action = Frame(id_=2, data=[0, 1, 1, 4, 200, 0, 0, 0], dlc=8)
            self.ch.write(grab_action)
            self.sendSTM_flag = False
            self.sendingTimeStart_STM = time.time()
        else:
            try:
                msg = self.ch.read(timeout=5000)  # timeout 機制
                print(msg)
                if (msg.data[2] == 2) and msg.data[3] == 4:  # STM
                    print("STM開始夾")
                    return True
                # elif :
                #     print("STM尚未開夾,重新發送")
                #     self.sendSTM_flag = True
                #     return False
            except canlib.CanNoMsg:
                if time.time() - self.sendingTimeStart_STM > 5:
                    self.sendSTM_flag = True
                    print("未收到STM是否開夾,重新發送")
                    return False

    def Release(self):

        print("sendSTM_flag")
        print(self.sendSTM_flag)
        if self.sendSTM_flag:
            start_release = Frame(id_=2, data=[0, 1, 1, 5, 10, 0, 0, 0], dlc=8)
            self.ch.write(start_release)
            self.sendSTM_flag = False
            # self.sendingTimeStart_STM = time.time()
            print(time.time() - self.sendingTimeStart_STM)
        else:
            try:
                msg = self.ch.read(timeout=5000)  # timeout 機制
                print(msg)
                if (msg.data[2] == 2) and msg.data[3] == 5:  # STM
                    print("c8 c8 c8 88c8 8c88 c88c 8c88 8c")
                    print("STM開始放開")
                    return True
                # else:
                #     print("STM尚未放開,重新發送")
                #     self.sendSTM_flag = True
                #     return False
            except canlib.CanNoMsg:
                if time.time() - self.sendingTimeStart_STM > 5:
                    self.sendSTM_flag = True
                    print("未收到STM是否放開,重新發送")
                    return False

    def shutdown(self):
        # 關閉機器
        self.ch.busOff()
        self.ch.close()
