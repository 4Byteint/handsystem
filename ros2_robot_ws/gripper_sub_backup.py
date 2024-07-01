# for connection test
from canlib import canlib, Frame                  # CHANGE
import time

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
        while count<2 :
            try:
                msg = self.ch.read(timeout=5000)                    # timeout 機制
                print(msg)
                if msg.data[2] == 2  and msg.data[3] == 1:        # STM
                   count +=1
                   STM_flag = True
                elif msg.data[2] == 3 and msg.data[3] == 1:       # UNO
                   count +=1
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
        U_init = Frame(id_=3, data=[0,1,1,2,0,0,0,0], dlc=8)  
        STM_init = Frame(id_=2, data=[0,1,1,3,10,0,0,0] , dlc=8) #初始值10(for STM)
        self.ch.write(U_init)
        self.ch.write(STM_init)
        time.sleep(1)
      # 2. 等待初始化完成
        count = 0
        U_flag = False
        STM_flag = False
        start_time = time.time()
        while count<2 :
            try:
                msg = self.ch.read(timeout=5000)                    # timeout 機制
                print(msg)
                if msg.data[2] == 2 and msg.data[3] == 2:        # STM
                   count +=1
                   STM_flag = True
                elif msg.data[2] == 3 and msg.data[3] == 2:      # UNO
                   count +=1
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
    def wait_for_command(self):
        print("正在等待手臂指令")
        return True
    def StartGrabbing(self):
        start_grab = Frame(id_=2, data=[0,1,1,4,50,0,0,0], dlc=8)
        request_sensor = Frame(id_=3, data=[0,1,1,5,0,0,0,0], dlc=8)
        self.ch.write(start_grab)
        self.ch.write(request_sensor)
        return True
    # def Hold(self):
    #     start_grab = Frame(id_=2, data=[0,1,1,4,50,0,0,0], dlc=8)
    #     self.ch.write(start_grab)
    # def Release(self):
    #     start_release = Frame(id_=2, data=[0,1,1,5,10,0,0,0], dlc=8)
    #     self.ch.write(start_release)
    def shutdown(self):
        # 關閉機器
        self.ch.busOff()
        self.ch.close()

def main(args=None):

    claw_machine = Claw()
    claw_machine.power_on()
    while True:
        if claw_machine.state == "CheckConnection":
                if claw_machine.check_connection():
                    print("連線成功!")
                    claw_machine.state = "Initialization"

        elif claw_machine.state == "Initialization":
                if claw_machine.initialization():
                    print("初始化成功!")
                    claw_machine.state = "wait_for_command"

        elif claw_machine.state == "wait_for_command":
                if claw_machine.wait_for_command():
                    print("開始夾取流程!")
                    claw_machine.state = "StartGrabbing"

        elif claw_machine.state == "StartGrabbing":
                if claw_machine.StartGrabbing():
                    print("開始夾取!")
                    break

    claw_machine.shutdown()
     
if __name__ == '__main__':
    main()