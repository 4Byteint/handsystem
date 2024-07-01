from canlib import canlib, Frame
import time
import sys
import select


class ClawMachine:
    def __init__(self):
        self.state = "PowerOn"  # 初始狀態
        self.ch = canlib.openChannel(
          	channel=0,
          	flags=canlib.Open.EXCLUSIVE,
          	bitrate=canlib.canBITRATE_250K
        )

# Set the CAN bus driver type to NORMAL.
        self.ch.setBusOutputControl(canlib.Driver.NORMAL)
 
# Activate the CAN chip.
        self.ch.busOn()
    def run(self):
        while True:
            if self.state == "PowerOn":
                self.power_on()
            elif self.state == "CheckConnection":
                if self.check_connection():  # 檢查連線結果
                  print("連線成功!進入初始化")
                  self.state = "Initialization"  # 連線成功，進入初始化狀態
                else:
                  print("連線失敗，請檢查連線")
                  time.sleep(1)  # 等待一段時間再重試
            elif self.state == "Initialization":
                self.initialization()
                if self.initialization():
                  print("初始化成功!等待手臂命令")
                  self.state = "StartGrabbing"
                  
    def power_on(self):
        # 上電後的初始化操作，例如檢查電源、啟動系統、檢查各個device是否開啟
        # ...
        self.state = "CheckConnection"

    def check_connection(self):
        ask_UBoot = Frame(id_=1, data=[0,1,1,1,0,0,0,0] , dlc=8)
        ready, _, _ = select.select([sys.stdin],[],[],0.1)
        if ready:
           user_input = input()
           if user_input == 'a':
             self.ch.write(ask_UBoot)
             time.sleep(0.5)
        else:
            while True:
                try:
                    msg = self.ch.read()
                    print(msg)
                    # 檢查uno開機訊號
                    if msg.id==3 and msg.data[3]==1 and msg.data[7]==1:
                      return True  # 連線成功
                except canlib.CanNoMsg:
                    time.sleep(0.5)
                    return False
        return False  # 沒有收到回應，連線失敗
        
    def initialization(self):
        # 進入初始狀態，例如歸零夾爪位置
      # 1. 發送歸零指令
      home_command = Frame(id_=1, data=[0,1,1,2,0,0,0,0], dlc=8)  # 假設 ID 為 2
      self.ch.write(home_command)
      time.sleep(1)
      # 2. 等待歸零完成
      while True:
          try:
              msg = self.ch.read()
              if msg.id == 3 and msg.data[3] == 2 and msg.data[7] == 1:
                  print("UNO歸零完成")
                  return True
          except canlib.CanNoMsg:
              print("UNO歸零失敗，請檢查")
              time.sleep(0.1)
      return False
              
        
if __name__ == "__main__":
    claw_machine = ClawMachine()
    try:
      claw_machine.run()
    finally:
        claw_machine.ch.busOff()  # 在 finally 區塊中關閉 CAN 通道
        claw_machine.ch.close()