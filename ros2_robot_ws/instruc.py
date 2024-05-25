from canlib import canlib, Frame
import time
import sys
import select
import os
import subprocess

class ClawMachine:
    def __init__(self):
        self.state = "PowerOn"  # 初始狀態
        self.ch = canlib.openChannel(
          	channel=0,
          	flags=canlib.Open.EXCLUSIVE,
          	bitrate=canlib.canBITRATE_1M
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
                  print("連線成功!")
                  self.state = "Initialization"  # 連線成功，進入初始化狀態
                  
            elif self.state == "Initialization":
                if self.initialization():
                  print("初始化成功!")
                  self.state = "wait_for_command"
                  
            elif self.state == "wait_for_command":
                if self.wait_for_command():
                  print("收到手臂指令!")
                  self.state = "StartGrabbing"
                  
            elif self.state == "StartGrabbing":
                if self.StartGrabbing():
                  print("開始夾取!")
                else:
                  break
    
    def power_on(self):
        # 上電後的初始化操作，例如檢查電源、啟動系統、檢查各個device是否開啟
        while True:
          # 在背景執行 process 
          process = subprocess.Popen(["python3", "digit_check.py"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
          time.sleep(3)
          if process.poll() is None:
            print("camera is running.")
            self.state = "CheckConnection"
            break
          else:
            print("digit not found")
          #if digit_check.check_boot(): # 檢查視覺是否開啟
            #print("found digit")
            #self.state = "CheckConnection"
          #else:
            #print("digit not found")
    def check_connection(self):
        ask_Boot = Frame(id_=1, data=[0,1,1,1,0,0,0,0] , dlc=8)
        while True:
           user_input = input()
           if user_input == 'a':
             self.ch.write(ask_Boot)
             time.sleep(0.1)
             break
        count = 0
        while count<2:
          try:
            msg = self.ch.read()
            print(msg)
            if (msg.data[2] == 2 or msg.data[2] == 3) and msg.data[3] == 1:
              count += 1
          except canlib.CanNoMsg:
            print("no msg")
            time.sleep(1)
        return True 
        
    def initialization(self):
      # 1. 發送初始化指令
      home_U = Frame(id_=3, data=[0,1,1,2,0,0,0,0], dlc=8)  
      home_STM = Frame(id_=2, data=[0,1,1,3,10,0,0,0] , dlc=8) #初始值10(for STM)
      self.ch.write(home_U)
      self.ch.write(home_STM)
      time.sleep(1)
      # 2. 等待初始化完成
      count = 0
      #while count<2:
      while count<1:
        try:
          msg = self.ch.read()
          print(msg)
          if (msg.data[2] == 2 or msg.data[2] == 3) and msg.data[3] == 2:
            count += 1
        except canlib.CanNoMsg:
            time.sleep(0.1)
      return True
      
    def wait_for_command(self):
        # 等待手臂傳遞開始夾取指令
      print("正在等待手臂指令")
           # 目前假裝有收到手臂id,num
           # arm_id = int(os.environ['ARM_ID'])
           # arm_num = int(os.environ['ARM_NUM'])
           # if arm_id == 4 and arm_num == 1:  
      return True
        #except canlib.CanNoMsg:
            #pass  # 沒有收到命令，繼續等待
      
    def StartGrabbing(self):
        return False
    def Hold(self):
        start_grab = Frame(id_=2, data=[0,1,1,4,50,0,0,0], dlc=8)
        self.ch.write(start_grab)
    def Release(self):
        start_release = Frame(id_=2, data=[0,1,1,5,10,0,0,0], dlc=8)
        self.ch.write(start_release)

if __name__ == "__main__":
    claw_machine = ClawMachine()
    try:
      claw_machine.run()
      # 測試開合
      while True:
        claw_machine.Release()
        time.sleep(3)
        claw_machine.Hold()
        time.sleep(3)
    finally:
        claw_machine.ch.busOff()  
        claw_machine.ch.close()