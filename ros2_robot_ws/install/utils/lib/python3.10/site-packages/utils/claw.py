import time

import numpy as np
from .table import (
    GripperState,
    ArmCmd,
    GripperInfomation,
    CanData,
    CanId,
    Device,
    Status,
)
import os
import can
from .can_sender import CANInterface
from robot_interfaces.msg import  GraspPose

class Claw:
    """This class handles all CAN-related tasks using python-can."""
    def __init__(self, channel='can0', bitrate=1000000, log_file="can_status.log"):
        """ init CAN channel and set up initial state """
        os.system("sudo ip link set can0 down")
        os.system("sudo ip link set can0 type can bitrate 1000000")
        os.system("sudo ip link set can0 up")
        
        self.can = CANInterface(channel=channel, bitrate=bitrate)

        self.connectCheckTimeStart = 0
        self.sendingTimeStart_STM = 0
        # self.sendingTimeStart_UNO = 0

        self.connectCheckPrintFlag = True
        
        # public
        self.canData = (0, 0, 0, 0, 0, 0, 0, 0)
        self.state = GripperState.STATE_POWER_OFF  # 初始狀態
        self.R_j62sensor = np.zeros((4, 4))
        self.R_sensor2conn = np.zeros((4, 4))

        #  True for CAN  been transmitted for the first time
        self.sendFirstTimeFlag = {Device.STM: False, Device.UNO: False}
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
            GripperState.STATE_GRABBING_MOTOR_ANGLE: self.GrabbingMotorAngle,
        }

    # ********************************************************************* #
    # ********************************************************************* #
    # ************************ Can Reading ******************************** #
    # ********************************************************************* #
    # ********************************************************************* #
    def readCanBlocking(self, timeout=0.01):
        """a blocking CAN data reading method"""
        try:
            msg = self.can.receive(timeout=timeout)
            if msg:
                self.canData = tuple(msg.data)
                # print(f"[Claw] claw Received: ID={hex(msg.arbitration_id)} Data={list(msg.data)}")
            else:
                # print("[Claw] No CAN message received within timeout.")
                self.canData = CanData.CAN_ERROR_FRAME + (0, 0, 0, 0)
        except can.CanError as e:
            print(f"[Claw] CAN read error: {e}")
            self.canData = CanData.CAN_ERROR_FRAME + (0, 0, 0, 0)
            

    # ********************************************************************* #
    # ********************************************************************* #
    # *************** Connection Check Related Func.*********************** #
    # ********************************************************************* #
    # ********************************************************************* #

    def ConnectCheck(self):
        timePass = time.time() - self.connectCheckTimeStart
        connction_data = list(CanData.CMD_PI_CONNECTION_CHECK) + [0, 0, 0, 0]
        # do the connect check every 10sec
        if timePass > 10:
            print("[Claw] Check Conncetion sending(per 10 sec)")
            self.connectCheckTimeStart = time.time()
            self.connectCheckPrintFlag = True
            # self.connectStatus[Device.UNO] = Status.UNKNOWN
            self.connectStatus[Device.STM] = Status.UNKNOWN
            self.can.send(CanId.CANID_PI_TO_ALL, connction_data)
        # reconnecting every 1 sec if one of devices is still unchecked
        elif timePass > 1:
            if (
                self.connectStatus[Device.STM] == Status.UNKNOWN
                # or self.connectStatus[Device.UNO] == Status.UNKNOWN
            ):  
                print("[Claw] not receive resp, Conncetion resend(per 1 sec)")
                self.connectCheckTimeStart = time.time()
                self.connectCheckPrintFlag = True
                self.can.send(CanId.CANID_PI_TO_ALL, connction_data)
   
        if self.connectCheckPrintFlag:
            # print(self.connectStatus[Device.STM])
            # print(self.connectStatus[Device.UNO])
            if self.connectStatus[Device.STM] == Status.SUCCESS: # and uno_ok:
                print("[Claw] All devices Connection check success")
            else:
                print("[Claw] Connection check not success")
            
            self.connectCheckPrintFlag = False

    def ConnectStatusUpdate(self):
        """called before self.canData cleared or covered"""
        # if self.canData[0:4] == CanData.STATE_UNO_CONNECTCHECK:
        #     self.connectStatus[Device.UNO] = Status.SUCCESS
        #     print("UNO_CONNNECT_SUCCESS")
        if self.canData[0:4] == CanData.STATE_STM_CONNECTCHECK:
            self.connectStatus[Device.STM] = Status.SUCCESS
            print("[Claw] STM_CONNNECT_SUCCESS")

    # ********************************************************************* #
    # ********************************************************************* #
    # ********************* State Machine Task **************************** #
    # ********************************************************************* #
    # Every state has its own state-task todo, task return class "Status"   #
    # when it's done.   *************************************************** #
    # ********************************************************************* #

    def PowerOff(self):
        """tell STM to turn off motor"""
        # print("power off")

        return Status.SUCCESS

    def PowerOn(self):
        """"""
        # print("power on")

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

        # wait for connection check
        if (
            # self.connectStatus[Device.UNO] == Status.SUCCESS and
            self.connectStatus[Device.STM] == Status.SUCCESS
        ):
            # **************************************************************************************
            # automatically go into init state
            self.state = GripperState.STATE_INITIALIZING
            # self.sendFirstTimeFlag[Device.UNO] = True
            self.sendFirstTimeFlag[Device.STM] = True
            return Status.SUCCESS

    def Initialization(self):
        # print("init")

        # if self.sendFirstTimeFlag[Device.UNO]:
        #     print("init UNO send")
        #     self.can.send(
        #         CanId.CANID_PI_TO_UNO,
        #         list(CanData.CMD_PI_UNO_INIT) + [0, 0, 0, 0],
        #     )
        #     self.sendFirstTimeFlag[Device.UNO] = False
        #     self.sendingTimeStart_UNO = time.time()
        # else:
        #     received = self.canData[0:4]
        #     if received == CanData.STATE_UNO_INIT_OK:
        #         print("UNO INIT SUCCESS")
        #         self.initStatus[Device.UNO] = Status.SUCCESS
        #     elif received == CanData.STATE_UNO_INIT_NOTOK:
        #         print("UNO INIT FAILED")
        #         self.initStatus[Device.UNO] = Status.FAILED
        #     elif time.time() - self.sendingTimeStart_UNO > 1.5: # unit: seconds
        #         print("NOT receive UNO INIT, resend")
        #         self.sendFirstTimeFlag[Device.UNO] = True
        #         self.initStatus[Device.UNO] = Status.FAILED

        if self.sendFirstTimeFlag[Device.STM]:
            print("[Claw] init STM send")
            self.can.send(
                CanId.CANID_PI_TO_STM,
                list(CanData.CMD_PI_STM_INIT) + [10, 0, 0, 0],
            )
            self.sendFirstTimeFlag[Device.STM] = False
            self.sendingTimeStart_STM = time.time()
        else:
            received = self.canData[0:4]
            if received == CanData.STATE_STM_INIT_OK:
                print("[Claw] STM INIT SUCCESS")
                self.initStatus[Device.STM] = Status.SUCCESS
            elif received == CanData.STATE_STM_INIT_NOTOK:
                print("[Claw] STM INIT FAILED")
                self.initStatus[Device.STM] = Status.FAILED
            elif time.time() - self.sendingTimeStart_STM > 1.5:
                print("[Claw] NOT receive STM INIT, resend")
                self.sendFirstTimeFlag[Device.STM] = True
                self.initStatus[Device.STM] = Status.FAILED
        # check if both devices are initialized
        if (
            self.initStatus[Device.STM] == Status.SUCCESS
            # and self.initStatus[Device.UNO] == Status.SUCCESS
        ):
            return Status.SUCCESS
        elif (
            self.initStatus[Device.STM] == Status.FAILED
            # or self.initStatus[Device.UNO] == Status.FAILED
        ):
            return Status.FAILED
        else:
            return Status.UNKNOWN

    def Grab(self):
        """tell:
        1. UNO return sensor data(optional) 
        2. CAM to watch for grabbing 
        3. STM to start grabbing
        """
        # if self.sendFirstTimeFlag[Device.UNO]:
        #     self.can.send(
        #         CanId.CANID_PI_TO_UNO,
        #         list(CanData.CMD_PI_SENSOR_REQUEST) + [0, 0, 0, 0],
        #     )
        #     self.sendFirstTimeFlag[Device.UNO] = False
    
        #  remove if and keep else if you want to send STM cmd Constantly
        if self.sendFirstTimeFlag[Device.STM]:

            # *******************************************
            # self.sendingTimeStart_STM = time.time()
            # *******************************************
            self.can.send(
                CanId.CANID_PI_TO_STM,
                list(CanData.CMD_PI_GRABBING) + [200, 0, 0, 0],
            )
            self.sendFirstTimeFlag[Device.STM] = False

            # ******************************************
            # self.sendingTimeStart_STM = time.time()
            # ******************************************

        else:
            try:
                if self.canData[0:4] == CanData.STATE_STM_START_GRABBING:

                    # *************************************************
                    # canTime = time.time() - self.sendingTimeStart_STM
                    # with open("CanToCanDelay.txt", "a") as file:
                    #     file.write(f"{canTime}\n")
                    # *************************************************

                    print("[Claw] STM start grabbing success")
                    return Status.SUCCESS
            except can.CanError as e:
                self.sendFirstTimeFlag[Device.STM] = True
                print(f"[Claw] 未收到STM是否開夾,重新發送 ,{e}")
                return Status.FAILED
          

    def Release(self):
        if self.sendFirstTimeFlag[Device.STM]:
            self.can.send(
                CanId.CANID_PI_TO_STM,
                list(CanData.CMD_PI_RELEASING) + [10, 0, 0, 0],
            )
            
            self.sendFirstTimeFlag[Device.STM] = False
            # self.sendingTimeStart_STM = time.time()
            # print(time.time() - self.sendingTimeStart_STM)
        else:
            # try:

            if self.canData[0:4] == CanData.STATE_STM_START_RELEASING:
                return Status.SUCCESS

            # except canlib.CanNoMsg:
            #     # if time.time() - self.sendingTimeStart_STM > 5:
            #     self.sendFirstTimeFlag[Device.STM] = True
            #     print("未收到STM是否放開,重新發送")
            #     return Status.UNKNOWN

    def OffLine(self):
        """nothing to do with STM,UNO"""
        # print("offline")
        return Status.SUCCESS

    def GrabbingMiss(self):
        """nothing to do with STM,UNO"""
        return Status.SUCCESS

    def ReleasingMiss(self):
        """nothing to do with STM,UNO"""
        # print("releasing miss")
        return Status.SUCCESS

    # ********************************************************************* #
    # ********************************************************************* #
    # ********************* return motor data **************************** #
    # ********************************************************************* #
    # ********************************************************************* #
    
    
    def calculate_transfer_matrix(self, motor_deg):
        L1 = 15
        L2 = 32.5
        offset = -27.5
        theta = np.radians(motor_deg) # radius
        dz = (
            L1 * np.cos(theta)
            + L2 * np.cos(np.arcsin(np.sin(theta) / L2))
            + offset
        )

        # print("dz= ", dz)
        
        self.R_j62sensor = np.array([
            [0, 0, 1, 0],
            [1, 0, 0, 201.62],
            [0, 1, 1, dz],
            [0, 0, 0, 1]
        ])
    
    def calculate_grasp_pose(self, x, y, vision_deg):
        vision_deg = np.radians(vision_deg)
        tx = x
        ty = y
        tz = 2.2
        cos_a = np.cos(vision_deg)
        sin_a = np.sin(vision_deg)
        R_sensor2conn = np.array([
            [cos_a, -sin_a, 0, tx],
            [sin_a, cos_a, 0, ty],
            [0, 0, 1, tz],
            [0, 0, 0, 1]     
        ])
        
        
        P_transformed = self.R_j62sensor @ R_sensor2conn
        trans_pose = P_transformed[:3, 0]
        # print(f"[ROS2] 轉換後的座標：{trans_pose}")
        return trans_pose
        
    def GrabbingMotorAngle(self):
        """coordinate the motor angle"""
        # print("[Claw] GrabbingMotorAngle")
        received = self.canData[0:4]
        if received == CanData.STATE_STM_GRABBING_MOTOR_ANGLE:
            # print(f"[ROS2] get motor angle data: {self.canData[-1]}")
            self.calculate_transfer_matrix(self.canData[-1]) # 更新 self.R_j62sensor
            
            return Status.SUCCESS
        

    # ********************************************************************* #
    # ********************************************************************* #
    # ************************* Other Task Func. ************************** #
    # ********************************************************************* #
    # ********************************************************************* #
    def NoTask(self):
        """nothing to do with STM,UNO"""
        return Status.SUCCESS

    def shutdown(self):
        self.can.close()