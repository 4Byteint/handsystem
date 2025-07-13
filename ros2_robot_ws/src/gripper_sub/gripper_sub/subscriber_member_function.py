import time
import threading

import numpy as np
from utils.claw import Claw
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from robot_interfaces.msg import GripperCommand, GripperInfo
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from utils.table import (
    GripperState,
    ArmCmd,
    GripperInfomation,
    CanData,
    CanId,
    Device,
    Status,
)
from .only_socket_receive_test import SocketReceiver

#########################################################################


class pubsub(Node):
    """this class contains all the callback func. and related func in this ROS2 system
    see readme to know more detail"""

    def __init__(self, sock: SocketReceiver):
        super().__init__("pubsub")

        self.sensorValue = 0

        self.pubFirstTimeFlag = False
        self.canFailedFirstTimeFlag = False
        self.stateTaskPrintFlag = True
        self.lastPrintedState = None  # 添加变量来跟踪上一次打印的状态
        self.pendingInfoMsg = None 

        self.currentCmd = ArmCmd.CMD_NO_NEWCMD

        self.preTaskStatus = Status.UNKNOWN
        self.curTaskStatus = Status.UNKNOWN
        
        self.sock = sock
        self.sock.on_update = self.socket_callback
   
        self.waiting_grasp = False
        self.grasp_start_time = None

        # self.delayStart = 0
        # self.time1 = 0
        # self.cmdStartTime = 0
        # self.canStartTime = 0

        self.claw = Claw()
        # Using ReentrantCallbackGroup
        self.callback_group = ReentrantCallbackGroup()

        # Subscriber, subscribe to topic "gripper_command"
        self.subscription = self.create_subscription(
            GripperCommand,
            "/gripper_command",
            self.listener_callback,
            10,
            callback_group=self.callback_group,
        )

        # # # 發布者:回傳 response
        # self.publisher_resp = self.create_publisher(
        #     GripperCommand,
        #     "/gripper_response",  # 請確認回傳的 topic 名稱
        #     10,
        #     callback_group=self.callback_group,
        # )
        
        # Subscriber, subscribe to get vision pose
        # self.subscription_vision = self.create_subscription(
        #     GraspPose,
        #     '/pose_data',
        #     self.grasp_condition_callback,
        #     10,
        #     callback_group=self.callback_group,
        # )
        
        # self.subscription_trans = self.create_subscription(
        #     GraspPose,
        #     '/pose_data',
        #     self.publish_trans_pose_callback,
        #     10,
        #     callback_group=self.callback_group,
        # )
  

        # Publisher, publish gripper infomation(state, error...)
        self.publisher_info = self.create_publisher(
            GripperInfo,
            "/gripper_info",  
            10,
            callback_group=self.callback_group,
        )

        # 狀態變數 called every 0.01 sec
        self.clawCtrlTimer = self.create_timer(
            0.01, self.clawControll_CallBack, callback_group=self.callback_group
        )
        
        # Monitor CAN every  0.05 sec
        self.readCanTimer = self.create_timer(
            0.05, self.readCan_CallBack, callback_group=self.callback_group
        )
        # check can connection every 5 sec
        self.clawConnectTimer = self.create_timer(
            5,
            self.clawCanConnectCheckTask_CallBack,
            callback_group=self.callback_group,
        )
        self.lock = threading.Lock()

        # ************************************************************************************************************#
        # four dictionaries below are used for next state and to-do task assignment
        # ************************************************************************************************************#
        # Cmd from arm and CanMsg from STM or UNO may change state or decides what to do but keep the state unchange
        # State switches depends on nextByCmd and nextByCAN, those not in the dictionary keep the state unchange
        # toDoTaskByCmd and toDoTaskByCan decide what to do when receiving the message
        # ************************************************************************************************************#
        # ************************************************************************************************************#

        # next state after receiving cmd
        # only depends on cmd received
        self.nextByCmd = {
            ArmCmd.CMD_POWEROFF: GripperState.STATE_POWER_OFF,
            ArmCmd.CMD_POWERON: GripperState.STATE_POWER_ON,
            ArmCmd.CMD_INIT: GripperState.STATE_INITIALIZING,
            ArmCmd.CMD_RELEASE: GripperState.STATE_RELEASING,
            ArmCmd.CMD_GRAB: GripperState.STATE_GRABBING,
        }

        # next state after receiving CAN msg
        # only depends on CAN received
        self.nextByCAN = {
            CanData.STATE_STM_MOTOR_OFFLINE: GripperState.STATE_OFFLINE,
            CanData.STATE_STM_GRABBING_MISS: GripperState.STATE_GRABBING_MISS,
            CanData.STATE_STM_RELEASING_MISS: GripperState.STATE_RELEASING_MISS,
            CanData.STATE_STM_GRABBING_MOTOR_ANGLE: GripperState.STATE_GRABBING_MOTOR_ANGLE,
        }

        self.toDoTaskByCmd = {
            ArmCmd.CMD_STATE_CHECK: self.CmdStateCheckTask,
            ArmCmd.CMD_INIT: self.CmdInitTask,
        }

        self.toDoTaskByCan = {
            CanData.DATA_UNO_SENSOR_DATA: self.CanSensorDataTask,
            CanData.STATE_UNO_CONNECTCHECK: self.CanConnectCheckTask,
            CanData.STATE_STM_CONNECTCHECK: self.CanConnectCheckTask,
            # CanData.STATE_STM_INIT_NOTOK: self.CanFailedInterrupt,
        }

        # ************************************************************************************************************#
        # ************************************************************************************************************#
        # msg to published when task finished or failed
        # ************************************************************************************************************#
        # ************************************************************************************************************#
        self.clawTaskSuccessInfo = {
            GripperState.STATE_POWER_OFF: GripperInfomation.NO_INFO,
            GripperState.STATE_POWER_ON: GripperInfomation.NO_INFO,
            GripperState.STATE_INITIALIZING: GripperInfomation.GRIPPER_INITIAL_OK,
            GripperState.STATE_RELEASING: GripperInfomation.GRIPPER_RELEASE,
        }

        self.clawTaskFailedInfo = {
            GripperState.STATE_POWER_OFF: GripperInfomation.NO_INFO,
            GripperState.STATE_POWER_ON: GripperInfomation.NO_INFO,
            GripperState.STATE_INITIALIZING: GripperInfomation.GRIPPER_INITIAL_NOTOK,
        }

    # ********************************************************************* #
    # ********************************************************************* #
    # *************** listener_callback and  related func. **************** #
    # ********************************************************************* #
    # ********************************************************************* #
    def listener_callback(self, msg):
        """focus on cmd ONLY"""
        """listener callback only processed when new msg on topic "GripperCommand" arrived"""
        """mainly assigning next state or some flags"""

        # for delay test
        # info = GripperInfo()
        # info.result = 10
        # self.publisher_info.publish(info)
        # *****************************************
        # for  delay test
        # self.delayStart = time.time()
        # *************************************

        with self.lock:

            print(f'I heard: "{msg.num}"')
            try:
                # get new cmd and change the claw state
                self.currentCmd = msg.num
                print(
                    f'Arm cmd receieved : "{ArmCmd.cmdDict.get(msg.num,"unknown cmd")}"'
                )
                if self.currentCmd == ArmCmd.CMD_GRAB:
                    self.waiting_grasp = True
                if self.currentCmd == ArmCmd.CMD_RELEASE:
                    self.waiting_grasp = False
                if self.currentCmd == ArmCmd.CMD_ERROR:
                    raise ValueError("例外發生")

            except ValueError as e:
                print(f"Exception occurred: {e}")

    # ********************************************************************* #
    # ********************************************************************* #
    # ******** clawCanConnectCheckTask_CallBack and  related func. ******** #
    # ********************************************************************* #
    # ********************************************************************* #
    def clawCanConnectCheckTask_CallBack(self):
        """"""
        with self.lock:
            self.claw.ConnectCheck()

    # ********************************************************************* #
    # ********************************************************************* #
    # **************** readCan_CallBack and  related func. **************** #
    # ********************************************************************* #
    # ********************************************************************* #
    def readCan_CallBack(self):
        """"""
        # use the new-build independent thread to deal with blocking process
        with self.lock:
            threading.Thread(target=self.claw.readCanBlocking).start()

    # ********************************************************************* #
    # ********************************************************************* #
    # ************* clawControll_CallBack and  related func. ************** #
    # ********************************************************************* #
    # ********************************************************************* #
    def clawControll_CallBack(self):
        """state machine, change state & publish info"""
        """assigning next state and do task based on state or messege"""

        # print("timr")
        # print(time.time() - self.time1)
        # self.time1 = time.time()

        #  lock for self.claw.canData, self.claw.connectStatus
        with self.lock:

            # CanData processing part
            # ****************************************************************** #
            # print(self.claw.canData)
            
            try:
                # for nextByCAN
                self.claw.state = self.nextByCAN[self.claw.canData[0:4]]
                self.stateTaskPrintFlag = True
                self.pubFirstTimeFlag = True
                self.claw.sendFirstTimeFlag[Device.STM] = True
                # self.claw.sendFirstTimeFlag[Device.UNO] = True
            except KeyError:
                # for toDoTaskByCan
                self.toDoTaskByCan.get(self.claw.canData[0:4], self.NoTask)()

            # ArmCmd processing part
            # ****************************************************************** #
            # print(self.currentCmd)
            try:
                # for nextByCmd
                self.claw.state = self.nextByCmd[self.currentCmd]
                self.stateTaskPrintFlag = True
                self.pubFirstTimeFlag = True
                self.claw.sendFirstTimeFlag[Device.STM] = True
                # self.claw.sendFirstTimeFlag[Device.UNO] = True
            except KeyError:
                # for toDoTaskByCmd
                self.toDoTaskByCmd.get(self.currentCmd, self.NoTask)()
            #
            # ************************print state************************************* #
            if self.stateTaskPrintFlag:
                current_state = self.claw.state
                # 只有当状态真正改变时才打印
                if self.lastPrintedState != current_state:
                    print(
                        f'Current state: "{GripperState.stateDict.get(current_state,"unknown state")}"'
                    )
                    self.lastPrintedState = current_state
                self.stateTaskPrintFlag = False
            # ********************************************************************** #
            
            self.preTaskStatus = self.curTaskStatus
            # claw.toDoTask function return  Status.SUCCESS/  Status.FAILED/  Status.UNKNOWN
            self.curTaskStatus = self.claw.toDoTask.get(
                self.claw.state, self.claw.NoTask
            )()
          
            if self.preTaskStatus != self.curTaskStatus:
                self.pubFirstTimeFlag = True

            # clean data buffer
            self.claw.canData = CanData.CAN_ERROR_FRAME + (0, 0, 0, 0)
            self.currentCmd = ArmCmd.CMD_NO_NEWCMD

        

        # if self.pubFirstTimeFlag:
        #     if self.pendingInfoMsg is not None:
        #         # 來自 socket_callback 的預備訊息
        #         print("[ROS2] Publishing pending infoMsg from socket")
        #         self.publisher_info.publish(self.pendingInfoMsg)
        #         self.pendingInfoMsg = None
        #         self.pubFirstTimeFlag = False
        #     else:
        #         infoMsg = GripperInfo()
        #         if self.curTaskStatus == Status.SUCCESS:
        #             infoMsg.result = self.clawTaskSuccessInfo.get(
        #                 self.claw.state, GripperInfomation.NO_INFO
        #             )
        #         elif self.curTaskStatus == Status.FAILED:
        #             infoMsg.result = self.clawTaskFailedInfo.get(
        #                 self.claw.state, GripperInfomation.NO_INFO
        #             )
        #         else:
        #             infoMsg.result = GripperInfomation.NO_INFO

        #         if infoMsg.result != GripperInfomation.NO_INFO:
        #             self.publisher_info.publish(infoMsg)
        #             self.pubFirstTimeFlag = False

        # ***************************************************************

    # def CanFailedInterrupt(self):
    #     self.pubFirstTimeFlag = True

    def CmdInitTask(self):
        """"""
        self.claw.initStatus[Device.STM] = Status.UNKNOWN
        # self.claw.initStatus[Device.UNO] = Status.UNKNOWN

    def CmdStateCheckTask(self):
        """"""
        respMsg = GripperCommand()
        respMsg.resp = self.claw.state
        self.publisher_resp.publish(respMsg)

    def CanSensorDataTask(self):
        """"""
        # need some value transformation
        self.sensorValue = self.claw.canData[4]

    def CanConnectCheckTask(self):
        """"""
        self.claw.ConnectStatusUpdate()

    def NoTask(self):
        pass

    # ********************************************************************* #
    # ********************************************************************* #
    # *************************  other  func. ***************************** #
    # ********************************************************************* #
    # ********************************************************************* #
    def socket_callback(self, x, y, angle): # grasp_condition
        with self.lock:
            if self.waiting_grasp:
                # first time entering
                if not self.grasp_start_time:
                    print("[ROS2] Start grasping process")
                    self.grasp_start_time = time.time()
                    self.grasp_pose_counter = 0 
                    
                self.grasp_pose_counter += 1
                # print(f"[ROS2] 收到 pose 第 {self.grasp_pose_counter} 筆")

                if self.grasp_pose_counter <= 3:
                    return
                
                elapsed = time.time() - self.grasp_start_time
                timeout = 5
                
                infoMsg = GripperInfo() 
                transformed_matrix = self.claw.conn2sensor_matrix(x, y, angle)
                infoMsg.data = transformed_matrix.flatten().tolist()
                
                # 成功即發布
                if y >= 34.5 and -90 <= angle <= 90:
                    infoMsg.result = GripperInfomation.GRASP_SUCCESS
                    infoMsg.adjust = 0.0
                   
                elif x == 0 or y == 0 or angle == 0:
                    print("[ROS2] Grasp FAIL")
                    infoMsg.result = GripperInfomation.GRASP_FAIL
                    infoMsg.adjust = 0.0
                elif 0 < y < 34.5 :
                    print("[ROS2] Grasp FAIL, y too small")
                    infoMsg.result = GripperInfomation.GRASP_MISS
                    infoMsg.adjust = 3.5
                    
                # 超時則發布
                elif elapsed > timeout:
                    print("[ROS2] Grasp TIMEOUT without vision pose")
                    infoMsg.result = GripperInfomation.GRASP_FAIL
                    infoMsg.adjust = 0.0
                    self.waiting_grasp = False
                    self.grasp_start_time = None
                else:
                    return
                # cmd 從 grab 轉為 release 時
                self.publisher_info.publish(infoMsg)
                
                
        
        
        
        
    # # Pub to /grasp_pose
    # def publish_trans_pose_callback(self, msg):
    #     with self.lock:
    #         if not self.vision_pose:
    #             print("[ROS2] 尚未接收到影像pose，請檢查程式")
    #             return
    #         if self.vision_pose:
    #             x = self.vision_pose.x
    #             y = self.vision_pose.y
    #             angle = self.vision_pose.angle
            
    #             transformed_matrix = self.claw.conn2sensor_matrix(
    #                 x, y, angle
    #             )

    #             new_msg = GripperInfo()
    #             new_msg.data = transformed_matrix.flatten().tolist()

    #             self.publisher_info.publish(new_msg)
    #         else:
    #             return

    def shutdown(self):
        self.clawCtrlTimer.cancel()
        self.clawConnectTimer.cancel()
        self.readCanTimer.cancel()
        self.claw.shutdown()
