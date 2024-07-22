# 6/30 整合
import time
import threading
from gripper_sub.claw import Claw
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from robot_interfaces.msg import GripperCommand, GripperInfo
from canlib import canlib, Frame, exceptions
from gripper_sub.table import (
    GripperState,
    ArmCmd,
    GripperInfomation,
    CanData,
    CanId,
    Device,
    Status,
)


#########################################################################
#########################################################################
#########################################################################
#########################################################################
#########################################################################


class pubsub(Node):

    def __init__(self):
        super().__init__("pubsub")

        self.sensorValue = 0
        self.pubFirstTimeFlag = False
        self.checkTimeStart = 0
        self.currentCmd = ArmCmd.CMD_NO_NEWCMD
        self.canMsg = Frame(id_=0, data=[0, 0, 0, 0, 0, 0, 0, 0], dlc=8)

        # self.delayStart = 0
        self.time1 = 0

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

        # # 發布者:回傳 response
        self.publisher_resp = self.create_publisher(
            GripperCommand,
            "/gripper_response",  # 請確認回傳的 topic 名稱
            10,
            callback_group=self.callback_group,
        )

        # Publisher, publish gripper infomation(state, error...)
        self.publisher_info = self.create_publisher(
            GripperInfo,
            "/gripper_info",  # 請確認回傳的 topic 名稱
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

        self.clawConnectTimer = self.create_timer(
            10, self.clawCanConnectCheck_CallBack, callback_group=self.callback_group
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
        }

        self.toDoTaskByCmd = {
            ArmCmd.CMD_STATE_CHECK: self.CmdStateCheckTask,
            ArmCmd.CMD_INIT: self.CmdInitTask,
        }

        self.toDoTaskByCan = {
            CanData.DATA_UNO_SENSOR_DATA: self.SensorDataProcess,
            CanData.STATE_UNO_CONNECTCHECK: self.CanConnectCheck,
            CanData.STATE_STM_CONNECTCHECK: self.CanConnectCheck,
            # CanData.STATE_STM_INIT_NOTOK: self.initNotOk,
        }

        # ************************************************************************************************************#
        # ************************************************************************************************************#
        # msg to published when task finished or failed for the first time
        # ************************************************************************************************************#
        # ************************************************************************************************************#
        self.clawTaskSuccessInfo = {
            GripperState.STATE_POWER_OFF: GripperInfomation.NO_INFO,
            GripperState.STATE_POWER_ON: GripperInfomation.NO_INFO,
            GripperState.STATE_INITIALIZING: GripperInfomation.GRIPPER_INITIAL_OK,
            GripperState.STATE_RELEASING: GripperInfomation.GRIPPER_START_RELEASING,
            GripperState.STATE_GRABBING: GripperInfomation.GRIPPER_START_GRABBING,
            GripperState.STATE_OFFLINE: GripperInfomation.GRIPPER_OFFLINE,
            GripperState.STATE_GRABBING_MISS: GripperInfomation.GRABBING_MISS,
            GripperState.STATE_RELEASING_MISS: GripperInfomation.RELEASING_MISS,
        }

        self.clawTaskFailedInfo = {
            GripperState.STATE_POWER_OFF: GripperInfomation.NO_INFO,
            GripperState.STATE_POWER_ON: GripperInfomation.NO_INFO,
            GripperState.STATE_INITIALIZING: GripperInfomation.GRIPPER_INITIAL_NOTOK,
        }

    def listener_callback(self, msg):
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
                if self.currentCmd == ArmCmd.CMD_ERROR:
                    raise ValueError("例外發生")

                # self.pubFirstTimeFlag = True
                self.claw.firstTimeFlag[Device.STM] = True
                self.claw.firstTimeFlag[Device.UNO] = True

            except ValueError as e:
                print(f"Exception occurred: {e}")

    def clawControll_CallBack(self):
        """state machine"""
        """assigning next state and do task based on state or messege"""

        # ****************************************************************** #
        # First Part, pubsub() process based on ArmCmd and CanMsg received
        # to change claw.state,or do task
        # ****************************************************************** #

        # print("timr")
        # print(time.time() - self.time1)
        # self.time1 = time.time()

        # CanData processing part
        # ****************************************************************** #
        #  lock for self.claw.canData
        with self.lock:

            print(self.claw.canData)
            # for nextByCAN
            try:
                self.claw.state = self.nextByCAN[self.claw.canData[0:4]]
                self.pubFirstTimeFlag = True
                # self.claw.firstTimeFlag[Device.STM] = True
                # self.claw.firstTimeFlag[Device.UNO] = True
            # for toDoTaskByCan
            except KeyError:
                self.toDoTaskByCan.get(self.claw.canData[0:4], self.NoTask)()
            self.claw.canData = CanData.CAN_NO_MSG + (0, 0, 0, 0)

        # ArmCmd processing part
        # ****************************************************************** #
        #  lock for self.currentCmd
        with self.lock:
            # for nextByCmd
            try:
                self.claw.state = self.nextByCmd[self.currentCmd]
                self.pubFirstTimeFlag = True
            # for toDoTaskByCmd
            except KeyError:
                self.toDoTaskByCmd.get(self.currentCmd, self.NoTask)()
            self.currentCmd = ArmCmd.CMD_NO_NEWCMD
        #
        #
        #
        #
        print(
            f'Current state: "{GripperState.stateDict.get(self.claw.state,"unknown state")}"'
        )
        #
        #
        #
        #

        # ************************************************ #
        # Second Part, Claw() process based on state
        # ************************************************ #

        infoMsg = GripperInfo()

        #  this lock dont work here
        with self.lock:

            #
            # if self.currentCmd == ArmCmd.CMD_INIT:
            #     self.claw.initStatus[Device.UNO] = Status.UNKNOWN

            # claw.toDoTask function return  Status.SUCCESS/  Status.FAILED/  Status.UNKNOWN
            taskStatus = self.claw.toDoTask.get(self.claw.state, self.claw.NoTask)()
            # ********************************************************************************************************

            #  新寫法，不確定影響
            # print("pubFirstTimeFlag 1")
            # print(self.pubFirstTimeFlag)
            if self.pubFirstTimeFlag:
                if taskStatus == Status.SUCCESS:
                    infoMsg.result = self.clawTaskSuccessInfo.get(
                        self.claw.state, GripperInfomation.NO_INFO
                    )
                elif taskStatus == Status.FAILED:
                    infoMsg.result = self.clawTaskFailedInfo.get(
                        self.claw.state, GripperInfomation.NO_INFO
                    )
                else:
                    infoMsg.result = GripperInfomation.NO_INFO

                if infoMsg.result != GripperInfomation.NO_INFO:
                    self.publisher_info.publish(infoMsg)
                    self.pubFirstTimeFlag = False
            # print("pubFirstTimeFlag 2")
            # print(self.pubFirstTimeFlag)

            # 原來寫法
            # if taskStatus == Status.SUCCESS and self.pubFirstTimeFlag:
            #     # success task response for the first time
            #     infoMsg.result = self.clawTaskSuccessInfo.get(self.claw.state, GripperInfomation.NO_INFO)
            #     if infoMsg.result != GripperInfomation.NO_INFO:
            #         self.publisher_info.publish(infoMsg)
            #     self.pubFirstTimeFlag = False
            # elif taskStatus == Status.FAILED and self.pubFirstTimeFlag:
            #     # failed task response for the first time
            #     infoMsg.result = self.clawTaskFailedInfo.get(self.claw.state, GripperInfomation.NO_INFO)
            #     if infoMsg.result != GripperInfomation.NO_INFO:
            #         self.publisher_info.publish(infoMsg)
            #     self.pubFirstTimeFlag = False

        # ***************************************************************
        # 未來移植到checkconnection_callback
        # for connection check
        # lock for self.claw.connectStatus
        with self.lock:
            if (
                self.claw.connectStatus[Device.STM] == Status.UNKNOWN
                or self.claw.connectStatus[Device.UNO] == Status.UNKNOWN
            ):
                timePass = time.time() - self.checkTimeStart
                if timePass > 1:
                    self.checkTimeStart = time.time()
                    print("connection resend")
                    self.claw.ch.write(
                        Frame(
                            id_=CanId.CANID_PI_TO_ALL,
                            data=list(CanData.CMD_PI_CONNECTION_CHECK) + [0, 0, 0, 0],
                            dlc=8,
                        )
                    )
            print("connect status")
            print(self.claw.connectStatus[Device.STM])
            print(self.claw.connectStatus[Device.UNO])
            # if (
            #     self.claw.connectStatus[Device.STM] == Status.SUCCESS
            #     and self.claw.connectStatus[Device.UNO] == Status.SUCCESS
            # ):
            #     print("connection success")

            if (
                self.claw.connectStatus[Device.STM] != Status.SUCCESS
                or self.claw.connectStatus[Device.UNO] != Status.SUCCESS
            ):
                print("connection not success")

        # ************************************************************************

    def clawCanConnectCheck_CallBack(self):
        """"""
        with self.lock:
            print("connection check sending")
            self.checkTimeStart = time.time()
            self.claw.connectStatus[Device.UNO] = Status.UNKNOWN
            self.claw.connectStatus[Device.STM] = Status.UNKNOWN
            self.claw.ch.write(
                Frame(
                    id_=CanId.CANID_PI_TO_ALL,
                    data=list(CanData.CMD_PI_CONNECTION_CHECK) + [0, 0, 0, 0],
                    dlc=8,
                )
            )

    def readCanBlocking(self):

        # lock for self.claw.canData
        # with self.lock:
        try:
            # print("start reading can")
            self.canMsg = self.claw.ch.read(timeout=10)
            self.claw.canData = tuple(self.canMsg.data)
            print(self.canMsg)
        except canlib.canNoMsg:
            self.claw.canData = CanData.CAN_NO_MSG + (0, 0, 0, 0)
        except canlib.canError:
            self.claw.canData = CanData.CAN_ERROR + (0, 0, 0, 0)

    def readCan_CallBack(self):
        """"""
        # 使用獨立的執行緒處理阻塞操作
        with self.lock:
            threading.Thread(target=self.readCanBlocking).start()

    def CanConnectCheck(self):
        """"""
        # with self.lock:
        if self.claw.canData[0:4] == CanData.STATE_UNO_CONNECTCHECK:
            self.claw.connectStatus[Device.UNO] = Status.SUCCESS
            print("UNO_CONNNECT_SUCCESS")
        elif self.claw.canData[0:4] == CanData.STATE_STM_CONNECTCHECK:
            self.claw.connectStatus[Device.STM] = Status.SUCCESS
            print("STM_CONNNECT_SUCCESS")

    def initNotOk(self):
        self.pubFirstTimeFlag = True

    def CmdInitTask(self):
        """"""
        self.claw.initStatus[Device.STM] = Status.UNKNOWN
        self.claw.initStatus[Device.UNO] = Status.UNKNOWN

    def CmdStateCheckTask(self):
        """"""
        respMsg = GripperCommand()
        respMsg.resp = self.claw.state
        self.publisher_resp.publish(respMsg)

    def SensorDataProcess(self):
        """"""
        # need some value transformation
        self.sensorValue = self.claw.canData[4]

    def NoTask(self):
        pass

    def shutdown(self):
        self.clawCtrlTimer.cancel()
        self.clawConnectTimer.cancel()
        self.readCanTimer.cancel()
        self.claw.shutdown()
