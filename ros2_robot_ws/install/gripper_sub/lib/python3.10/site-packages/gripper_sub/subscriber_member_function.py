# 6/30 整合
import time
import threading
from gripper_sub.claw import Claw
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from robot_interfaces.msg import GripperCommand, GripperInfo
from canlib import canlib, Frame
from gripper_sub.table import ArmCmd
from gripper_sub.table import (
    GripperState,
    GripperInfomation,
    CanData,
    Device,
    Status,
    CanId,
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
        self.pub1stTimeFlag = False
        self.checkTimeStart = 0
        self.currentCmd = ArmCmd.CMD_NO_NEWCMD
        self.claw = Claw()

        # Using ReentrantCallbackGroup
        self.callback_group1 = ReentrantCallbackGroup()
        self.callback_group2 = ReentrantCallbackGroup()
        self.callback_group3 = ReentrantCallbackGroup()
        self.callback_group4 = ReentrantCallbackGroup()
        self.callback_group5 = ReentrantCallbackGroup()
        self.callback_group6 = ReentrantCallbackGroup()

        # Subscriber, subscribe to topic "gripper_command"
        self.subscription = self.create_subscription(
            GripperCommand,
            "/gripper_command",
            self.listener_callback,
            10,
            callback_group=self.callback_group1,
        )

        # # 發布者:回傳 response
        self.publisher_resp = self.create_publisher(
            GripperCommand,
            "/gripper_response",  # 請確認回傳的 topic 名稱
            10,
            callback_group=self.callback_group2,
        )

        # Publisher, publish gripper infomation(state, error...)
        self.publisher_info = self.create_publisher(
            GripperInfo,
            "/gripper_info",  # 請確認回傳的 topic 名稱
            10,
            callback_group=self.callback_group3,
        )

        # 狀態變數 called every 0.01 sec
        self.clawCtrlTimer = self.create_timer(
            0.2, self.clawControll_CallBack, callback_group=self.callback_group4
        )

        # Monitor CAN every  0.05 sec
        self.clawConnectTimer = self.create_timer(
            5, self.clawConnectionCheck_CallBack, callback_group=self.callback_group5
        )

        self.lock = threading.Lock()

        # ************************************************************************************************************#
        # Two dictionaries below are used for next state and to-do task assignment
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

        self.toDoTaskByCmd = {ArmCmd.CMD_STATE_CHECK: self.StateCheckReturn}

        self.toDoTaskByCan = {
            CanData.DATA_UNO_SENSOR_DATA: self.SensorDataProcess,
            CanData.STATE_UNO_CONNECTCHECK: self.ConnectionCheck,
            CanData.STATE_STM_CONNECTCHECK: self.ConnectionCheck,
        }

        # ************************************************************************************************************#
        # ************************************************************************************************************#

        # msg to published when task finished for the first time
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

    # listener callback only processed when new msg on topic "GripperCommand" arrived
    # mainly assigning next state or some flags
    def listener_callback(self, msg):

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

                self.pub1stTimeFlag = True
                self.claw.firstTimeFlag[Device.STM] = True
                self.claw.firstTimeFlag[Device.UNO] = True

                # self.Arm_action()

                ###################
                ####################
                # if msg.num == ArmCmd.CMD_HOLD:
                #     self.currentCmd = ArmCmd.CMD_HOLD_STR
                #     self.Arm_action_grip()

                # elif msg.num == 2:
                #     self.currentCmd = "release"
                #     self.Arm_action_grip()

                # elif msg.num == -1:
                #     raise ValueError("例外發生")
            except ValueError as e:
                print(f"Exception occurred: {e}")

    #  need to modify!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    # def Arm_action(self):

    #     if self.currentCmd == "grab":
    #         print("開始夾取流程")

    #         self.claw.sendSTM_flag = True
    #         self.claw.sendUNO_flag = True
    #         print(self.claw.sendSTM_flag)
    #         print("開始傳")
    #         self.claw.state = "grabbing"
    #     elif self.currentCmd == "release":
    #         print("開始放下流程")

    #         self.claw.sendSTM_flag = True
    #         self.claw.state = "releasing"
    #         self.sendingTimeStart_STM = time.time()

    # state machine
    # read can msg and assign next state(it should be processed like a external interrupt, deal with it somedays~)
    # do the task based on current state
    def clawControll_CallBack(self):

        # ****************************************************************** #
        # First Part, pubsub() process based on ArmCmd and CanMsg received
        # to change state,or do task
        # ****************************************************************** #

        # Can processing part
        # ****************************************************************** #
        with self.lock:
            try:
                self.claw.canMsg = self.claw.ch.read(timeout=100)
                print(self.claw.canMsg.data)
                # print(tuple(self.claw.canMsg.data[0:4]))

                # for nextByCAN
                try:
                    self.claw.state = self.nextByCAN[tuple(self.claw.canMsg.data)]
                    self.pub1stTimeFlag = True
                    self.claw.firstTimeFlag[Device.STM] = True
                    self.claw.firstTimeFlag[Device.UNO] = True
                # for toDoTaskByCan
                except KeyError:
                    self.toDoTaskByCan.get(
                        tuple(self.claw.canMsg.data[0:4]), self.NoTask
                    )()
                    self.claw.state = self.claw.state
            except canlib.canNoMsg:
                self.claw.state = self.claw.state

        # ArmCmd processing part
        # ****************************************************************** #
        with self.lock:
            # for nextByCmd
            try:
                self.claw.state = self.nextByCmd[self.currentCmd]
            # for toDoTaskByCmd
            except KeyError:
                self.toDoTaskByCmd.get(self.currentCmd, self.NoTask)()
                self.claw.state = self.claw.state
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
        # returning true means the task is finished

        infoMsg = GripperInfo()

        if self.currentCmd == ArmCmd.CMD_INIT:
            self.claw.initStatus[Device.UNO] = Status.UNKNOWN

        # claw.toDoTask function return  Status.SUCCESS  Status.FAILED / Status.UNKNOWN
        taskStatus = self.claw.toDoTask.get(self.claw.state, self.claw.NoTask)()

        if taskStatus == Status.SUCCESS and self.pub1stTimeFlag:
            # success task response for the first time
            infoMsg.result = self.clawTaskSuccessInfo.get(self.claw.state, 0)
            if infoMsg.result != 0:
                self.publisher_info.publish(infoMsg)
            with self.lock:
                self.pub1stTimeFlag = False
        elif taskStatus == Status.FAILED and self.pub1stTimeFlag:
            # failed task response for the first time
            infoMsg.result = self.clawTaskFailedInfo.get(self.claw.state, 0)
            if infoMsg.result != 0:
                self.publisher_info.publish(infoMsg)
            with self.lock:
                self.pub1stTimeFlag = False

        with self.lock:
            self.currentCmd = ArmCmd.CMD_NO_NEWCMD

        # for connection check
        if time.time() - self.checkTimeStart > 3 and (
            self.claw.connectStatus[Device.STM] == Status.UNKNOWN
            or self.claw.connectStatus[Device.UNO] == Status.UNKNOWN
        ):
            print("connection failed")
            infoMsg.result = GripperInfomation.GRIPPER_OFFLINE
            self.publisher_info.publish(infoMsg)

        if (
            self.claw.connectStatus[Device.STM] == Status.SUCCESS
            and self.claw.connectStatus[Device.UNO] == Status.SUCCESS
        ):
            print("connection success")
        ########################################################
        ########################################################
        ########################################################
        ########################################################
        ########################################################
        ########################################################
        ########################################################
        ########################################################
        # if self.claw.state == GripperState.STATE_POWER_OFF:
        #     self.claw.power_off()

        # if not self.claw.power_flag:
        #     self.claw.power_on()
        #     self.claw.power_flag = True
        # if self.claw.state == "CheckConnection":
        #     if self.claw.check_connection():
        #         print("連線成功!")
        #         self.claw.state = "Initialization"

        # if self.claw.state == "Initialization":
        #     if self.claw.initialization():
        #         print("初始化成功!")
        #         self.claw.state = "wait_for_command"

        # if self.claw.state == "wait_for_command":
        #     print("等待命令...")
        #     # time.sleep(1)

        # if self.claw.state == "grabbing":
        #     # print("開始夾取流程")
        #     if self.claw.grabbing():
        #         msg = GripperInfo()
        #         msg.result = 1
        #         self.publisher_info.publish(msg)
        #         print("回傳已夾取")
        #         self.claw.state = "wait_for_command"

        # if self.claw.state == "releasing":
        #     # print("開始放下流程")
        #     if self.claw.Release():
        #         msg = GripperInfo()
        #         msg.result = 3
        #         self.publisher_info.publish(msg)
        #         print("回傳已放開")
        #         self.claw.state = "wait_for_command"
        #     else:
        #         print("Release失敗")

        # # elif not ros_flag:
        # #             self.claw.state = "wait_for_command"

        # # if self.claw.state == "hold":
        # #     if self.claw.Hold():
        # #         print("開始夾取!")
        # #         pass

    def clawConnectionCheck_CallBack(self):
        """"""
        # print("connection check sending")
        self.checkTimeStart = time.time()
        self.claw.connectStatus[Device.UNO] == Status.UNKNOWN
        self.claw.connectStatus[Device.STM] == Status.UNKNOWN
        self.claw.ch.write(
            Frame(
                id_=CanId.CANID_PI_TO_ALL,
                data=list(CanData.CMD_PI_CONNECTION_CHECK) + [0, 0, 0, 0],
                dlc=8,
            )
        )

    def ConnectionCheck(self):
        """"""
        if tuple(self.claw.canMsg.data[0:4]) == CanData.STATE_UNO_CONNECTCHECK:
            self.claw.connectStatus[Device.UNO] = Status.SUCCESS
        elif tuple(self.claw.canMsg.data[0:4]) == CanData.STATE_STM_CONNECTCHECK:
            self.claw.connectStatus[Device.STM] = Status.SUCCESS

    def StateCheckReturn(self):
        """"""
        respMsg = GripperCommand()
        respMsg.resp = self.claw.state
        self.publisher_resp.publish(respMsg)

    def SensorDataProcess(self):
        """"""
        # need some value transformation
        self.sensorValue = self.claw.canMsg.data[4]

    def NoTask(self):
        pass

    def shutdown(self):
        self.clawCtrlTimer.cancel()
        self.clawConnectTimer.cancel()
        self.claw.shutdown()
