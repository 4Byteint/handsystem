Middle level of management of Hand System
---
### Overview
1. ROS2 used to communicate with high level system (robotic arm)
2. CANbus used to give orders to the lower level system
3. 
---
### ROS2
1. two topic, each has: 
    - gripper_command
    - gripper_response ?

### state
兩種state:claw要做事與純回傳


要做的事情被主要分類為
claw負責與pubsub負責
claw負責和底下STM UNO有關的事情，通常是給他們指令、判斷他們回傳的東西等，由claw.state決定
pubsub負責主流程，手臂端的指令、例外處裡等


0707
undone :
---
- read can data in clawControll_CallBack(or using timer canRead_CallBack), try the easy way first
- can msg happened such that state change, put in the first few line of  clawControll_CallBack
- CANDATA class not yet finished
- clawConnectionCheck_CallBack for check connection every 0.05sec
- taskDoneInfo not yet finished
- nextByCAN not yet finished , seems not necessary
- toDoTask and corresponding function not yet finished
- Arm_action and first time flag problem
