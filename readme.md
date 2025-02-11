Middle level of management of Hand System
---
### Overview
1. ROS2 used to communicate with high level system (robotic arm)
2. CANbus used to give orders to the lower level system
3. 
---
### ROS2 
1. Topic : 
    - gripper_command : 
        - Messege received from Arm-Rpi, mainly order or request.
    - gripper_response : 
        - Messege send to Arm-Rpi after a request received.
    - gripper_inform : 
        - Messege send to Arm-Rpi when some events or exception that we must tell Arm-Rpi happen.
2. Multi-threading :
    - ReentrantCallbackGroup( ) are used to achived Multi-threading.
3. Callback Function :
    - listener_callback :
        - Called when a new messege comes to the subscribed topic
        - Used to receive messege from Arm-Rpi
    - clawControll_CallBack :
        - Timer-based, called every a fixed period of time
        - Basically a state machine updating state by receieved CAN-msg or Cmd from Arm_Rpi
        - Publish messege if needed
    - readCan_CallBack :
        - Timer-based, called every a fixed period of time
        - Note that Kvaser has a bloking reading function, so we establish a new threading in this callback to avoid blocking the others.
    - clawCanConnectCheckTask_CallBack :
        - Timer-based, called every a fixed period of time
        - Send the connecting check request every a fixed period of time
        - Resend the connecting check request if the check has been failed

### table.py
- this file contains all the basic information and are used as constant class in the program 





0724
undone :
---
1. Motor offline state processing
2. Sensor Data processing and file conversion
3. connection check failed processing(ex : if beeen failed for more than 5 times)

