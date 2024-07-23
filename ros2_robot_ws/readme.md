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
        - timer-based, called for a fixed period of time
        - 
    - 






0724
undone :
---

