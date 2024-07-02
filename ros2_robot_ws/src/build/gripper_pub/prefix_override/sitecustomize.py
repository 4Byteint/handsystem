import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pi/handsystem/ros2_robot_ws/src/install/gripper_pub'
