# can_interface.py
import os
import can
import rclpy
from rclpy.node import Node
from datetime import datetime

class CANInterface(Node):
    def __init__(self, channel='can0', bitrate=1000000):
        super().__init__('can_interface')
        
        self.bus = can.interface.Bus(channel=channel, bustype='socketcan')
        # ROS logger setup
        print(f"[CAN] CAN Initialized on {channel} with {bitrate} bps")

    def send(self, can_id, data):
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
            print(f"[CAN] CAN Sent: ID={hex(can_id)} Data={data}")
        except can.CanError as e:
            print(f"[CAN] CAN Send failed: {e}")

    def receive(self, timeout=1.0):
        """接收 CAN 訊息(有 timeout)"""
        try:
            msg = self.bus.recv(timeout)
            if msg:
                print(f"[CAN] CAN Received: ID={hex(msg.arbitration_id)} Data={list(msg.data)}")
                return msg
        except can.CanError as e:
            print(f"[CAN] CAN Receive failed: {e}")
        return None
    
    def close(self):
        """釋放 CAN bus 資源"""
        try:
            self.bus.shutdown()
            print("[CAN] CAN bus shutdown successfully.")
        except Exception as e:
            print(f"[CAN] CAN bus shutdown error: {e}")

def main():
    rclpy.init()
    can = CANInterface()

    # 發送一筆資料
    can.send(0x01, [0x10, 0x20, 0x30])

    # 等待接收回應
    while True:
        try:
            response = can.receive(timeout=5.0)
            if response:
                print(f"[CAN] Received data: {response.data}")
                break
            else:
                print("[CAN] No data received, continue waiting...")
        except KeyboardInterrupt:
            print("[CAN] Keyboard Interrupt")
            can.close()
            break
    
    rclpy.shutdown()
        
if __name__ == "__main__":
    main()
    