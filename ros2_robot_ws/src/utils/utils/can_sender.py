# can_interface.py
import can
import logging
from datetime import datetime

class CANInterface:
    def __init__(self, channel='vcan0', bitrate=500000, log_file="can_status.log"):
        self.bus = can.interface.Bus(channel=channel, bustype='socketcan')
        # Log file setup (optional)
        self.logger = logging.getLogger("CAN") 
        self.logger.setLevel(logging.DEBUG) 

        # 設定 log 格式與輸出到檔案
        handler = logging.FileHandler(log_file)
        formatter = logging.Formatter("[%(asctime)s] [CAN] %(message)s", datefmt="%m%d-%H:%M:%S")
        handler.setFormatter(formatter)
        self.logger.addHandler(handler)
        self.logger.info(f"CAN Initialized on {channel} with {bitrate} bps and log file {log_file}")
        print(f"[CAN] CAN Initialized on {channel} with {bitrate} bps and log file {log_file}")

    def send(self, can_id, data):
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
            self.logger.info(f"CAN Sent: ID={hex(can_id)} Data={data}")
            print(f"[CAN] CAN Sent: ID={hex(can_id)} Data={data}")
        except can.CanError as e:
            self.logger.error(f"CAN Send failed: {e}")
            print(f"[CAN] CAN Send failed: {e}")

    def receive(self, timeout=1.0):
        """接收 CAN 訊息(有 timeout)"""
        try:
            msg = self.bus.recv(timeout)
            if msg:
                self.logger.info(f"CAN Received: ID={hex(msg.arbitration_id)} Data={list(msg.data)}")
                print(f"[CAN] CAN Received: ID={hex(msg.arbitration_id)} Data={list(msg.data)}")
                return msg
        except can.CanError as e:
            self.logger.error(f"CAN Receive failed: {e}")
            print(f"[CAN] CAN Receive failed: {e}")
        return None
    
    def close(self):
        """釋放 CAN bus 資源與 log handler"""
        try:
            self.bus.shutdown()
            self.logger.info("CAN bus shutdown successfully.")
            print("[CAN] CAN bus shutdown successfully.")
        except Exception as e:
            self.logger.warning(f"CAN bus shutdown error: {e}")
            print(f"[CAN] CAN bus shutdown error: {e}")

        self.logger.removeHandler(self.log_handler)
        self.log_handler.close()

def main():
    can = CANInterface()

    # 發送一筆資料
    can.send(0x01, [0x10, 0x20, 0x30])

    # 等待接收回應
    while True:
        try:
            response = can.receive(timeout=5.0)
            if response:
                print("接收到資料：", response.data)
                break
            else:
                print("沒有收到資料，繼續等待...")
        except KeyboardInterrupt:
            print("中斷接收")
            can.close()
            break
        
if __name__ == "__main__":
    main()
    