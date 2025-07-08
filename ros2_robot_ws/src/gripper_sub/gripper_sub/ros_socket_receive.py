import socket
import threading

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import GraspPose
from math import isnan

class SocketReceiver(Node):
    def __init__(self):
        super().__init__('pose_node') # node name
        self.host = '127.0.0.1' 
        self.port = 5005
        
        self.publisher_ = self.create_publisher(
            GraspPose, 
            '/pose_data', 
            10
        )
        
        self.server_thread = threading.Thread(target=self.start_socket_server, daemon=True)
        self.server_thread.start()

    def start_socket_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.host, self.port))
            s.listen()
            print(f"[socket] Socket server 啟動：{self.host}:{self.port}")

            while True:
                print("[socket] 等待 client 連線...")
                conn, addr = s.accept()
                print(f"[socket] 已連線：{addr}")
                with conn:
                    while True:
                        data = conn.recv(1024)
                        if not data:
                            break
                        try:
                            message = data.decode('utf-8').strip()
                            parts = message.split(',')
                            x, y, angle = map(lambda v: float(v) if v.lower() != 'nan' else float('nan'), parts)
                            
                            print(f"[socket] 收到資料：x: {x:.2f}, y: {y:.2f}, angle: {angle:.2f}")
                            if not (isnan(x) or isnan(y) or isnan(angle)):
                                msg = GraspPose()
                                msg.x = x
                                msg.y = y
                                msg.angle = angle
                                self.publisher_.publish(msg)
                 
                        except Exception as e:
                            print(f"[socket] 解析資料錯誤：{e}")

def main():
    rclpy.init()
    receiver = SocketReceiver()
    try:
        rclpy.spin(receiver)
    except KeyboardInterrupt:
        print("\n[socket] 程式結束")
    finally:
        receiver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
