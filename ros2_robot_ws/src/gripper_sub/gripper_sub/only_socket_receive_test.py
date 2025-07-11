import socket
import threading

class SocketReceiver:
    def __init__(self):
        self.host = '127.0.0.1'
        self.port = 5005
        self.x = 0
        self.y = 0
        self.angle = 0
        self.result = False
        self.on_update = None # callback for updates
        self.prev_x = self.prev_y = self.prev_angle = None
        self._lock = threading.Lock() 
        self.server_thread = threading.Thread(target=self.start_socket_server, daemon=True)
        self.server_thread.start()
    
    def is_success(self):
        with self._lock:
            return self.result
   
    def get_pose(self):
        with self._lock:
            return self.x, self.y, self.angle, self.result
        
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
                            self.result = False
                            break
                        try:
                            parts = data.decode('utf-8').strip().split(',')
                            x, y, angle = map(float, parts)
                            if (x, y, angle) == (self.prev_x, self.prev_y, self.prev_angle):
                                continue  # 忽略重複資料
                            
                            with self._lock:
                                self.x, self.y, self.angle = x, y, angle
                                self.result = True
                            
                            self.prev_x, self.prev_y, self.prev_angle = x, y, angle
                            
                            if self.on_update:
                                try:
                                    self.on_update(x,y,angle)
                                    print(f"[socket] 收到資料：x: {self.x:.2f}, y: {self.y:.2f}, angle: {self.angle:.2f}")

                                except Exception as e:
                                    print(f"[socket] callback error：{e}")

                        except Exception as e:
                            print(f"[socket] 解析資料錯誤：{e}")
                            

def main():
    receiver = SocketReceiver()
    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("\n[socket] 程式結束")

if __name__ == '__main__':
    main()