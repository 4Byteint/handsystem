import socket
import threading
import time

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
        self.filter_mx = self.EMA(alpha=0.05)
        self.filter_my = self.EMA(alpha=0.05)
        self.filter_angle = self.EMA(alpha=0.05)
        # time / log
        self.prev_ts = None          # 上一次收到資料的時間點
        self.dts_ms  = []            # 所有 Δt (ms) 以便最後算平均
        self.log_f   = open('socket_time_log.txt', 'w', buffering=1)   # 行緩衝，自動 flush

    class EMA:
      def __init__(self, alpha=0.2):
          self.alpha = alpha
          self.value = None  # 初始沒有值

      def update(self, new_value):
          if self.value is None:
              self.value = new_value
          else:
              self.value = self.alpha * new_value + (1 - self.alpha) * self.value
          return self.value
    
    
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
                
                self._handle_connection(conn)
                # finally:
                #     self._write_average_and_reset()
                    
    def _handle_connection(self, conn):
        buffer = ""
        with conn:
            while True:
                data = conn.recv(1024)
                if not data:
                    self.result = False
                    break
                
                buffer += data.decode("utf-8")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    try:
                        parts = line.strip().split(',')
                        x, y, angle = map(float, parts)
                        now = time.monotonic()
                        if self.prev_ts is None:
                            dt_ms = 0.0
                        else:
                            dt_ms = (now - self.prev_ts) * 1000
                            # 紀錄 Δt,第一筆記0
                            self.dts_ms.append(dt_ms)
                            self.log_f.write(f"{dt_ms:.2f}\n")
                        self.prev_ts = now
                        # 更新姿態
                        with self._lock:
                            self.x, self.y, self.angle = x, y, angle
                            self.result = True
                        # self.prev_x, self.prev_y, self.prev_angle = x, y, angle
                        
                        if self.on_update:
                            try:
                                if x != 0 or y != 0 or angle != 0:
                                    # 使用 EMA 平滑處理
                                    x = self.filter_mx.update(x)
                                    y = self.filter_my.update(y)
                                    angle = self.filter_angle.update(angle)
                                self.on_update(x,y,angle)
                                print(f"[socket] 收到資料：x: {x:.2f}, y: {y:.2f}, angle: {angle:.2f}")
                                

                            except Exception as e:
                                print(f"[socket] callback error：{e}")

                    except Exception as e:
                        print(f"[socket] 解析資料錯誤：{e}")
                    
    # def _write_average_and_reset(self):
    #     if self.dts_ms:
    #         avg_dt = sum(self.dts_ms) / len(self.dts_ms)
    #         print(f"[socket] 平均 Δt: {avg_dt:.2f} ms")
    #         self.log_f.write(f"平均 Δt: {avg_dt:.2f} ms\n")
    #     else:
    #         print("[socket] 沒有收到任何資料")
    #         self.log_f.write("沒有收到任何資料\n")
        
        # 重置狀態，便下一次連線
        self.prev_ts = None
        self.dts_ms.clear()


def main():
    receiver = SocketReceiver()
    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("\n[socket] 程式結束")

if __name__ == '__main__':
    main()