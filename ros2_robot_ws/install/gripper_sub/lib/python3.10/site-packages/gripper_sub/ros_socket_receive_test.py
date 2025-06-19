import rclpy
from rclpy.node import Node
from robot_interfaces.msg import GraspPose
import socket
import threading

class SocketGraspPoseNode(Node):
    def __init__(self):
        super().__init__('socket_grasp_pose_node')

        # 建立 publisher 和 subscriber
        self.publisher_ = self.create_publisher(GraspPose, '/grasp_pose', 10)
        self.subscription = self.create_subscription(
            GraspPose,
            '/grasp_pose',
            self.listener_callback,
            10
        )

        # 啟動 socket server 執行緒
        self.server_thread = threading.Thread(target=self.start_socket_server, daemon=True)
        self.server_thread.start()

        self.get_logger().info("✅ Node 啟動：等待 socket 資料，並訂閱 /grasp_pose")

    def start_socket_server(self, host='127.0.0.1', port=5005):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((host, port))
            s.listen()
            self.get_logger().info(f"🧩 Socket server 啟動：{host}:{port}")

            while True:
                self.get_logger().info("🧩 等待 client 連線...")
                conn, addr = s.accept()
                self.get_logger().info(f"📡 已連線：{addr}")
                with conn:
                    while True:
                        data = conn.recv(1024)
                        if not data:
                            break
                        try:
                            message = data.decode('utf-8').strip()
                            parts = message.split(',')
                            x, y, angle = map(float, parts)
                            print(f"x: {x:.1f}, y: {y:.1f}, angle: {angle:.1f}")
                            # msg = GraspPose()
                            # msg.x = x
                            # msg.y = y
                            # msg.angle = angle
                            # self.publisher_.publish(msg)

                            # self.get_logger().info(f"📤 已發佈 GraspPose: ({msg.x:.2f}, {msg.y:.2f}, {msg.angle:.2f})")
                        except Exception as e:
                            self.get_logger().error(f"解析資料錯誤：{e}")

    def listener_callback(self, msg):
        self.get_logger().info(f"📨 收到 /grasp_pose：x={msg.x:.2f}, y={msg.y:.2f}, angle={msg.angle:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = SocketGraspPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()