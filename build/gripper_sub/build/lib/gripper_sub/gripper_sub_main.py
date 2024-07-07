from gripper_sub.subscriber_member_function import pubsub
import rclpy
from rclpy.executors import MultiThreadedExecutor


def main(args=None):
    rclpy.init(args=args)

    pubsub_instance = pubsub()
    # Claw_instance = Claw()
    # 使用 MultiThreadedExecutor 運行節點
    executor = MultiThreadedExecutor()
    executor.add_node(pubsub_instance)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        pubsub_instance.destroy_node()
        pubsub_instance.claw.shutdown()
        rclpy.shutdown()


# def testForGit():
#     print("放下完成")

if __name__ == "__main__":
    main()
