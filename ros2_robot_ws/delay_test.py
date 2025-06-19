import RPi.GPIO as GPIO
import can

class Claw:
    def __init__(self):
        # 初始化 CAN 总线
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000)
        GPIO.setmode(GPIO.BOARD)  # BOARD:物理腳位 BCM:GPIO
        GPIO.setup(7, GPIO.OUT)
        GPIO.output(7, False)

    def grabbing(self):
        print("grab")
        # 创建 CAN 消息
        grab_msg = can.Message(
            arbitration_id=2,
            data=[0, 1, 1, 6, 100, 0, 0, 0],
            is_extended_id=False
        )
        self.bus.send(grab_msg)

    def release(self):
        print("release")
        # 创建 CAN 消息
        release_msg = can.Message(
            arbitration_id=2,
            data=[0, 1, 1, 7, 10, 0, 0, 0],
            is_extended_id=False
        )
        self.bus.send(release_msg)

    def __del__(self):
        # 确保在对象销毁时关闭 CAN 总线
        if hasattr(self, 'bus'):
            self.bus.shutdown()


def main(args=None):
    Claw_instance = Claw()
    try:
        while True:
            input_var = input("請輸入開始鍵a 或 b: ")
            if input_var == "a":
                Claw_instance.grabbing()
                GPIO.output(7, True)
            elif input_var == "b":
                Claw_instance.release()
                GPIO.output(7, False)
            else:
                print("無效的輸入，請輸入 a 或 b")
    except KeyboardInterrupt:
        print("退出程序")
    finally:
        GPIO.cleanup()


if __name__ == "__main__":
    main()
