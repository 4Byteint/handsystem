import RPi.GPIO as GPIO
from canlib import canlib, Frame


class Claw:
    def __init__(self):
        self.ch = canlib.openChannel(
            channel=0, flags=canlib.Open.EXCLUSIVE, bitrate=canlib.canBITRATE_1M
        )
        self.ch.setBusOutputControl(canlib.Driver.NORMAL)
        self.ch.busOn()
        GPIO.setmode(GPIO.BOARD)  # BOARD:物理腳位 BCM:GPIO
        GPIO.setup(7, GPIO.OUT)
        GPIO.output(7, False)

    def grabbing(self):
        print("grab")
        grab_action = Frame(id_=2, data=[0, 1, 1, 6, 100, 0, 0, 0], dlc=8)
        self.ch.write(grab_action)

    def release(self):
        print("release")
        release_action = Frame(id_=2, data=[0, 1, 1, 7, 10, 0, 0, 0], dlc=8)
        self.ch.write(release_action)


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
