# from canlib import canlib, Frame  
from threading import Thread
from multiprocessing import Process
import time

class Robot:
    def __init__(self, ch, aa, bb):
        self.ch = ch
        self.aa = aa
        self.bb = bb

    def init_task(self):
        while True:
            print(f'init...')
            time.sleep(1)

    def grabbing_task(self):
        while True:
            print(f'grabbing task : {self.ch}')
            time.sleep(1)

    def release_task(self):
        while True:
            print(f'release task : {self.ch}')
            time.sleep(1)



class Task():

    def __init__(self, robot):
        self.task = Process(target=robot.init_task)
        self.task.start()

    def stop(self):
        self.task.terminate()

    def change_task(self, task):
        self.stop()
        self.task = Process(target=task)
        self.task.start()

    


def main():

    robot = Robot('ch', 'aa', 'bb')
    curtask = Task(robot)

    while True:
        cmd = input('input command: ')
        if cmd == '1':
            curtask.change_task(robot.grabbing_task)
        elif cmd == '2':
            curtask.change_task(robot.release_task)
        else:
            curtask.stop()
            break

        


if __name__ == '__main__':
    main()