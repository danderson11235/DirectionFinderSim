import serial
import numpy as np

MAX_ROTATIONS = 2038
# Directions
CLOCKWISE = -1
WITTERSHINS = 1
# Modes
CONTINUIOUS = 0
BOUNDED = 1

class SerialDevice:
    def __init__(self, com_port:str="COM3", baud_rate:int=9600, timeout:int=30):
        self.com_port = com_port
        self.baud_rate = baud_rate
        self.serial = serial.Serial(port=com_port, baudrate=baud_rate, timeout=timeout)
        self.cur_angle = 0
        self.direction = WITTERSHINS
        self.mode = CONTINUIOUS

    def gotoContAngle(self, angle:float) -> int:
        step = (angle * MAX_ROTATIONS) // 360
        return self.gotoContStep(step)

    def gotoContStep(self, step:int) -> int:
        self.setMode(CONTINUIOUS)
        while (step < 0):
            step += MAX_ROTATIONS
        while (step > MAX_ROTATIONS):
            step -= MAX_ROTATIONS
        self.serial.write(bytes(f"{step}\n", 'utf-8'))
        self.cur_angle = int(self.serial.readline())
        return self.cur_angle

    def gotoBoundAngle(self, angle:float) -> int:
        step = (angle * MAX_ROTATIONS) // 360
        return self.gotoBoundStep(step)

    def gotoBoundStep(self, step:int) -> int:
        self.setMode(BOUNDED)
        self.serial.write(bytes(f"{step}\n", 'utf-8'))
        self.cur_angle = int(self.serial.readline())
        return self.cur_angle


    def setDirection(self, direction) -> int:
        if self.direction == direction:
            return direction
        
        if direction > 1 or direction < -1:
            return 0
        
        self.serial.write(bytes(f"r\n", 'utf-8'))
        self.direction *= -1
        return int(self.serial.readline())

    def setMode(self, mode) -> str:
        if self.mode == mode:
            return ""
        if mode == CONTINUIOUS:
            self.serial.write(bytes("mc\n", 'utf-8'))
        elif mode == BOUNDED:
            self.serial.write(bytes("mb\n", 'utf-8'))
        return self.serial.readline()


    def reset(self) -> int:
        self.serial.write(bytes(f"z\n", 'utf-8'))
        self.cur_angle = int(self.serial.readline())
        return self.cur_angle

    def setSpeed(self, speed:int) -> int:
        self.serial.write(bytes(f"s {speed}\n", 'utf-8'))
        self.cur_angle = int(self.serial.readline())
        return self.cur_angle

    def gotoBoundStepFast(self, step):
        self.setMode(BOUNDED)
        self.serial.write(bytes(f"step\n", 'utf-8'))
        # self.cur_angle = int(self.serial.readline())
        self.needsRead = 1
        return self.cur_angle

