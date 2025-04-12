import serial
import numpy as np

MAX_ROTATIONS = 2038
CLOCKWISE = -1
WITTERSHINS = 1

class SerialDevice:
    def __init__(self, com_port:str="COM3", baud_rate:int=9600):
        self.com_port = com_port
        self.baud_rate = baud_rate
        self.serial = serial.Serial(port=com_port, baudrate=baud_rate)
        self.cur_angle = 0
        self.direction = WITTERSHINS

    def gotoAngle(self, angle:float) -> int:
        step = (angle * 2038) // 360
        return self.gotoStep(step)
        

    def gotoStep(self, step:int) -> int:
        while (step < 0):
            step += MAX_ROTATIONS
        while (step > MAX_ROTATIONS):
            step -= MAX_ROTATIONS
        self.serial.write(bytes(f"{step}\n"), 'utf-8')
        self.cur_angle = int(self.serial.readline())
        return self.cur_angle

    def setDirection(self, direction) -> int:
        if self.direction == direction:
            return direction
        
        if direction > 1 or direction < -1:
            return 0
        
        self.serial.write(bytes(f"r\n"), 'utf-8')
        self.direction *= -1
        return int(self.serial.readline())

    def reset(self) -> int:
        self.serial.write(bytes(f"z\n", 'utf-8'))
        self.cur_angle = int(self.serial.readline())
        return self.cur_angle
