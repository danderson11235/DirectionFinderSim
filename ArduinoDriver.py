import serial
import numpy as np

MAX_ROTATIONS = 2038
CLOCKWISE = -1
WITTERSHINS = 1

class SerialDevice:
    def __init__(self, com_port="COM3", baud_rate=9600):
        self.com_port = com_port
        self.baud_rate = baud_rate
        self.serial = serial.Serial(port=com_port, baudrate=baud_rate)
        self.cur_angle = 0
        self.direction = WITTERSHINS

    def gotoAngle(self, angle):
        step = (angle * 2038) // 360
        self.cur_angle = self.gotoStep(step)
        return self.cur_angle
        

    def gotoStep(self, step):
        self.serial.write(bytes(f"{step}"), 'utf-8')
        self.cur_angle = int(self.serial.readline())
        return self.cur_angle

    def setDirection(self, direction):
        if self.direction == direction:
            return direction
        
        if direction > 1 or direction < -1:
            return 0
        
        self.serial.write(bytes(f"r"), 'utf-8')
        self.direction *= -1
        return int(self.serial.readline())
