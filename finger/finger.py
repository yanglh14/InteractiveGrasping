import serial
import pandas as pd

class Finger():
    def __init__(self,PATH = '/dev/ttyUSB1'):
        self.ser = serial.Serial(PATH, 57600)
        self.ser.flushInput()
        self.data = self.ser.readline().decode()
    def GetData(self):
        self.ser.flushInput()
        # self.data = self.ser.readline().decode()
        self.ser.readlines(1000)
        self.data = self.ser.readline().decode('utf-8')
        self.data = self.data.split(',')
        self.data = list(map(float, self.data))

        return self.data
    def close():
        self.ser.close()

finger = Finger(PATH = '/dev/ttyUSB1')
print(finger.GetData())
