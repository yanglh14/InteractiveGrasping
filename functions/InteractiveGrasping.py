import serial
import pandas as pd
import time
import sys
import os

class Finger():
    def __init__(self,PATH = '/dev/ttyUSB0'):
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

from Gripper.GripperDriver import GripperController
import joblib

model_1 = joblib.load("model/finger_1/final_model_forest_Tz.pkl") # DIFF
model_2 = joblib.load("model/finger_2/final_model_forest_Tz.pkl") # DIF
finger_1 = Finger(PATH = '/dev/ttyUSB2')
finger_2 = Finger(PATH = '/dev/ttyUSB1')
gripper = GripperController(PATH = "/dev/ttyUSB0")

_root_path = '/home/yang/ml/DeepClawDev'
sys.path.append(_root_path)
os.chdir(_root_path)
print('work_dir: ', _root_path)

from deepclaw.driver.arms.URController_rtde import URController
ur10 = URController(_root_path + '/configs/TactileSensor/ur10e.yaml')

RPosition = ur10.get_state()['TCP_Pose']
RPosition[:3] = [-0.658,0.1269,0.50786]
ur10.move_p(RPosition)
gripper.change_mode(3)
gripper.reset()

RPosition = ur10.get_state()['TCP_Pose']
RPosition[2] = RPosition[2]-0.1
ur10.move_p(RPosition)

import time
# gripper.change_mode(0)
# gripper.moving_force([20,0,0])
# Position = gripper.read_position()
# Position[4] = 1550
# Position[3] = 3100
# Position[5] = 3600
# gripper.moving_position(Position,Wait = False)
gripper.change_mode(0)
gripper.moving_force([15,15,15])
time.sleep(3)


# while 1:
#     Tz_1 = model_1.predict([finger_1.GetData()])[0]
#     Tz_2 = model_2.predict([finger_2.GetData()])[0]
#     print(Tz_1,Tz_2)
#     if abs(Tz_2+0.007) < 0.008 and abs(Tz_1-0.015) < 0.008:
#         print('optimized finished')
#         break
#
#     Position = gripper.read_position()
#     print([int(5000 * (Tz_2+0.007)),int(5000*(Tz_1-0.015))])
#     # input("Press Enter to continue...")
#
#     # Cposition = gripper.read_position()
#     # Cposition[4] = Cposition[4] - 100
#     # Cposition[3] = Cposition[3] - 100
#     # Cposition[5] = Cposition[5] - 100
#     #
#     # gripper.moving_position(Cposition,Wait = False)
#     gripper.moving_force([-10,-10,-10])
#     time.sleep(2)
#
#     if abs(Tz_1-0.015) > 0.002:
#         Position[0] = Position[0] + int(5000 * (Tz_1-0.015))
#
#     if abs(Tz_2+0.007) > 0.002:
#         Position[1] = Position[1] + int(5000 * (Tz_2+0.007))
#     gripper.moving_position(Position,Number = 3)
#
#     # Position = gripper.read_position()
#     # Position[4] = 1550
#     # Position[3] = 3100
#     # Position[5] = 3600
#
#     # gripper.moving_position(Position,Wait = False)
#     # time.sleep(1)
#     gripper.moving_force([15,15,15])
#     time.sleep(3)

gripper.moving_force([-3,-3,-3])
time.sleep(1)
gripper.moving_force([10,10,10])
RPosition = ur10.get_state()['TCP_Pose']
RPosition[2] = RPosition[2]+0.1
ur10.move_p(RPosition,0.03,0.03)
time.sleep(3)
gripper.close()
