# MIT License.
# Copyright (c) 2020 by BioicDL. All rights reserved.
# Created by LiuXb on 2020/10/11
# -*- coding:utf-8 -*-

"""
@Modified:
@Description:
"""
import numpy as np
import os
import sys
import time
import pandas as pd

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

_root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(_root_path)
os.chdir(_root_path)
print('work_dir: ', _root_path)
#_root_path = '/home/yang/DeepClawDev'

#Initialize robot
from deepclaw.driver.arms.URController_rtde import URController
ur10 = URController(_root_path + '/configs/TactileSensor/ur10e.yaml')
CPosition = ur10.get_state()['TCP_Pose']
HomePosition = CPosition.copy()
HomePosition[:3] = [-0.686,0.607,0.1466]

##Initialize Finger
from Finger import Finger
finger = Finger(PATH = '/dev/ttyUSB0')

##Initialize FTSensor
from FTSensor import FTSensor
FTSensor = FTSensor(IP = '192.168.1.101')

a = []
p_index = 1
m_index = 1

while 1:

    GPosition = HomePosition.copy()
    GPosition[1] = GPosition[1]-0.003*(p_index-1)
    ur10.move_p(GPosition,0.01,0.01)
    index_data = [p_index,0]

    finger_data = finger.GetData()
    force_data = FTSensor.GetData()
    data = index_data + finger_data + force_data
    a.append(data)

    while 1:
        GPosition[0] = GPosition[0]-0.001*m_index
        ur10.move_p(GPosition,0.01,0.01)
        index_data = [p_index,m_index]
        finger_data = finger.GetData()
        force_data = FTSensor.GetData()
        data = index_data + finger_data + force_data
        a.append(data)
        GPosition[0] = GPosition[0]+0.001*m_index
        ur10.move_p(GPosition,0.01,0.01)
        m_index = m_index +1
        if m_index == 11:
            m_index = 1
            break
    print(a)
    p_index = p_index +1
    df = pd.DataFrame(a)
    df.to_csv('projects/proj_TactileSensor/Data/10.csv',index = False, header = ['p_index','m_index','a1','a2','a3','a4','a5','Fy','Fx','Fz','Ty','Tx','Tz'])
    if p_index == 11:
        break
