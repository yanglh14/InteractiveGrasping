# MIT License.
# Copyright (c) 2020 by BioicDL. All rights reserved.
# Created by LiuXb on 2020/5/19
# -*- coding:utf-8 -*-

"""
@Modified: 
@Description:
"""
from URController_rtde import URController
import time
# strat_pose = [-0.31166,-0.60553, 0.37510,3.14,0,-1.57] euler
# strat_pose = [-0.31166,-0.60553, 0.37510,2.2216,-2.2198,-0.0017]
#
# end_pose = [0.3,-0.608, 0.37510,2.2216,-2.2198,-0.0017]

robot = URController('../../../configs/robcell-ur5-rg6-d435/franka.yaml')
stop_pose =  [0,-0.60553, 0.5,2.2216,-2.2198,-0.0017]

# robot.move_L(stop_pose,0.5,0.5)
# 0
start_0 = [-0.32,-0.609, 0.334,2.2216,-2.2198,-0.0017]
end_0 = [0.3,-0.61, 0.33,2.2216,-2.2198,-0.0017]
# 30

# 15

# 45
start_45 = [-0.31,-0.605, 0.392,1.7620,-1.7571,-0.7328]
end_45 =   [0.35,   -0.607, 0.39,1.7620,-1.7571,-0.7328]
# 60
start_60 = [-0.33,-0.605, 0.378,1.5855,-1.5855,-0.9174]
end_60 = [0.37,-0.607, 0.373,1.5855,-1.5855,-0.9174]
cnt = 1
velocity = 0.45
a = 2
angle_1 = 0.78549
angle_2 = 1.04732

# for i in range(cnt):
#     t1 = time.time()
#     robot.move_L(start_60,velocity,a)
#     t2 = time.time()
#     robot.move_L(end_60,velocity,a)
#     t3 = time.time()
#     print('t1:',t2-t1, ' t2:',t3-t2)
# robot.move_L(stop_pose,0.5,0.5)


# for i in range(cnt):
#     t1 = time.time()
#     robot.move_L(start_45,velocity,a)
#     t2 = time.time()
#     robot.move_L(end_45,velocity,a)
#     t3 = time.time()
#     print('t1:',t2-t1, ' t2:',t3-t2)
# robot.move_L(stop_pose,0.5,0.5)


for i in range(cnt):
    t1 = time.time()
    robot.move_L(start_0,velocity,a)
    time.sleep(0.1)

    t2 = time.time()
    robot.move_L(end_0,velocity,a)
    t3 = time.time()
    print('t1:',t2-t1, ' t2:',t3-t2)
# robot.move_L(stop_pose,0.5,0.5)


# speed_c = [,0,0,0,0,0]
# for i in range(cnt):
#     robot.speed_Ls






