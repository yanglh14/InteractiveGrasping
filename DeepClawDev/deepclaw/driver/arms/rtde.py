import rtde_receive
import rtde_control

import time
import numpy as np


# receive test
try:
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.10")
    print(rtde_receive.isConnected())
except:
    print('HHHHHHHHHHHHHHhhh')
rtde_r.isConnected()
rtde_r.getTargetQ()
rtde_r.getTargetQ()
rtde_r.getTargetQd()
rtde_r.getTargetQdd()
rtde_r.getTargetCurrent()
rtde_r.getTargetMoment()
rtde_r.getActualQ()
rtde_r.getActualQd()
rtde_r.getActualCurrent()
rtde_r.getActualQd()
rtde_r.getActualTCPPose()


t1 = time.time()
for i in range(20):
    # tt1 = time.time()
    # time.sleep(0.005)
    # print(i)
    actual_q = rtde_r.getActualQ()
    actual_q = rtde_r.getActualQ()
    print(actual_q)
    # print('len:', len(kk), 'time:',time.time()-tt1)
t2 = time.time()
print('time:', (t2-t1)/200 )


rtde_c = rtde_control.RTDEControlInterface("192.168.1.10")
s1 = rtde_c.moveL([-0.036, -0.647, 0.5, 2.88, -1.149, -0.125], 1.5, 1.3)
s2 = rtde_c.moveJ([-1.26675971, -1.50360084, -2.01986912, -1.18507832,  1.55369178,1.2], 1.5, 1.3)
# moveJ path {j1,j2,j3,j4,j5,j6,v,a,blend}
path_test = []
pos_j_1 = np.array([-1.26675971, -1.50360084, -2.01986912, -1.18507832,  1.55369178,1.2,1.0,1.0,0.03])
pos_j_2 = np.array([-1.26675971, -1.50360084, -2.01986912, -1.18507832,  2.0,1.2,1.0,1.0,0.03])
pos_j_3 = np.array([-1.26675971, -1.50360084, -2.01986912, -1.18507832,  1.55369178,0.5,1.0,1.0,0.03])
pos_j_4 = np.array([-1.26675971, -1.50360084, -2.01986912, -1.18507832,  1.55369178,1.2,1.0,1.0,0.03])
path_test.append(pos_j_1)
path_test.append(pos_j_2)
path_test.append(pos_j_3)
path_test.append(pos_j_4)
s2 = rtde_c.moveJ(path_test)
rtde_c.moveP([-0.036, -0.647, 0.5, 2.88, -1.149, -0.125], 1.5, 1.3,0.1)

print('==============')
print(s2)
# time.sleep(3)
# kk = rtde_c.stopRobot()
s3 = rtde_c.moveL([-0.036, -0.647, 0.6, 2.88, -1.149, -0.125], 1.5, 1.3)
print(s1,s2,s3)
