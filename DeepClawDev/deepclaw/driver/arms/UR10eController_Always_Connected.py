# Copyright (c) 2019 by liuxiaobo. All Rights Reserved.
# !/usr/bin/python
# coding=utf-8
import numpy as np
import socket
import struct
import time
import math
import sys
import os

from queue import Queue
import threading

# _root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# sys.path.append(_root_path)

# from Driver.Controller import Controller
# from ToolKit.Configuration import *
# from ToolKit.Calibration3D import *

# Copyright (c) 2019 by Hank. All Rights Reserved.
# !/usr/bin/python
# coding=utf-8
import tf
class Controller(object):
    def __init__(self):
        self.HOME_POSE = [[0, 0, 0], [0, 0, 0]]
        self.PICK_Z = 0
        self.PLACE_Z = 0

    def goHome(self):
        raise NotImplementedError(' goHome method does not implement. ')

    def execute(self, group, plan):
        raise NotImplementedError(' execute method does not implement. ')

    def move(self, goal_pose):
        raise NotImplementedError(' move method does not implement. ')

    def openGripper(self):
        raise NotImplementedError(' openGripper method does not implement. ')

    def closeGripper(self):
        raise NotImplementedError(' closeGripper method does not implement. ')

    # def calibrating(self):
    #     raise NotImplementedError(' calibrating method does not implement. ')

    def rpy2orientation(self, row, pitch, yaw):
        q = tf.transformations.quaternion_from_euler(row, pitch, yaw, axes='sxyz')
        return q
        
class UR10eController(Controller):
    def __init__(self):
        super(UR10eController, self).__init__()
        # self.cfg = readConfiguration('ur10e')
        self._robot_ip = "192.168.1.10"
        self._port = 30003
        self._home_pose = [[0.03,-0.54,0.4],[3.14,-0.4,0]]
        self._home_joints = [[-72.94,-80.84,-130.1],[-51.87,111.72,-161.44]]
        self._pick_z = 0
        self._place_z = 0.08
        self._calibration_tool = ''
        self._R = np.zeros((3, 3))
        self._t = np.zeros((3, 1))

        self.pos = Queue()
        self.shut_down_thread = False
        self.to_read = False
        self._s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._s.settimeout(10)
        self._s.connect((self._robot_ip, self._port))
        self.read_thread = threading.Thread(target=self.run, daemon=True)
        self.start_thread()

    # target function of a new thread for continuously reading the robot state (500Hz)
    def run(self):
        while (not self.shut_down_thread):
            pos = self.ur_get_state()['tool_vector_actual']
            if self.to_read == True:
                self.pos.put(pos)
                self.to_read = False

    def read_pos(self):
        self.to_read = True
        cur_pos = self.pos.get()
        return cur_pos

    def start_thread(self):
        self.read_thread.start()

    def delete_thread(self):
        self.shut_down_thread = True
        time.sleep(1)
        self._s.close()       

    def goHome(self):
        # print('homing...')
        joint = [self._home_joints[0][0], self._home_joints[0][1], self._home_joints[0][2],
                 self._home_joints[1][0], self._home_joints[1][1], self._home_joints[1][2]]
        pose = [self._home_pose[0][0], self._home_pose[0][1], self._home_pose[0][2],
                self._home_pose[1][0], self._home_pose[1][1], self._home_pose[1][2]]
        self.move([joint, pose], useJoint=True)

    def execute(self, group, plan):
        pass

    def multiple_points_move(self, command):
        self._s.send(str.encode(command))

    def move_d(self,dx=0,dy=0,dz=0,Rx=3.14,Ry=0,Rz=0,a=0.02,v=0.02,verifyPos = False):
        '''
        Move robot along x,y,z-axis by distance dx,dy,dz (unit: mm)
        '''
        current_pos = self.read_pos()
        x_pos = current_pos[0]
        y_pos = current_pos[1]
        z_pos = current_pos[2]

        x_pos = x_pos + dx/1000
        y_pos = y_pos + dy/1000
        z_pos = z_pos + dz/1000

        goal = [[x_pos,y_pos,z_pos],[Rx,Ry,Rz]]
        self.move(goal,a,v,verifyPos=verifyPos)

    def move_to(self,x,y,z,Rx=3.14,Ry=0,Rz=0,a=0.02,v=0.02,verifyPos = False):
        '''
        Move robot to position (x,y,z) (unit: m)
        '''
        goal = [[x,y,z],[Rx,Ry,Rz]]
        self.move(goal,a,v,verifyPos=verifyPos) 

    def move(self, goal_pose, a=0.1, v=0.1,useJoint = False, verifyPos = False):
        '''
        Move robot to goal_pose [[x,y,z],[Rx,Ry,Rz]] (unit: m, rad)
        '''

        goal_position = goal_pose[0]
        goal_orientation = goal_pose[1]

        if(useJoint==False):
            # s.send ("movej(p[ %f, %f, %f, %f, %f, %f], a = %f, v = %f)\n" %(x/1000.0,y/1000.0,z/1000.0,Rx,Ry,Rz,a,v))
            x, y, z = goal_position[0], goal_position[1], goal_position[2]
            Rx, Ry, Rz = self.rpy2rotation(goal_orientation[0], goal_orientation[1], goal_orientation[2])
            MOVE_COMMAND = f"movel(p[{x},{y},{z},{Rx},{Ry},{Rz}], a = {a}, v = {v})\n" 
            MOVE_COMMAND = bytes(MOVE_COMMAND,encoding='utf-8')
            self._s.send (MOVE_COMMAND)
            if verifyPos==True:
                self.verifyPosition([x, y, z, Rx, Ry, Rz])
            else:
                pass
        else:
            #radian of each joint
            x, y, z = goal_position[0], goal_position[1], goal_position[2]
            Rx, Ry, Rz = goal_position[3], goal_position[4], goal_position[5]
            MOVE_COMMAND = f"movel([{x*3.14159/180.0},{y*3.14159/180.0},{z*3.14159/180.0},{Rx*3.14159/180.0},{Ry*3.14159/180.0},{Rz*3.14159/180.0}], a = {a}, v = v{v})\n"
            MOVE_COMMAND = bytes(MOVE_COMMAND,encoding='utf-8')
            self._s.send (MOVE_COMMAND)
            hx, hy, hz = goal_orientation[0], goal_orientation[1], goal_orientation[2]
            hRx, hRy, hRz = goal_orientation[3], goal_orientation[4], goal_orientation[5]
            #self.verifyJoints([hx, hy, hz, hRx, hRy, hRz])

    def openGripper(self):
        self._s.send(str.encode('set_digital_out(4,%s)\n' %True))

    def closeGripper(self):
        self._s.send(str.encode('set_digital_out(4,%s)\n' %False))

    def verifyJoints(self, targetPosition):
        delay_time = True
        cnt = 0
        timeGap = 1
        while(delay_time and cnt < 100):
            currentPose = self.read_pos()
            # print(targetPosition)
            # print(currentPose)
            dpose = np.zeros(3)
            inv_dpose = np.zeros(3)
            dpose[0] = abs(currentPose[0]-targetPosition[0])
            dpose[1] = abs(currentPose[1]-targetPosition[1])
            dpose[2] = abs(currentPose[2]-targetPosition[2])
            # dpose[3] = abs(currentPose[3]-targetPosition[3])
            # dpose[4] = abs(currentPose[4]-targetPosition[4])
            # dpose[5] = abs(currentPose[5]-targetPosition[5])

            inv_dpose[0] = abs(currentPose[0]-targetPosition[0])
            inv_dpose[1] = abs(currentPose[1]-targetPosition[1])
            inv_dpose[2] = abs(currentPose[2]-targetPosition[2])
            # inv_dpose[3] = abs(-currentPose[3]-targetPosition[3])
            # inv_dpose[4] = abs(-currentPose[4]-targetPosition[4])
            # inv_dpose[5] = abs(-currentPose[5]-targetPosition[5])

            if (max(dpose) < 0.02 or max(inv_dpose) < 0.02):
                delay_time = False
                return True
            else:
                time.sleep(timeGap)
                cnt = cnt + 1
            if(cnt*timeGap >= 20):
                print("Time Out!")
                return False

    def verifyPosition(self, targetPosition):
        delay_time = True
        cnt = 0
        timeGap = 1
        while(delay_time and cnt < 100):
            currentPose = self.read_pos()
            # print(targetPosition)
            # print(currentPose)
            dpose = np.zeros(6)
            inv_dpose = np.zeros(6)
            dpose[0] = abs(currentPose[0]-targetPosition[0])
            dpose[1] = abs(currentPose[1]-targetPosition[1])
            dpose[2] = abs(currentPose[2]-targetPosition[2])
            dpose[3] = abs(currentPose[3]-targetPosition[3])
            dpose[4] = abs(currentPose[4]-targetPosition[4])
            dpose[5] = abs(currentPose[5]-targetPosition[5])

            inv_dpose[0] = abs(currentPose[0]-targetPosition[0])
            inv_dpose[1] = abs(currentPose[1]-targetPosition[1])
            inv_dpose[2] = abs(currentPose[2]-targetPosition[2])
            inv_dpose[3] = abs(-currentPose[3]-targetPosition[3])
            inv_dpose[4] = abs(-currentPose[4]-targetPosition[4])
            inv_dpose[5] = abs(-currentPose[5]-targetPosition[5])

            if (max(dpose) < 0.0001 or max(inv_dpose) < 0.0001):
                delay_time = False
                return True
            else:
                time.sleep(timeGap)
                cnt = cnt + 1
            if(cnt*timeGap >= 40):
                print("Time Out!")
                return False

    def get_rigid_transform(self, A, B):
        assert len(A) == len(B)
        N = A.shape[0]  # Total points
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)
        AA = A - np.tile(centroid_A, (N, 1))  # Centre the points
        BB = B - np.tile(centroid_B, (N, 1))
        H = np.dot(np.transpose(AA), BB)  # Dot is matrix multiplication for array
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)
        if np.linalg.det(R) < 0:  # Special reflection case
            Vt[2, :] *= -1
            R = np.dot(Vt.T, U.T)
        t = np.dot(-R, centroid_A.T) + centroid_B.T
        return R, t

    def calibrating(self, camera):
        initial_pose = self.cfg['initial_pose']
        x_step = self.cfg['x_step_length']
        y_step = self.cfg['y_step_length']
        z_step = self.cfg['z_step_length']

        self.move(initial_pose)
        x = initial_pose[0][0]
        y = initial_pose[0][1]
        z = initial_pose[0][2]

        observed_pts = []
        measured_pts = []
        for i in range(4):
            for j in range(4):
                for k in range(4):
                    self.move([[x + x_step * i, y + y_step * j, z + z_step * k], initial_pose[1]])
                    color_image, info = camera.getImage()
                    depth_image = info[0]
                    observed_pt = image_callback(color_image, depth_image, camera.get_depth_scale())
                    measured_pt = [x + x_step * i, y + y_step * j, z + z_step * k + 0.17]
                    if len(observed_pt)!=0:
                        observed_pts.append(observed_pt)
                        measured_pts.append(measured_pt)
        np.savez(os.path.dirname(_root_path)+"/Data/calibration_data.npz", observed_pts, measured_pts)

    def matrix_load(self):
        d = np.load(os.path.dirname(_root_path)+"/Data/calibration_data.npz")
        observed_pts = d['arr_0']
        measured_pts = d['arr_1']
        self._R, self._t = self.get_rigid_transform(observed_pts, measured_pts)

    def uvd2xyz(self, u, v, depth_image, depth_scale):
        camera_z = np.mean(np.mean(depth_image[v - 5:v + 5, u - 5:u + 5])) * depth_scale
        camera_x = np.multiply(u - 642.142, camera_z / 922.378)
        camera_y = np.multiply(v - 355.044, camera_z / 922.881)
        # camera_x = np.multiply(u - 963.212, camera_z / 1383.57)
        # camera_y = np.multiply(v - 532.567, camera_z / 1384.32)

        view = depth_image[v - 30:v + 30, u - 30:u + 30]
        view[view == 0] = 10000
        avoid_z = np.min(view)
        # print(camera_z, avoid_z * depth_scale)
        avoid_v = np.where(depth_image[v - 30:v + 30, u - 30:u + 30] == avoid_z)[0][0] + v - 5
        avoid_u = np.where(depth_image[v - 30:v + 30, u - 30:u + 30] == avoid_z)[1][0] + u - 5
        avoid_x = np.multiply(avoid_u - 642.142, avoid_z * depth_scale / 922.378) # 1280, 720
        avoid_y = np.multiply(avoid_v - 355.044, avoid_z * depth_scale / 922.881) # 1280, 720
        # avoid_x = np.multiply(avoid_u - 963.212, avoid_z * depth_scale / 1383.57) # 1920, 1080
        # avoid_y = np.multiply(avoid_v - 532.567, avoid_z * depth_scale / 1384.32) # 1920, 1080

        xyz = self._R.dot(np.array([camera_x, camera_y, camera_z]).T) + self._t.T
        avoid_xyz = self._R.dot(np.array([avoid_x, avoid_y, avoid_z * depth_scale]).T) + self._t.T
        return list(xyz.T), avoid_xyz[2]



    def rpy2rotation(self, roll, pitch, yaw):
        yawMatrix = np.matrix([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]
        ])

        pitchMatrix = np.matrix([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ])

        rollMatrix = np.matrix([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ])

        R = yawMatrix * pitchMatrix * rollMatrix
        theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1) / 2)
        multi = 1 / (2 * math.sin(theta))
        rx = multi * (R[2, 1] - R[1, 2]) * theta
        ry = multi * (R[0, 2] - R[2, 0]) * theta
        rz = multi * (R[1, 0] - R[0, 1]) * theta
        rotation = np.zeros(3)
        rotation[0] = rx
        rotation[1] = ry
        rotation[2] = rz
        return rotation

    def open_socket(self):
        self._s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._s.settimeout(10)
        self._s.connect((self._robot_ip, self._port))

    def close_socket(self):
        self.shut_down_thread = True
        time.sleep(1)
        self._s.close()     

    def msg_unpack(self, ur_msg, start_mark: int, size_length: int, number_of_data: int):
        unpacked_msg = []
        for i in range(number_of_data):
            start = start_mark+i*size_length
            end = start_mark+(i+1)*size_length
            unpacked_msg.append(struct.unpack('!d', ur_msg[start:end])[0])
        return unpacked_msg

    def ur_get_state(self):
        ur_msg = self._s.recv(1108)        # recv_len = {'UR5': 1116, 'UR10e': 1108}
        # check the received data
        cnt = 0
        while len(ur_msg) != 1108 and cnt <100:
            ur_msg = self._s.recv(1108)
            cnt = cnt + 1

        msg = {'message_size': struct.unpack('!i', ur_msg[0:4])[0],
               'time': struct.unpack('!d', ur_msg[4:12])[0],
               'q_target': self.msg_unpack(ur_msg, 12, 8, 6),
               'qd_target': self.msg_unpack(ur_msg, 60, 8, 6),
               'qdd_target': self.msg_unpack(ur_msg, 108, 8, 6),
               'i_target': self.msg_unpack(ur_msg, 156, 8, 6),
               'm_target': self.msg_unpack(ur_msg, 204, 8, 6),
               'q_actual': self.msg_unpack(ur_msg, 252, 8, 6),
               'qd_actual': self.msg_unpack(ur_msg, 300, 8, 6),
               'i_actual': self.msg_unpack(ur_msg, 348, 8, 6),
               'i_control': self.msg_unpack(ur_msg, 396, 8, 6),
               'tool_vector_actual': self.msg_unpack(ur_msg, 444, 8, 6),
               'tcp_speed_actual': self.msg_unpack(ur_msg, 492, 8, 6),
               'tcp_force': self.msg_unpack(ur_msg, 540, 8, 6),
               'tool_vector_target': self.msg_unpack(ur_msg, 588, 8, 6),
               'tcp_speed_target': self.msg_unpack(ur_msg, 636, 8, 6),
               'digital_input_bits': self.msg_unpack(ur_msg, 684, 8, 1),
               'motor_temperatures': self.msg_unpack(ur_msg, 692, 8, 6),
               'controller_timer': self.msg_unpack(ur_msg, 740, 8, 1),
               'test_value': self.msg_unpack(ur_msg, 748, 8, 1),
               'robot_mode': self.msg_unpack(ur_msg, 756, 8, 1),
               'joint_mode': self.msg_unpack(ur_msg, 764, 8, 6),
               'safety_mode': self.msg_unpack(ur_msg, 812, 8, 1),
               'none_value_0': self.msg_unpack(ur_msg, 820, 8, 6),
               'tool_acelerometer_values': self.msg_unpack(ur_msg, 868, 8, 3),
               'none_value_1': self.msg_unpack(ur_msg, 892, 8, 6),
               'speed_scaling': self.msg_unpack(ur_msg, 940, 8, 1),
               'linear_momentum_norm': self.msg_unpack(ur_msg, 948, 8, 1),
               'none_value_2': self.msg_unpack(ur_msg, 956, 8, 1),
               'none_value_3': self.msg_unpack(ur_msg, 964, 8, 1),
               'v_main': self.msg_unpack(ur_msg, 972, 8, 1),
               'v_robot': self.msg_unpack(ur_msg, 980, 8, 1),
               'i_robot': self.msg_unpack(ur_msg, 988, 8, 1),
               'v_actual': self.msg_unpack(ur_msg, 996, 8, 6),
               'digital_outputs': self.msg_unpack(ur_msg, 1044, 8, 1),
               'program_state': self.msg_unpack(ur_msg, 1052, 8, 1),
               'elbow_position': self.msg_unpack(ur_msg, 1060, 8, 3),
               'elbow_velocity': self.msg_unpack(ur_msg, 1084, 8, 3)}
        return msg