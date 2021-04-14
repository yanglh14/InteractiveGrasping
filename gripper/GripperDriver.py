#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Yang Linhan

#
# *********     Read and Write Example      *********
#
#
# Available Dynamixel model on this example : All models using Protocol 2.0
# This example is designed for using a Dynamixel AX-12A, and an USB2DYNAMIXEL.
# This module is used for a gripper consisting of multiple motors which are set as ID: 0,1,2,3.... and  Baudrate : 1000000

import os
import numpy as np
import time
# if os.name == 'nt':
#     import msvcrt
#     def getch():
#         return msvcrt.getch().decode()
# else:
#     import sys, tty, termios
#     fd = sys.stdin.fileno()
#     old_settings = termios.tcgetattr(fd)
#     def getch():
#         try:
#             tty.setraw(sys.stdin.fileno())
#             ch = sys.stdin.read(1)
#         finally:
#             termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#         return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library


class GripperController():
    def __init__(self,MotorNum = 6,BAUDRATE = 57600, PATH = "/dev/ttyUSB0"):

        # Control table address
        self._ADDR_PRO_TORQUE_ENABLE       = 64	           # Control table address is different in Dynamixel model
        self._ADDR_PRO_GOAL_POSITION       = 116
        self._ADDR_PRO_PRESENT_POSITION    = 132
        self._ADDR_PRO_MOVING_SPEED        = 112
        self._ADDR_PRO_OPERATION_MODE      = 11
        self._ADDR_PRO_GOAL_CURRENT        = 102

        # Protocol version
        self._PROTOCOL_VERSION             = 2.0               # See which protocol version is used in the Dynamixel

        # Default setting
        self._MotorNum                    = MotorNum
        self._DXL_ID                      = [1,2,3,4,5,6]                 # Dynamixel ID : 1
        self._BAUDRATE                    = BAUDRATE            # Dynamixel default baudrate : 57600
        self._DEVICENAME                  = PATH    # Check which port is being used on your controller
        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
        self._TORQUE_ENABLE               = 1                 # Value for enabling the torque
        self._TORQUE_DISABLE              = 0                 # Value for disabling the torque
        self._DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
        self._DXL_MOVING_SPEED            = 10
        self._HOME_POSITION               = [3096,2160,2560,2560,1024,3072]
        # self._HOME_POSITION               = [3697,1472,2560,2560,1024,3072]

        self._GOAL_POSITION               = []


        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(self._DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(self._PROTOCOL_VERSION)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        if self.portHandler.setBaudRate(self._BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

        # Enable Dynamixel Torque
        for i in range(self._MotorNum):
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, i+1 , self._ADDR_PRO_TORQUE_ENABLE, self._TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("ID%d %s" % (i+1 , self.packetHandler.getTxRxResult(dxl_comm_result)) )
            elif dxl_error != 0:
                print("ID%d %s"%(i+1 , self.packetHandler.getRxPacketError(dxl_error))  )
            else:
                print("ID%d Dynamixel has been successfully connected" %(i+1))

    # Moing to a goal position Input: list of angles
    def moving_position(self,GoalPosition,Number = 6, Wait = True):
        while 1:
            for i in range(Number):
                #Set Moving Speed
                dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, i+1, self._ADDR_PRO_MOVING_SPEED, self._DXL_MOVING_SPEED)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))

                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, i+1 , self._ADDR_PRO_GOAL_CURRENT, 100)


                # Write goal position
                dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, i+1, self._ADDR_PRO_GOAL_POSITION, GoalPosition[i])
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))


            #Stop when arrived
            if Wait == True:
                while 1:
                    diff = []
                    for i in range(Number):
                        # Read present position
                        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, i+1, self._ADDR_PRO_PRESENT_POSITION)
                        if dxl_comm_result != COMM_SUCCESS:
                            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                        elif dxl_error != 0:
                            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
                        diff.append(abs(GoalPosition[i] - dxl_present_position))

                    if max(diff) < self._DXL_MOVING_STATUS_THRESHOLD:
                        break
            break

    # Moving to the home position
    def reset(self):
        # self.change_mode(3)
        self.moving_position(self._HOME_POSITION)


    def read_position(self):
        position = []
        for i in range(self._MotorNum):
            # Read present position
            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, i+1, self._ADDR_PRO_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            position.append(dxl_present_position)
        return position

    def release(self):
        self._GOAL_POSITION = self.read_position()
        self._GOAL_POSITION[3:6] = self._HOME_POSITION[3:6]

        self.moving_position(self._GOAL_POSITION)

    def change_mode(self,mode):
        for i in range(3,6):
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, i+1 , self._ADDR_PRO_TORQUE_ENABLE, self._TORQUE_DISABLE)
        for i in range(3,6):
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, i+1 , self._ADDR_PRO_OPERATION_MODE, mode)
            if dxl_comm_result != COMM_SUCCESS:
                print("ID%d %s" % (i+1 , self.packetHandler.getTxRxResult(dxl_comm_result)) )
            elif dxl_error != 0:
                print("ID%d %s"%(i+1 , self.packetHandler.getRxPacketError(dxl_error))  )
            else:
                print("ID%d Dynamixel mode has been successfully changed" %(i+1))
        for i in range(3,6):
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, i+1 , self._ADDR_PRO_TORQUE_ENABLE, self._TORQUE_ENABLE)

    def moving_force(self,goal_force = [10,10,10]):

        for i in range(3,6):
            #Set Moving Speed
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, i+1, self._ADDR_PRO_MOVING_SPEED, self._DXL_MOVING_SPEED)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, i+1 , self._ADDR_PRO_GOAL_CURRENT, goal_force[i-3])
            if dxl_comm_result != COMM_SUCCESS:
                print("ID%d %s" % (i+1 , self.packetHandler.getTxRxResult(dxl_comm_result)) )
            elif dxl_error != 0:
                print("ID%d %s"%(i+1 , self.packetHandler.getRxPacketError(dxl_error))  )
            else:
                print("ID%d Dynamixel has been successfully connected" %(i+1))


    def close(self):
        for i in range(self._MotorNum):
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, i+1 , self._ADDR_PRO_TORQUE_ENABLE, self._TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("ID%d %s" % (i+1 , self.packetHandler.getTxRxResult(dxl_comm_result)) )
            elif dxl_error != 0:
                print("ID%d %s"%(i+1 , self.packetHandler.getRxPacketError(dxl_error))  )
            else:
                print("ID%d Dynamixel has been successfully closed" %(i+1))
        self.portHandler.closePort()

if __name__ == "__main__":
    gripper = GripperController()

    gripper.moving_force([5,0,0])
    gripper.close()
