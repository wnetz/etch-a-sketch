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

# Author: Ryu Woon Jung (Leon)

#
# *********     Read and Write Example      *********
#
#
# Available DXL model on this example : All models using Protocol 1.0
# This example is tested with a DXL MX-28, and an USB2DYNAMIXEL
# Be sure that DXL MX properties are already set as %% ID : 1 / Baudnum : 34 (Baudrate : 57600)
#

import os
import time
import math
from dynamixel_sdk import *

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



# Control table address
ADDR_MX_CCW_ANGLE_LIMIT     = 8
ADDR_MX_TORQUE_ENABLE      = 24
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_MOVING_SPEED       = 32
ADDR_MX_TORQUE_LIMIT       = 34
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MX_PRESENT_SPEED   = 38

# Protocol version
PROTOCOL_VERSION            = 1.0

# Default setting
DXL1_ID                      = 1
DXL2_ID                      = 2
BAUDRATE                    = 1000000
DEVICENAME                  = 'COM5'

TORQUE_ENABLE               = 1
TORQUE_DISABLE              = 0
DXL_MOVING_STATUS_THRESHOLD = 5
DXL_MAXIMUM_TORQUE   = 250
FULL_SPEED = 1023

class Position:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.domain = 4.15*360/(100/341)
        self.range = 2.6*360/(100/341)
        self.rotationBuffer = 10
        self.zeroPosition = [0,0]
        self.rotations = [0,0]
        self.alpha = .05
        self.rotAlpha = .1
        self.off = [[],[]]
        self.err = [[],[]]
        self.rottimes = [[],[]]

        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(DEVICENAME)
        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL1_ID, ADDR_MX_CCW_ANGLE_LIMIT, 0)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel 1 has been successfully connected")
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL2_ID, ADDR_MX_CCW_ANGLE_LIMIT, 0)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel 2 has been successfully connected")

        # Enable Dynamixel Torque
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)

        #limit torque for safty
        self.packetHandler.write2ByteTxRx(self.portHandler, DXL1_ID, ADDR_MX_TORQUE_LIMIT, DXL_MAXIMUM_TORQUE)
        self.packetHandler.write2ByteTxRx(self.portHandler, DXL2_ID, ADDR_MX_TORQUE_LIMIT, DXL_MAXIMUM_TORQUE)
    def getSpeed(self,xDistance,yDistance):
        xSpeed = 0
        ySpeed = 0
        if abs(xDistance) > abs(yDistance):
            xSpeed = FULL_SPEED
            ySpeed = FULL_SPEED * abs(yDistance/xDistance)
            if abs(yDistance) < 1227.6:
                ySpeed = abs(yDistance)/1227.6*ySpeed
                ySpeed = max(min(1023,ySpeed),150)
            xSpeed = ySpeed / abs(yDistance/xDistance)
            if yDistance < 0:
                ySpeed = ySpeed+1024
            if xDistance < 0:
                xSpeed = xSpeed+1024
            if yDistance > -self.rotationBuffer and yDistance < self.rotationBuffer:
                ySpeed = 0
            if xDistance > -self.rotationBuffer and xDistance < self.rotationBuffer:
                xSpeed = 0
        else:
            xSpeed = FULL_SPEED * abs(xDistance/yDistance)
            ySpeed = FULL_SPEED
            if abs(xDistance) < 1227.6:
                xSpeed = abs(xDistance)/1227.6*xSpeed
                xSpeed = max(min(1023,xSpeed),150)
            ySpeed = xSpeed / abs(xDistance/yDistance)
            if xDistance < 0:
                xSpeed = xSpeed+1024
            if yDistance < 0:
                ySpeed = ySpeed+1024
            if yDistance > -self.rotationBuffer and yDistance < self.rotationBuffer:
                ySpeed = 0
            if xDistance > -self.rotationBuffer and xDistance < self.rotationBuffer:
                xSpeed = 0

        return [xSpeed,ySpeed]
    def unBlind(self,position,oldPosition,avgRotationDifferance, timeDifferance, side, buffer):
        if position > side and position < side + buffer:
            differance = abs(oldPosition+avgRotationDifferance*timeDifferance-position)
            if differance > 1023:
                differance = differance - 1227.6
            self.off.append(abs(oldPosition+avgRotationDifferance*timeDifferance-position))
            self.err.append(abs(oldPosition+avgRotationDifferance*timeDifferance-position)/1227.6)
            return False
        return True
    def goTo(self,toX,toY):
        if toX < 0 or toX > self.domain:
            print("out side of domain")
        elif toY < 0 or toY > self.range:
            print("out side of range")
        else:
            speed = self.getSpeed(toX-self.x,toY-self.y)
            print(speed)
            xZeroPosition, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION)
            while (xZeroPosition > 923 and xZeroPosition <= 1023) or (xZeroPosition >= 0 and xZeroPosition < 100) or (xZeroPosition > 615 and xZeroPosition < 630):
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL1_ID, ADDR_MX_MOVING_SPEED, 2047)
                xZeroPosition, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION)
            self.packetHandler.write2ByteTxRx(self.portHandler, DXL1_ID, ADDR_MX_MOVING_SPEED, 0)
            xZeroPosition, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION)

            yZeroPosition, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION)
            while (yZeroPosition > 923 and yZeroPosition <= 1023) or (yZeroPosition >= 0 and yZeroPosition < 100) or (yZeroPosition > 615 and yZeroPosition < 630):
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL2_ID, ADDR_MX_MOVING_SPEED, 2047)
                yZeroPosition, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION)
            self.packetHandler.write2ByteTxRx(self.portHandler, DXL2_ID, ADDR_MX_MOVING_SPEED, 0)
            yZeroPosition, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION)

            self.zeroPosition = [xZeroPosition,yZeroPosition]
            blind = [False,False]
            start = [True,True]
            oldpos = [0,0]
            avgRotationDifferance = [0,0]
            rotationTime = [0,0]
            position = [self.zeroPosition[0],self.zeroPosition[1]]
            oldPosition = [self.zeroPosition[0],self.zeroPosition[1]]
            projectedPosition = [self.zeroPosition[0],self.zeroPosition[1]]
            off = [[],[]]
            err = [[],[]]
            rotationtimes = [[],[]]
            oldTime = 0
            side = [0,0]
            buffer = [100,100]

            if speed[0] > 1023:
                buffer[0] = -100
                side[0] = 1023 + buffer[0]
            if speed[1] > 1023:
                buffer[1] = -100
                side[1] = 1023 + buffer[1]

            print(self.zeroPosition,position)

            tine = time.time()
            while toX < self.x - self.rotationBuffer or toX > self.x + self.rotationBuffer or toY < self.y - self.rotationBuffer or toY > self.y + self.rotationBuffer:

                position[0], dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION)
                position[1], dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION)

                speed = self.getSpeed(toX-self.x,toY-self.y)
                print(speed)
                self.packetHandler.write2ByteTxRx(self.portHandler, DXL1_ID, ADDR_MX_MOVING_SPEED, int(round(speed[0],0)))
                self.packetHandler.write2ByteTxRx(self.portHandler, DXL2_ID, ADDR_MX_MOVING_SPEED, int(round(speed[1],0)))

                timeDifferance = time.time()-tine
                projectedPosition = [oldPosition[0] + timeDifferance*avgRotationDifferance[0], oldPosition[1] + timeDifferance*avgRotationDifferance[1]]
                print("time diff: " + str(timeDifferance))
                print(oldPosition)
                if blind[0]:
                    print("X blind")
                    blind[0] = self.unBlind(position[0],oldPosition[0],avgRotationDifferance[0], timeDifferance, side[0], buffer[0])
                    if blind[0]:
                        position[0] = projectedPosition[0]
                else:
                    if avgRotationDifferance[0] == 0:
                        avgRotationDifferance[0] = (position[0]-oldPosition[0])/timeDifferance
                    avgRotationDifferance[0] = avgRotationDifferance[0]*self.rotAlpha + ((position[0]-oldPosition[0])/timeDifferance)*(1-self.rotAlpha)
                if blind[1]:
                    print("Y blind")
                    blind[1] = self.unBlind(position[1],oldPosition[1],avgRotationDifferance[1], timeDifferance, side[1], buffer[1])
                    if blind[1]:
                        position[1] = projectedPosition[1]
                else:
                    if avgRotationDifferance[1] == 0:
                        avgRotationDifferance[1] = (position[1]-oldPosition[1])/timeDifferance
                    avgRotationDifferance[1] = avgRotationDifferance[1]*self.rotAlpha + ((position[1]-oldPosition[1])/timeDifferance)*(1-self.rotAlpha)

                if not blind[0]:
                    if position[0] + avgRotationDifferance[0]*timeDifferance >= 1023:
                        position[0] = projectedPosition[0]
                        blind[0] = True
                    elif position[0] + avgRotationDifferance[0]*timeDifferance <= 0:
                        projectedPosition[0] = projectedPosition[0]-1227.6
                        position[0] = projectedPosition[0]
                        blind[0] = True
                if not blind[1]:
                    if position[1] + avgRotationDifferance[1]*timeDifferance >= 1023:
                        position[1] = projectedPosition[1]
                        blind[1] = True
                    elif position[1] + avgRotationDifferance[1]*timeDifferance <= 0:
                        projectedPosition[1] = projectedPosition[1]-1227.6
                        position[1] = projectedPosition[1]
                        blind[1] = True

                if start[0] and position[0] > self.zeroPosition[0] + self.rotationBuffer:
                    start[0] = False
                if start[1] and position[1] > self.zeroPosition[1] + self.rotationBuffer:
                    start[1] = False

                print(self.zeroPosition[0] - self.rotationBuffer,position[0],self.zeroPosition[0] + self.rotationBuffer,start[0],tine-rotationTime[0])
                if position[0] > self.zeroPosition[0] - self.rotationBuffer and position[0] < self.zeroPosition[0] + self.rotationBuffer and not start[0] and tine-rotationTime[0] > .5:
                    rotationtimes[0].append(tine-rotationTime[0])
                    rotationTime[0] = tine
                    self.rotations[0] = self.rotations[0] + 1
                    print("X rotations " + str(self.rotations[0]))
                if speed[0] != 0:
                    self.x = self.rotations[0]*360/(100/341) + position[0] - self.zeroPosition[0]
                if position[1] > self.zeroPosition[1] - self.rotationBuffer and position[1] < self.zeroPosition[1] + self.rotationBuffer and not start[1] and tine-rotationTime[1] > .5:
                    rotationtimes[1].append(tine-rotationTime[1])
                    rotationTime[1] = tine
                    self.rotations[1] = self.rotations[1] + 1
                    print("Y rotations " + str(self.rotations[1]))
                if speed[1] != 0:
                    self.y = self.rotations[1]*360/(100/341) + position[1] - self.zeroPosition[1]

                if self.x < 0:
                    self.x = self.x + 1227
                if self.y < 0:
                    self.y = self.y + 1227
                z = [self.x,self.y]
                print("Pos", z,"RotPos",position,"ProjPos",projectedPosition,"Rots",self.rotations,"speed",speed,"rotDiff",[position[0]-oldPosition[0],position[1]-oldPosition[1]],"avgRotDiff",avgRotationDifferance)
                oldPosition = [position[0],position[1]]
                tine = time.time()
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL1_ID, ADDR_MX_MOVING_SPEED, 0)
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL2_ID, ADDR_MX_MOVING_SPEED, 0)
    def printError(self):
        print([self.x,self.y])
        print(self.off)
        print(self.err)
        print(self.rottimes)
    def end(self):
        # Disable Dynamixel Torque
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)

        # Close port
        self.portHandler.closePort()


draw = Position()
print([draw.domain/2,draw.range/2])
draw.goTo(draw.domain/2,draw.range/2)
#draw.goTo(draw.domain,draw.range)
