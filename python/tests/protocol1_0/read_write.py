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
ADDR_MX_TORQUE_ENABLE       = 24
ADDR_MX_D                   = 26
ADDR_MX_I                   = 27
ADDR_MX_P                   = 28
ADDR_MX_GOAL_POSITION       = 30
ADDR_MX_MOVING_SPEED        = 32
ADDR_MX_TORQUE_LIMIT        = 34
ADDR_MX_PRESENT_POSITION    = 36
ADDR_MX_PRESENT_SPEED       = 38
ADDR_MX_MOVING              = 46
ADDR_MX_GOAL_ACCELERATION   = 73

# Protocol version
PROTOCOL_VERSION            = 1.0

# Default setting
HORIZONTAL                  = 0
VERTICAL                    = 1
BAUDRATE                    = 1000000
DEVICENAME                  = 'COM5'

TORQUE_ENABLE               = 1
TORQUE_DISABLE              = 0
DXL_MOVING_STATUS_THRESHOLD = 5
DXL_MAXIMUM_TORQUE          = 800
DXL_D                       = 254
DXL_I                       = 0
DXL_P                       = 20
DXL_MAXIMUM_SPEED           = 500 
DXL_MINIMUM_SPEED           = 15  

class Position:
    def __init__(self):        

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
        
        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, HORIZONTAL, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel 1 has been successfully connected")
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, VERTICAL, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel 2 has been successfully connected")

        #limit torque for safty
        self.packetHandler.write2ByteTxRx(self.portHandler, HORIZONTAL, ADDR_MX_TORQUE_LIMIT, DXL_MAXIMUM_TORQUE)
        self.packetHandler.write2ByteTxRx(self.portHandler, VERTICAL, ADDR_MX_TORQUE_LIMIT, DXL_MAXIMUM_TORQUE)

        #D
        self.packetHandler.write2ByteTxRx(self.portHandler,HORIZONTAL,ADDR_MX_D,DXL_D)
        self.packetHandler.write2ByteTxRx(self.portHandler,VERTICAL,ADDR_MX_D,DXL_D)
        #I
        self.packetHandler.write2ByteTxRx(self.portHandler,HORIZONTAL,ADDR_MX_I,DXL_I)
        self.packetHandler.write2ByteTxRx(self.portHandler,VERTICAL,ADDR_MX_I,DXL_I)
        #P
        self.packetHandler.write2ByteTxRx(self.portHandler,HORIZONTAL,ADDR_MX_P,DXL_P)
        self.packetHandler.write2ByteTxRx(self.portHandler,VERTICAL,ADDR_MX_P,DXL_P)

        self.xStartPosition, dont, care = self.packetHandler.read2ByteTxRx(self.portHandler,HORIZONTAL,ADDR_MX_PRESENT_POSITION)
        self.yStartPosition, dont, care = self.packetHandler.read2ByteTxRx(self.portHandler,VERTICAL,ADDR_MX_PRESENT_POSITION)

        self.x = 0
        self.y = 0

        self.xRotations = 0
        self.yRotations = 0

        self.xRecord = self.xStartPosition
        self.yRecord = self.yStartPosition

        self.xLastPosition = self.xStartPosition
        self.yLastPosition = self.yStartPosition

    def goto(self,toX,toY):         
        xPosition = self.x
        xInitialDistance = abs(toX-xPosition)
        yPosition = self.y
        yInitialDistance = abs(toY-yPosition)

        while toX - 100 > xPosition or toX + 100 < xPosition or toY - 100 > yPosition or toY + 100 < yPosition:
            xPosition, xRelativePosition, xRotationPosition, yPosition, yRelativePosition,yRotationPosition = self.getPosition()
            self.setRotations(xRotationPosition,toX > xPosition,yRotationPosition, toY > yPosition)
            self.setVelocity(toX,xPosition,xInitialDistance,toY,yPosition,yInitialDistance) 
            print(toX - 100 > xPosition, toX + 100 < xPosition, toY - 100 > yPosition, toY + 100 < yPosition,toX - 100 > xPosition or toX + 100 < xPosition or toY - 100 > yPosition or toY + 100 < yPosition)
            print("----------------------------------------")
            self.xLastPosition = xRotationPosition
            self.yLastPosition = yRotationPosition
        self.packetHandler.write2ByteTxRx(self.portHandler,HORIZONTAL,ADDR_MX_MOVING_SPEED,0)
        self.packetHandler.write2ByteTxRx(self.portHandler,VERTICAL,ADDR_MX_MOVING_SPEED,0)
        self.x, xRelativePosition, self.xLastPosition, self.y, yRelativePosition, self.yLastPosition = self.getPosition()

    def getPosition(self):
        xRotationPosition, dont, care = self.packetHandler.read2ByteTxRx(self.portHandler,HORIZONTAL,ADDR_MX_PRESENT_POSITION)
        yRotationPosition, dont, care = self.packetHandler.read2ByteTxRx(self.portHandler,VERTICAL,ADDR_MX_PRESENT_POSITION)

        xRelativePosition = int(round((xRotationPosition - self.xStartPosition)/10)*10)
        yRelativePosition = int(round((yRotationPosition - self.yStartPosition)/10)*10)
        if xRelativePosition < 0:
            xRelativePosition = xRelativePosition + 4095
        if yRelativePosition < 0:
            yRelativePosition = yRelativePosition + 4095
        xPosition = xRelativePosition + self.xRotations*4095
        yPosition = yRelativePosition + self.yRotations*4095
        print(xPosition, xRelativePosition, xRotationPosition, yPosition, yRelativePosition,yRotationPosition)
        return xPosition, xRelativePosition, xRotationPosition, yPosition, yRelativePosition,yRotationPosition

    def setRotations(self,xRotationPosition,xForward,yRotationPosition,yForward):
        print(self.xLastPosition,xRotationPosition,self.xStartPosition, self.xRotations,xForward,self.yLastPosition,yRotationPosition,self.yStartPosition,self.yRotations,yForward)
        if xForward and (xRotationPosition-self.xLastPosition > 5):
            if (xRotationPosition > self.xStartPosition and self.xStartPosition > self.xLastPosition) or (self.xStartPosition > self.xLastPosition and self.xLastPosition > xRotationPosition):
                self.xRotations = self.xRotations + 1
        elif (not xForward) and (xRotationPosition-self.xLastPosition < -5) :
            if (xRotationPosition < self.xStartPosition and self.xStartPosition < self.xLastPosition) or (self.xStartPosition < self.xLastPosition and self.xLastPosition < xRotationPosition):
                self.xRotations = self.xRotations - 1
        
        if yForward and (yRotationPosition-self.yLastPosition > 5):
            if (yRotationPosition > self.yStartPosition and self.yStartPosition > self.yLastPosition) or (self.yStartPosition > self.yLastPosition and self.yLastPosition > yRotationPosition):
                self.yRotations = self.yRotations + 1
        elif (not yForward) and (yRotationPosition-self.yLastPosition < -5) :
            if (yRotationPosition < self.yStartPosition and self.yStartPosition < self.yLastPosition) or (self.yStartPosition < self.yLastPosition and self.yLastPosition < yRotationPosition):
                self.yRotations = self.yRotations - 1

        print(self.xLastPosition,xRotationPosition,self.xStartPosition, self.xRotations,xForward,self.yLastPosition,yRotationPosition,self.yStartPosition,self.yRotations,yForward)

    def setVelocity(self,toX,xPosition,xInitialDistance,toY,yPosition,yInitialDistance):
        ratiox = .5
        ratioy = .5

        speed = max(min(abs(toX-xPosition)/xInitialDistance,abs(toY-yPosition)/yInitialDistance)*DXL_MAXIMUM_SPEED,DXL_MINIMUM_SPEED)
        print(abs(toX-xPosition)/xInitialDistance,abs(toY-yPosition)/yInitialDistance,speed)
        
        ratiox = abs((xPosition - toX))/(abs(yPosition - toY) + abs(xPosition - toX))
        ratioy = 1 - ratiox
        
        xVelocity = ratiox*speed
        yVelocity = ratioy*speed
        if xPosition > toX:
            xVelocity = xVelocity + 1023
        if yPosition > toY:
            yVelocity = yVelocity + 1023
        self.packetHandler.write2ByteTxRx(self.portHandler,HORIZONTAL,ADDR_MX_MOVING_SPEED,math.floor(xVelocity))
        self.packetHandler.write2ByteTxRx(self.portHandler,VERTICAL,ADDR_MX_MOVING_SPEED,math.floor(yVelocity))
        print(ratiox,xVelocity,ratioy,yVelocity)

    def disconect(self):
        # Disable Dynamixel Torque
        self.packetHandler.write1ByteTxRx(self.portHandler, HORIZONTAL, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, VERTICAL, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)

        # Close port
        self.portHandler.closePort()

test = Position()
xmax = 18000
ymax = 12000
inp = input()
test.getPosition()
while "q" not in inp:    
    loc = inp.find(",")
    xval = int(inp[0:loc])
    yval = int(inp[loc+1:])
    resolution = 20
    test.goto(xval,yval)
    inp = input()