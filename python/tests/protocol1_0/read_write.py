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
DXL_MAXIMUM_SPEED           = 225 #225    

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

        self.xStartPosition, dont, care = self.packetHandler.read2ByteTxRx(self.portHandler,HORIZONTAL,ADDR_MX_PRESENT_POSITION)
        self.yStartPosition, dont, care = self.packetHandler.read2ByteTxRx(self.portHandler,VERTICAL,ADDR_MX_PRESENT_POSITION)
        print(self.xStartPosition,self.yStartPosition)

        #set speed
        self.packetHandler.write2ByteTxRx(self.portHandler,HORIZONTAL,ADDR_MX_MOVING_SPEED,DXL_MAXIMUM_SPEED)
        self.packetHandler.write2ByteTxRx(self.portHandler,VERTICAL,ADDR_MX_MOVING_SPEED,DXL_MAXIMUM_SPEED)
        #D
        self.packetHandler.write2ByteTxRx(self.portHandler,HORIZONTAL,ADDR_MX_D,DXL_D)
        self.packetHandler.write2ByteTxRx(self.portHandler,VERTICAL,ADDR_MX_D,DXL_D)
        #I
        self.packetHandler.write2ByteTxRx(self.portHandler,HORIZONTAL,ADDR_MX_I,DXL_I)
        self.packetHandler.write2ByteTxRx(self.portHandler,VERTICAL,ADDR_MX_I,DXL_I)
        #P
        self.packetHandler.write2ByteTxRx(self.portHandler,HORIZONTAL,ADDR_MX_P,DXL_P)
        self.packetHandler.write2ByteTxRx(self.portHandler,VERTICAL,ADDR_MX_P,DXL_P)

    def goto(self,toX,toY):
        
        self.packetHandler.write2ByteTxRx(self.portHandler,HORIZONTAL,ADDR_MX_GOAL_POSITION,math.floor((toX + self.xStartPosition)))
        self.packetHandler.write2ByteTxRx(self.portHandler,VERTICAL,ADDR_MX_GOAL_POSITION,math.floor((toY + self.yStartPosition)))
        #if not (self.x - toX == 0):
        #    slope = (self.y - toY)/(self.x - toX)
        #    print(slope)
        #    if slope <= 1:
        #        print(math.floor(DXL_MAXIMUM_SPEED*(1-slope)),math.floor(DXL_MAXIMUM_SPEED*slope))
        #        self.packetHandler.write2ByteTxRx(self.portHandler,HORIZONTAL,ADDR_MX_MOVING_SPEED,math.floor(DXL_MAXIMUM_SPEED*slope))
        #        self.packetHandler.write2ByteTxRx(self.portHandler,VERTICAL,ADDR_MX_MOVING_SPEED,math.floor(DXL_MAXIMUM_SPEED*(1-slope)))
        #    else:
        #        print(math.floor(DXL_MAXIMUM_SPEED*(1/slope)),DXL_MAXIMUM_SPEED*(1-(1/slope)))
        #        self.packetHandler.write2ByteTxRx(self.portHandler,HORIZONTAL,ADDR_MX_MOVING_SPEED,math.floor(DXL_MAXIMUM_SPEED*(1-(1/slope))))
        #        self.packetHandler.write2ByteTxRx(self.portHandler,VERTICAL,ADDR_MX_MOVING_SPEED,math.floor(DXL_MAXIMUM_SPEED*(1/slope)))
        xmoving = True
        ymoving = True

        while xmoving or ymoving:
            xmoving, dont, care = self.packetHandler.read2ByteTxRx(self.portHandler,HORIZONTAL,ADDR_MX_MOVING)
            ymoving, dont, care = self.packetHandler.read2ByteTxRx(self.portHandler,VERTICAL,ADDR_MX_MOVING)


        self.x, dont, care = self.packetHandler.read2ByteTxRx(self.portHandler,HORIZONTAL,ADDR_MX_PRESENT_POSITION)
        self.x = self.xStartPosition - self.x
        self.y, dont, care = self.packetHandler.read2ByteTxRx(self.portHandler,VERTICAL,ADDR_MX_PRESENT_POSITION)
        self.y = self.yStartPosition - self.y
    def getPosition(self):
        return [self.x,self.y]
    def disconect(self):
        # Disable Dynamixel Torque
        self.packetHandler.write1ByteTxRx(self.portHandler, HORIZONTAL, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, VERTICAL, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)

        # Close port
        self.portHandler.closePort()

test = Position()
xmax = 18000
ymax = 12000
test.goto(0,0)
inp = "0,0"
while "q" not in inp:    
    loc = inp.find(",")
    xval = int(inp[0:loc])
    yval = int(inp[loc+1:])
    resolution = 20
    test.goto(xval,yval)
    print(test.getPosition())
    inp = input()