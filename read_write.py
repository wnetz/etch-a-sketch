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

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_MX_CCW_ANGLE_LIMIT     = 8
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_MOVING_SPEED       = 32
ADDR_MX_TORQUE_LIMIT       = 34
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MX_PRESENT_SPEED   = 38

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 2                 # Dynamixel ID : 1
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = 'COM5'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1023            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 5                # Dynamixel moving status threshold
DXL_MAXIMUM_TORQUE   = 250

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CCW_ANGLE_LIMIT, 0)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 2, ADDR_MX_CCW_ANGLE_LIMIT, 0)

# Enable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")

dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_LIMIT, DXL_MAXIMUM_TORQUE)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 2, ADDR_MX_TORQUE_LIMIT, DXL_MAXIMUM_TORQUE)
#x = 4.35
tme = time.time()
oldTme = 0
movingAvg = 0
pos = 0
avg = 0
range = 2.75*360/(100/341)
domain = 4.35*360/(100/341)
length = range
rots = 0
blind = False
start = True
speed = 0
alpha = .05
rotAlpha = .1
oldpos = 0
diff = 0
avgRotDiff = 0
rotTime = 0
oldPosi = 0
off = []
err = []
rottimes = []

zeroPos, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
print(zeroPos)
while (zeroPos > 923 and zeroPos <= 1023) or (zeroPos >= 0 and zeroPos < 100) or (zeroPos > 615 and zeroPos < 630):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_MOVING_SPEED, 2047)
    zeroPos, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_MOVING_SPEED, 0)
print(zeroPos)
proj = zeroPos
oldPosi = zeroPos

while pos < length:
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_MOVING_SPEED, 1023)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    posi, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    speed, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_SPEED)
    diff = time.time()-tme
    proj = oldPosi + diff*avgRotDiff
    if blind:
        if posi > 0 and posi < 100:
            blind = False
            print(str(time.time()-tme)+" "+str(avg))
            off.append(abs(oldPosi+avgRotDiff*diff-1227.6-posi))
            err.append(abs(oldPosi+avgRotDiff*diff-1227.6-posi)/1227.6)
        else:
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))


            print("time " + str(diff))
            #posi = oldPosi + (diff)*avg
            posi = proj
    else:

        print("time diff: " + str(diff))
        if avg == 0:
            avg = speed
        if speed > 0:
            avg = avg*alpha + speed*(1-alpha)
        if avgRotDiff == 0:
            avgRotDiff = (posi-oldPosi)/diff
        avgRotDiff = avgRotDiff*rotAlpha + ((posi-oldPosi)/diff)*(1-rotAlpha)

    if posi + avgRotDiff*diff >= 1023 and not blind:
        blind = True
        proj = posi
    if start and posi > zeroPos +7:
        start = False
    if posi > zeroPos -7 and posi < zeroPos +7 and not start and (rotTime == 0 or tme-rotTime > .5):
        rottimes.append(tme-rotTime)
        rotTime = tme
        rots = rots + 1
        print("rots " + str(rots))
    pos = rots*360/(100/341) + posi - zeroPos
    if pos < 0:
        pos = pos + 1024 + 60
    print("pos " +  str(pos) + " posi "+str(posi)  + " Proj " + str(proj) + " rots " + str(rots) + " speed " + str(speed) + " avg "+str(avg)  + " rotDiff " + str(posi-oldPosi) + " avgRotDiff "+str(avgRotDiff))
    oldPosi = posi
    tme = time.time()
print("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")
end = pos
start = False
rots = 0
while pos > 0:
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_MOVING_SPEED, 2047)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    posi, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    speed, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_SPEED)
    diff = time.time()-tme
    proj = oldPosi - diff*avgRotDiff
    if blind:
        if posi < 1023 and posi > 923:
            blind = False
            print(str(time.time()-tme)+" "+str(avg))
            off.append(abs(oldPosi+avgRotDiff*diff-posi))
            err.append(abs(oldPosi+avgRotDiff*diff-posi)/1227.6)
        else:
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))


            print("time " + str(diff))
            #posi = oldPosi + (diff)*avg
            posi = proj
    else:

        print("time diff: " + str(diff))
        if avg == 0:
            avg = speed
        if speed > 0:
            avg = avg*alpha + speed*(1-alpha)
        if avgRotDiff == 0:
            avgRotDiff = (oldPosi-posi)/diff
        avgRotDiff = avgRotDiff*rotAlpha + ((oldPosi-posi)/diff)*(1-rotAlpha)

    if posi - avgRotDiff*diff <= 0 and not blind:
        blind = True
        proj = posi + 1227.6
        posi = proj
    if start and posi < end -7 or tme - rotTime > .5:
        start = False
    if posi > zeroPos -7 and posi < zeroPos +7 and not start and (rotTime == 0 or tme-rotTime > .5):
        rottimes.append(tme-rotTime)
        rotTime = tme
        rots = rots + 1
        print("rots " + str(rots))
    pos = end - rots*360/(100/341) + (posi -1227.6)
    if pos < 0:
        pos = pos + 1024 + 60
    print("pos " + str(pos) + " posi "+str(posi)  + " Proj " + str(proj) + " rots " + str(rots) + " speed " + str(speed) + " avg "+str(avg)  +  " avgRotDiff "+str(avgRotDiff))
    oldPosi = posi
    tme = time.time()
print(posi)
print(zeroPos)
print(off)
print(err)
print(rottimes)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_MOVING_SPEED, 0)
# Disable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()
