#!/usr/bin/env python
# -*- coding: utf-8 -*-



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
ADDR_MX_TORQUE_ENABLE       = 24
ADDR_MX_D                   = 26
ADDR_MX_I                   = 27
ADDR_MX_P                   = 28
ADDR_MX_MOVING_SPEED        = 32
ADDR_MX_TORQUE_LIMIT        = 34
ADDR_MX_PRESENT_POSITION    = 36

# Protocol version
PROTOCOL_VERSION            = 1.0

# Default setting
HORIZONTAL                  = 0
VERTICAL                    = 1
BAUDRATE                    = 1000000
DEVICENAME                  = 'COM5'

TORQUE_ENABLE               = 1
TORQUE_DISABLE              = 0
DXL_MAXIMUM_TORQUE          = 800
DXL_D                       = 254
DXL_I                       = 0
DXL_P                       = 20
DXL_MAXIMUM_SPEED           = 500 
DXL_MINIMUM_SPEED           = 15  

class Draw:
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
        #zero position
        self.xStartPosition, dont, care = self.packetHandler.read2ByteTxRx(self.portHandler,HORIZONTAL,ADDR_MX_PRESENT_POSITION)
        self.yStartPosition, dont, care = self.packetHandler.read2ByteTxRx(self.portHandler,VERTICAL,ADDR_MX_PRESENT_POSITION)
        #global position
        self.x = 0
        self.y = 0

        self.xRotations = 0
        self.yRotations = 0

        self.xRecord = self.xStartPosition
        self.yRecord = self.yStartPosition
        #for counting rotations
        self.xLastPosition = self.xStartPosition
        self.yLastPosition = self.yStartPosition

    def goto(self,x,y): 
        #to prevent using diferent values        
        xPosition = self.x
        yPosition = self.y
        #for approch behavior
        xInitialDistance = abs(x-xPosition)        
        yInitialDistance = abs(y-yPosition)
        #while not at goal
        while x - 100 > xPosition or x + 100 < xPosition or y - 100 > yPosition or y + 100 < yPosition:
            #get current position
            xPosition, xRotationPosition, yPosition, yRotationPosition = self.getPosition()
            #update rotations
            self.setRotations(xRotationPosition,x > xPosition,yRotationPosition, y > yPosition)
            self.setVelocity(x,xPosition,xInitialDistance,y,yPosition,yInitialDistance) 
            print(x - 100 > xPosition, x + 100 < xPosition, y - 100 > yPosition, y + 100 < yPosition,x - 100 > xPosition or x + 100 < xPosition or y - 100 > yPosition or y + 100 < yPosition)
            print("----------------------------------------")
            self.xLastPosition = xRotationPosition
            self.yLastPosition = yRotationPosition
        #stop servos once at goal
        self.packetHandler.write2ByteTxRx(self.portHandler,HORIZONTAL,ADDR_MX_MOVING_SPEED,0)
        self.packetHandler.write2ByteTxRx(self.portHandler,VERTICAL,ADDR_MX_MOVING_SPEED,0)
        #update self.x and self.y to adjust for time delay between last read, and servos stopping
        self.x, self.xLastPosition, self.y, self.yLastPosition = self.getPosition()

    def getPosition(self):
        #get servos local position
        xRotationPosition, dont, care = self.packetHandler.read2ByteTxRx(self.portHandler,HORIZONTAL,ADDR_MX_PRESENT_POSITION)
        yRotationPosition, dont, care = self.packetHandler.read2ByteTxRx(self.portHandler,VERTICAL,ADDR_MX_PRESENT_POSITION)
        #position if start position were 0
        xRelativePosition = int(round((xRotationPosition - self.xStartPosition)/10)*10)
        yRelativePosition = int(round((yRotationPosition - self.yStartPosition)/10)*10)
        if xRelativePosition < 0:
            xRelativePosition = xRelativePosition + 4095
        if yRelativePosition < 0:
            yRelativePosition = yRelativePosition + 4095
        #global position
        xPosition = xRelativePosition + self.xRotations*4095
        yPosition = yRelativePosition + self.yRotations*4095
        print(xPosition, xRotationPosition, yPosition, yRotationPosition)
        return xPosition, xRotationPosition, yPosition, yRotationPosition

    def setRotations(self,xRotationPosition,xForward,yRotationPosition,yForward):
        print(self.xLastPosition,xRotationPosition,self.xStartPosition, self.xRotations,xForward,self.yLastPosition,yRotationPosition,self.yStartPosition,self.yRotations,yForward)
        #allow cushion for error in reading position
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

    def setVelocity(self,x,xPosition,xInitialDistance,y,yPosition,yInitialDistance):
        ratiox = .5
        ratioy = .5
        #velocity must be between DXL_MINIMUM_SPEED and DXL_MAXIMUM_SPEED
        #takes the servo closest to its goal to determin the total velocity shared
        velocity = max(min(abs(x-xPosition)/xInitialDistance,abs(y-yPosition)/yInitialDistance)*DXL_MAXIMUM_SPEED,DXL_MINIMUM_SPEED)
        print(abs(x-xPosition)/xInitialDistance,abs(y-yPosition)/yInitialDistance,speed)
        #how far x has to go compared to y
        ratiox = abs((xPosition - x))/(abs(yPosition - y) + abs(xPosition - x))
        ratioy = 1 - ratiox
        #velocity split between the two based on ration of distance to go
        xVelocity = ratiox*velocity
        yVelocity = ratioy*velocity
        #if the velocity is supposed to negative add 1023 since the last bit ditermins the direction
        if xPosition > x:
            xVelocity = xVelocity + 1023
        if yPosition > y:
            yVelocity = yVelocity + 1023
        #set velocities
        self.packetHandler.write2ByteTxRx(self.portHandler,HORIZONTAL,ADDR_MX_MOVING_SPEED,math.floor(xVelocity))
        self.packetHandler.write2ByteTxRx(self.portHandler,VERTICAL,ADDR_MX_MOVING_SPEED,math.floor(yVelocity))
        print(ratiox,xVelocity,ratioy,yVelocity)

    def disconect(self):
        # Disable Dynamixel Torque
        self.packetHandler.write1ByteTxRx(self.portHandler, HORIZONTAL, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, VERTICAL, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)

        # Close port
        self.portHandler.closePort()

#xmax = 18000
#ymax = 12000