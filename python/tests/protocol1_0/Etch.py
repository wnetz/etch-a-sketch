#!/usr/bin/env python
# -*- coding: utf-8 -*-



import os
import time
import math
import sys
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
ADDR_MX_GOAL_ACCELERATION   = 73

# Protocol version
PROTOCOL_VERSION            = 1.0

# Default setting
HORIZONTAL                  = 0
VERTICAL                    = 1
BAUDRATE                    = 1000000
DEVICENAME                  = 'COM3'

TORQUE_ENABLE               = 1
TORQUE_DISABLE              = 0
DXL_MAXIMUM_TORQUE          = 800
DXL_D                       = 254
DXL_I                       = 0
DXL_P                       = 20
DXL_MAXIMUM_SPEED           = 50 
DXL_MINIMUM_SPEED           = 5  
DIFF                        = DXL_MAXIMUM_SPEED-DXL_MINIMUM_SPEED
GIVE                        = 50 

class Etch:
    def __init__(self):        

        self.sourceFile = open('C:/Users/wnetz/Documents/etch-a-sketch/python/tests/protocol1_0/etchlog.txt', 'w')
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

        #self.packetHandler.write1ByteTxRx(self.portHandler, HORIZONTAL, ADDR_MX_GOAL_ACCELERATION, 1)
        #self.packetHandler.write1ByteTxRx(self.portHandler, VERTICAL, ADDR_MX_GOAL_ACCELERATION, 1)

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
        self.xLastPosition = 0
        self.yLastPosition = 0

    def checkInBounds(self,x,y):
        xmax = 18000
        ymax = 12000
        if x > xmax or y > ymax or x < 0 or y < 0:
            return False
        else:
            return True

    def goto(self,x,y,output = False): 
        #update self.x and self.y to adjust for time delay between last read, and servos stopping 
        self.x, xRotationPosition, self.y, yRotationPosition = self.getPosition(False)
        #to prevent using diferent values        
        xPosition = self.x
        yPosition = self.y
        #for approch behavior
        xInitialDistance = abs(x-xPosition)        
        yInitialDistance = abs(y-yPosition)
        if output:
            print("got to: (",x,",",y,")",file = self.sourceFile)

        xcount = 0
        ycount = 0
        #while not at goal
        while x - GIVE > xPosition or x + GIVE < xPosition or y - GIVE > yPosition or y + GIVE < yPosition:        
            #get current position
            xPosition, xRotationPosition, yPosition, yRotationPosition = self.getPosition(output)
            #update rotations
            xChange,yChange = self.setRotations(xPosition,x > xPosition,yPosition, y > yPosition,output)
            if xChange and yChange:
                xPosition, dont, yPosition, care = self.getPosition(output)
            elif xChange:
                xPosition, do, nt, care = self.getPosition(output)
            elif yChange:
                do, nt, yPosition, care = self.getPosition(output)

            if (xPosition < x and x < self.xLastPosition) or (xPosition > x and x > self.xLastPosition):
                xcount = xcount + 1
            if (yPosition < y and y < self.yLastPosition) or (yPosition > y and y > self.yLastPosition):
                ycount = ycount + 1
            if xcount == 2 or ycount == 2:
                x = xPosition
                y = yPosition            
            else:
                print("xcount: ",xcount," ycount: ",ycount,file = self.sourceFile)
                self.setVelocity(x,xPosition,xInitialDistance,y,yPosition,yInitialDistance,output) 
                if output:
                    print(x - GIVE > xPosition, x + GIVE < xPosition, y - GIVE > yPosition, y + GIVE < yPosition,x - GIVE > xPosition or x + GIVE < xPosition or y - GIVE > yPosition or y + GIVE < yPosition,file = self.sourceFile)
                    print("----------------------------------------",file = self.sourceFile)
                self.xLastPosition = xPosition
                self.yLastPosition = yPosition
        #stop servos once at goal
        self.packetHandler.write2ByteTxRx(self.portHandler,HORIZONTAL,ADDR_MX_MOVING_SPEED,0)
        self.packetHandler.write2ByteTxRx(self.portHandler,VERTICAL,ADDR_MX_MOVING_SPEED,0)
        #update self.x and self.y to adjust for time delay between last read, and servos stopping       

    def getPosition(self, output):
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
        if output:
            print("position(",xPosition,",",yPosition,") rotpos(",xRotationPosition,",",yRotationPosition,") startpos(",self.xStartPosition,",",self.yStartPosition,")",file = self.sourceFile)
        return xPosition, xRotationPosition, yPosition, yRotationPosition

    def setRotations(self,xPosition,xForward,yPosition,yForward,output):
        #allow cushion for error in reading position
        xChange = False
        yChange = False
        xdiff = abs(xPosition - self.xLastPosition)
        ydiff = abs(yPosition - self.yLastPosition)
        if xForward :
            if (xdiff > 3000):
                self.xRotations = self.xRotations + 1
                xChange = True
        else :
            if (xdiff > 3000):
                self.xRotations = self.xRotations - 1
                xChange = True
        
        if yForward:
            if (ydiff > 3000):
                self.yRotations = self.yRotations + 1
                yChange = True
        else:
            if (ydiff > 3000):
                self.yRotations = self.yRotations - 1
                yChange = True
        if output:
            print("diff(",xdiff,",",ydiff,") rots(", self.xRotations,",",self.yRotations,") forward(",xForward,",",yForward,")",file = self.sourceFile)
        return xChange,yChange

    def setVelocity(self,x,xPosition,xInitialDistance,y,yPosition,yInitialDistance,output):
        ratiox = .5
        if abs(yPosition - y) + abs(xPosition - x) > 0:
            ratiox = abs((xPosition - x))/(abs(yPosition - y) + abs(xPosition - x))
        ratioy = 1 - ratiox
        #velocity must be between DXL_MINIMUM_SPEED and DXL_MAXIMUM_SPEED
        #takes the servo closest to its goal to determin the total velocity shared
        velocity = 0
        if xInitialDistance < GIVE:
            #velocity = max(min(abs(y-yPosition)/yInitialDistance*DXL_MAXIMUM_SPEED,DXL_MAXIMUM_SPEED),DXL_MINIMUM_SPEED)
            ratiox = 0
            if output:
                print("x 0 vel",velocity,file = self.sourceFile)
        elif yInitialDistance < GIVE:
            #velocity = max(min(abs(x-xPosition)/xInitialDistance*DXL_MAXIMUM_SPEED,DXL_MAXIMUM_SPEED),DXL_MINIMUM_SPEED)
            ratioy=0
            if output:
                print("y 0 vel",velocity,file = self.sourceFile)
        else:
            #velocity = max(min(abs(x-xPosition)/xInitialDistance,abs(y-yPosition)/yInitialDistance)*DXL_MAXIMUM_SPEED,DXL_MINIMUM_SPEED)
            if output:
                print("vel",velocity,file = self.sourceFile)            
        #how far x has to go compared to y
        
        #velocity split between the two based on ration of distance to go
        xVelocity = DXL_MINIMUM_SPEED + ratiox*DIFF
        yVelocity = DXL_MINIMUM_SPEED + ratioy*DIFF
        #if the velocity is supposed to negative add 1023 since the last bit ditermins the direction
        if xPosition > x:
            xVelocity = xVelocity + 1024
        if yPosition > y:
            yVelocity = yVelocity + 1024
        #set velocities
        self.packetHandler.write2ByteTxRx(self.portHandler,HORIZONTAL,ADDR_MX_MOVING_SPEED,math.floor(xVelocity))
        self.packetHandler.write2ByteTxRx(self.portHandler,VERTICAL,ADDR_MX_MOVING_SPEED,math.floor(yVelocity))
        if output:
            print("ratio(",ratiox,",",ratioy,") vels(",xVelocity,",",yVelocity,")",file = self.sourceFile)

    def disconect(self):
        # Disable Dynamixel Torque
        self.packetHandler.write1ByteTxRx(self.portHandler, HORIZONTAL, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, VERTICAL, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)

        # Close port
        self.portHandler.closePort()