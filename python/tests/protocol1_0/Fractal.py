#!/usr/bin/env python
# -*- coding: utf-8 -*-
from Etch import Etch
import math
PRINT_ETCH = True

class Fractal:
    def __init__(self):
        self.points = []
        self.frontier = [] #to be expanded
        self.etch = Etch()
    def generateTree(self,depth,lenght,ratio,angle):
        inBounds = True        
        if depth == 0:
            #root
            self.points.append([9000,0])
            inBounds = self.etch.checkInBounds(9000, lenght)
            if inBounds:
                self.points.append([9000, round(lenght)])
                self.frontier.append([9000, lenght,0])
                print(self.points)
                print(self.frontier)
                print("-----------------------------------------")
            else:
                print("out of bounds at depth:", depth)
        elif self.generateTree(depth-1, lenght, ratio, angle):
            points = []
            for point in self.frontier:
                #center around orgin                
                x = 0
                y = pow(ratio,depth)*lenght
                #left
                sin = math.sin(point[2]+angle)
                cos = math.cos(point[2]+angle)                               
                xL = x * cos - y * sin + point[0]
                yL = x * sin + y * cos + point[1]
                points.append([xL,yL,point[2]+angle])#third argument is total angle from orgin 
                #right
                sin = math.sin(point[2]-angle)
                cos = math.cos(point[2]-angle)
                xR = x * cos - y * sin + point[0]
                yR = x * sin + y * cos + point[1]
                points.append([xR,yR,point[2]-angle])#third argument is total angle from orgin 

                #if these or any in the past have been out of bounds
                inBounds = self.etch.checkInBounds(xL, yL) and self.etch.checkInBounds(xR, yR) and inBounds
            if inBounds:
                self.frontier = points
                for point in self.frontier:
                    self.points.append([round(point[0]),round(point[1])])
                print(self.points)
                print(self.frontier)
                print("-----------------------------------------")
            else:
                print("out of bounds at depth:", depth)
        return inBounds
    def drawTree(self):        
        self.etch.goto(self.points[0][0],self.points[0][1],PRINT_ETCH)
        i = 1
        visited = [0]
        while i > 0:
            visited.append(i)
            #print(i,visited)
            print(self.points[i][0],self.points[i][1],i,"]]]]]]]]]]]]]]]]]]]]]]]]]]]]]")
            self.etch.goto(self.points[i][0],self.points[i][1],PRINT_ETCH)
            if i*2 < len(self.points) and not(i*2 in visited):
                i = i*2
            elif i*2 < len(self.points) and not(i*2+1 in visited):
                i = i*2+1
            else:
                i = math.floor(i/2)
        self.etch.goto(self.points[0][0],self.points[0][1],PRINT_ETCH)

    def clear(self):
        self.points = []
        self.frontier = []
        self.etch.goto(0,0)

fractal = Fractal()


print("enter depth")
depth = int(input())
print("enter lenght")
lenght = float(input())
print("enter ratio")
ratio = float(input())
print("enter angle")
angle = float(input())
print(depth,lenght,ratio,math.radians(angle))
fractal.generateTree(depth,lenght,ratio,math.radians(angle))
fractal.drawTree()
fractal.clear()