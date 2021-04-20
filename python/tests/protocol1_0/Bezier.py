#!/usr/bin/env python
# -*- coding: utf-8 -*-
from Draw import Draw
import math

class Bezier:
    def __init__(self):
        self.xPoints = []
        self.yPoints = []
        self.draw = Draw()
    def GenerateCurv(self,resolution):
        x = 0
        y = 0
        n = len(self.xPoints)
        for j in range(resolution):
            for i in range(n):
                coefficient = math.factorial(n)/(math.factorial(i)*math.factorial(n-i))
                term = pow(j/resolution,i)*pow(1-j/resolution,n-i)
                x = x + coefficient*term*self.xPoints[i]
                y = y + coefficient*term*self.yPoints[i]

        
        
bezier = Bezier()
print("enter points with ()")
        inpt = input()
        open = 0
        comma = 0
        for i in range(len(inpt)):
            if inpt[i] == '(':
                open = i+1
            elif inpt[i] == ',':
                comma = i
                bezier.xPoints.append(int(inpt[open:comma]))
            elif inpt[i] == ')':
                bezier.yPoints.append(int(inpt[comma+1:i]))
        print(bezier.xPoints,bezier.yPoints)
        print("enter rezolution")