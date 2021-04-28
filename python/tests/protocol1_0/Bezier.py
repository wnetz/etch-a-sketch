#!/usr/bin/env python
# -*- coding: utf-8 -*-
from Etch import Etch
import math

class Bezier:
    def __init__(self):
        self.xPoints = []
        self.yPoints = []
        self.etch = Etch()
    def generateCurv(self,resolution):
        x = 0
        y = 0
        n = len(self.xPoints)
        coefficient = []
        for i in range(n):
            coefficient.append(math.factorial(n-1)/(math.factorial(i)*math.factorial(n-i-1)))            
        print(coefficient)

        for rez in range(resolution+1):
            x = 0
            y = 0
            t = rez/resolution
            for i in range(n):
                term = pow(t,i)*pow(1-t,n-i-1)
                x = x + coefficient[i]*term*self.xPoints[i]
                y = y + coefficient[i]*term*self.yPoints[i]
            print("(",x,",",y,")",rez,"]]]]]]]]]]]]]]]]]]]]]]]]]]]")
            self.etch.goto(x,y)
    def clear(self):
        self.xPoints = []
        self.yPoints = []

        
        
bezier = Bezier()
print("enter points with ()")
while 1:
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
    bezier.generateCurv(20)
    bezier.clear()
print("enter rezolution")