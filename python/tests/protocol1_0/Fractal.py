#!/usr/bin/env python
# -*- coding: utf-8 -*-
from Etch import Etch
import math

class Fractal:
    def __init__:
        self.points = []
        self.frontier = []
        self.etch = Etch()
    def generateTree(self,depth,lenght,angle):
        self.frontier.append([9000,6000])
        outOfBounds = False

        while depth > 0 or not outOfBounds:
            for point in self.frontier:
                