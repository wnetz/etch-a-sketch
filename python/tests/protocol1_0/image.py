import sys
import cv2
from matplotlib import pyplot as plt
from skimage.filters import sobel
import numpy as np
import math
from Etch import Etch

PRINT_ETCH = True

class Image:
    def __init__(self):
        self.points = []
        self.image = cv2.imread("C:/Users/wnetz/Documents/etch-a-sketch/python/tests/protocol1_0/tri.png", 0)
        self.imageShape = 0
        self.etch = Etch()
        self.sourceFile = open('C:/Users/wnetz/Documents/etch-a-sketch/python/tests/protocol1_0/test.txt', 'w')
        self.sourceFile2 = open('C:/Users/wnetz/Documents/etch-a-sketch/python/tests/protocol1_0/test2.txt', 'w')
        np.set_printoptions(threshold=sys.maxsize)
    def processImage(self):
        self.imageShape = self.image.shape
        sig = .3
        median = np.median(self.image)
        lower = int(max(0,(1.0-sig)*median))
        upper = int(min(255,(1.0+sig)*median))
        self.image = cv2.Canny(self.image,lower,upper)
        plt.imshow(self.image, cmap='gray')
        plt.show()
    def sort(self):
        #loop x
        for x in range(self.imageShape[0]):
            #loop y
            for y in range(self.imageShape[1]):
                #if there is an edge pixle
                if self.image[x][y] == 255:
                    point = (((x  -self.imageShape[1] + 1) * -1) * 18000/self.imageShape[1], y * 12000/self.imageShape[0])
                    self.points.append(point)
                    #print ("("+str(point[0]) + "," + str(point[1])+")")
                    print("X",end='',file = self.sourceFile)
                else:
                    print(" ",end='',file = self.sourceFile)
            print("",file = self.sourceFile)
        print(len(self.points))
    def drawImage(self):
        avg = 0
        numpoints = 0
        minpoint = [0,0]
        length = len(self.points)
        
        while len(self.points) > 1:
            oldmin = minpoint
            min = math.pow(math.pow(18000,2) + math.pow(12000,2),.5)
            minpoint = []
            lessmin = []
            for point in self.points:
                dist = math.pow(math.pow(point[0]-oldmin[0],2) + math.pow(point[1]-oldmin[1],2),.5)
                if min < dist and dist < 100:
                    lessmin.append(point)
                if dist < min:
                    min = dist
                    minpoint = point
                #if min < 3:
                    #break
            if len(minpoint) > 0:
                print(str(min) + " (" + str(minpoint[0]) + "," + str(minpoint[1]) + ")", file = self.sourceFile2)
            if min > 1:
                avg = avg + min
                numpoints = numpoints + 1
            for point in lessmin:
                self.points.remove(point)
            if len(minpoint) > 0:
                self.points.remove(minpoint)
                self.etch.goto(minpoint[0],minpoint[1],PRINT_ETCH)
            if len(self.points) % 1000 == 0:
                print(len(self.points))
            print(str(min) + " (" + str(minpoint) + ") ",len(self.points))
            
        print("total " + str(avg) + " " + str(numpoints))
        print("total " + str(avg/numpoints))
    def end(self):
        self.sourceFile.close()
        self.sourceFile2.close()
        self.etch.goto(0,0,PRINT_ETCH)

image = Image()
#print("enter image path")
#print(input())
image.processImage()
image.sort()
image.drawImage()
image.end()


