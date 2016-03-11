import sys
import ast
import numpy as np
import cv2
import cv2.cv as cv
import math
from matplotlib import pyplot as plt
from operator import mul
from itertools import imap



class Overlay:
    res = 0.0
    pos = (0, 0, 0.0)
    gArray2d = []

    overhead = None
    ohPos = None
    distLights = .24 #meters
    overheadRes= None
    
    #constructor... filler for now
    def __init__(self, gmap, overheadPic,gPos,overheadPos,gRes):
        self.gArray2d=gmap
        self.gPos=gPos
        self.gRes=gRes
        self.overheadPos=overheadPos[:3]
        self.overheadPic=overheadPic
        self.overheadRes=overheadPos[3]
    #returns radians gmapAngle-ohAngle
    def angleDifference(self):
        gmapAngle = self.gPos[2] #radians
        ohAngle = self.overheadPos[2] #radians

        if gmapAngle < 0:
            gmapAngle *= -1
            gmapAngle += 180

        if ohAngle < 0:
            ohAngle *= -1
            ohAngle += 180

        return gmapAngle - ohAngle
        
    def euclidDist(self, p1, p2):
        return ((p1[0]-p2[0])**2.0+(p1[1]-p2[1])**2.0)**.50
    
    #Returns the midpoint of two points... the average of the two
    def getMidPoint(self, p1, p2):
        return ((p1[0]+p2[0])/2, (p1[1]+p2[1])/2.0)

    #takes angle in degrees!
    #may be backwards
    def rotateImage(self, image, centerPoint, angle, scale=1.0):
        M = cv2.getRotationMatrix2D(centerPoint, angle, scale)
        return cv2.warpAffine(image, M, (image.shape[0], image.shape[1])) #is this resized/correctly???
        #return cv2.warpAffine(image, M, ((len(self.overheadPic[0])), len(self.overheadPic))) #is this resized???


        
    
    #returns one array. Convert all pixels from VALUE to (0,0,0) if they're unseen in the gmap
    def overlay(self):
        '''
        print "ppm gmap: " + str(ppmG)
        print "ppm overhead: " + str(self.overheadRes)
        print "gmap square meters: ", ((len(self.gArray2d) * len(self.gArray2d[0])) / ppmG**2)
        print "overhead square meters: ", (len(self.overheadPic) * len(self.overheadPic[0])/ self.overheadRes**2)
        print "gmap meters high: " + str(len(self.gArray2d) / ppmG) + "\toverhead meters high: " + str(len(self.overheadPic)/self.overheadRes)
        print "gmap meters wide: " + str(len(self.gArray2d[0]) / ppmG) + "\toverhead meters wide: " + str(len(self.overheadPic[0])/self.overheadRes)
        print "overhead pixel to gmap pixel: " + str(self.overheadRes**2/ppmG**2)
        print "gmap 1m**2: " + str(ppmG**2)
        print "overhead 1m**2: " + str(self.overheadRes**2)
        print "m**2 ratio: " + str(self.overheadRes**2/ppmG**2)
        print "increase dimensions by: " + str(math.sqrt(self.overheadRes**2/ppmG**2))
        '''
        #pre manipulation
        npG = self.displayGMAP()
        cv2.circle(npG,(self.gPos[0],self.gPos[1]),2,(255,255,225),thickness=3)
        cv2.namedWindow('pre-flipper', cv2.WINDOW_NORMAL)
        cv2.imshow('pre-flipper', npG)
        cv2.waitKey(0)
        
        ppmG = 1.0/self.gRes
        #rotate and resize the gmap to agree with the overhead picture
        rotation = -1*math.degrees(self.angleDifference()) #how much should I rotate
        print "rotation: " + str(rotation)
        print "rotating from: " + str(math.degrees(self.gPos[2]))
        #resize the image
        rotatedGmap = self.displayGMAP()
        rotationPoint = (rotatedGmap.shape[1]//2, rotatedGmap.shape[0]//2)  #assuming this is right (width, height)?... shape does (rows, col)
        robDistFromMiddle = self.euclidDist((self.gPos[1],self.gPos[0]), rotationPoint)
        rotatedGmap = self.rotateImage(rotatedGmap, rotationPoint, rotation) #should i rotate around the robot's location?
        newRow = -1*math.sin((math.degrees(self.gPos[2]) - rotation)+90)*robDistFromMiddle   # the +90 is because 0 is directly to the right
        newCol = -1*math.cos((math.degrees(self.gPos[2]) - rotation)+90)*robDistFromMiddle   # the +90 is because 0 is directly to the right
        newGmapPos = (rotationPoint[1]+newRow, rotationPoint[0]+newCol)          ####FIX THIS LATER... SUPER GHETTO
        print "x,y: " + str(newGmapPos)
        print "distance: " + str(robDistFromMiddle)


        #draw a circle on the new gmap
        #cv2.circle(rotatedGmap,newGmapPos,3,(0,0,225),thickness=5)
        #cv2.namedWindow('overhead', cv2.WINDOW_NORMAL)
        #rotatedGmap = cv2.resize(rotatedGmap, (len(self.overheadPic), len(self.overheadPic[0])))

        ratio = self.overheadRes/ppmG
        print "ratio: " + str(ratio)
        print int(len(rotatedGmap)*ratio), int(len(rotatedGmap[0])*ratio)
        print len(rotatedGmap), len(rotatedGmap[0])
        rotatedGmap = cv2.resize(rotatedGmap, (int(len(rotatedGmap)*ratio), int(len(rotatedGmap[0])*ratio)))
        newGmapPos = (int(ratio*newGmapPos[0]), int(ratio*newGmapPos[1]))          ####is this right???
        print rotatedGmap[0]

        #place circle
        cv2.circle(rotatedGmap,(newGmapPos[1],newGmapPos[0]),30,(255,255,225),thickness=20)
        cv2.namedWindow('flipper', cv2.WINDOW_NORMAL)
        cv2.imshow('flipper', rotatedGmap)
        cv2.waitKey(0)

        #make new copy of overhead image. Transpose the...
        oh2 = self.overheadPic.copy()



        #Suppose I do find that pixel... what do I do now?
        #get center of robot in oh
        
        print "new oh dimensions: ", oh2.shape
        print "new fipper dimensions: ", rotatedGmap.shape

        #find difference between row and column coordinates for the center of robot gmap-overhead
        dRow = newGmapPos[0]-self.overheadPos[0]
        dCol = newGmapPos[1]-self.overheadPos[1]
        #iterate through overhead picture assigning [-1,-1,-1] to all pixels that are black in the gmap
        rowNum = 0
        for row in oh2:
            print rowNum, dRow, dCol
            #print newGmapPos[0], self.overheadPos[0], newGmapPos[1], self.overheadPos[1]
            if rowNum+dRow >= rotatedGmap.shape[0]:
                for pixel in row:
                    pixel[0] = 0
                    pixel[1] = 0
                    pixel[2] = 0
                rowNum += 1
                continue
            col = 0
            for pixel in row:
                #make this better later
                if col+dCol >= rotatedGmap.shape[1]:
                    pixel[0] = 0
                    pixel[1] = 0
                    pixel[2] = 0
                    col+=1
                    continue
                if sum(rotatedGmap[rowNum+dRow][col+dCol]) == 0:
                    pixel[0] = 0
                    pixel[1] = 0
                    pixel[2] = 0
                #reset pixel in rotatedGmap to white to view changes
                rotatedGmap[rowNum+dRow][col+dCol] = [255,255,255]                
                col += 1
            rowNum += 1

        #print out new overhead image
#        cv2.namedWindow('new overhead', cv2.WINDOW_NORMAL)
#        cv2.imshow('new overhead', oh2)
#        cv2.waitKey(0)

#        cv2.namedWindow('new gmap', cv2.WINDOW_NORMAL)
#        cv2.imshow('new gmap', rotatedGmap)
#        cv2.waitKey(0)
        
        return rotatedGmap #Probably the same scale, but maybe not the same size
        
    #show the overhead picture with color
    def displayOverhead(self, filename):
        img = self.overheadPic
        cv2.namedWindow('overhead', cv2.WINDOW_NORMAL)
        cv2.imshow('overhead', img)
        cv2.waitKey(0)

    #display gmap array graphically. THIS TAKES WHAT 'readGMAP' RETURNS!!!
    def displayGMAP(self):
        res = self.gRes
        pos = self.gPos
        array = self.gArray2d
        totalPixels = 0
        for ll in array:
            for xx in ll:
                totalPixels += 1
        print "orientation: " + str(pos[2]) + "\tdegrees: " + str(math.degrees(pos[2]))
        
        nGmap = np.array(array)
        #these values may be mixed up
        (cols, rows)=np.shape(nGmap)
        print rows, cols
        print nGmap
        
        #print gmap
        im = np.zeros((cols, rows, 3))
        im[nGmap >= 50] = (255, 0, 0)
        im[(nGmap >= 0) & (nGmap < 50)] = (0, 255, 0)
        #draw the ROBOT!!! with an arrow...
        cv2.circle(im, (pos[0], pos[1]), 5, (0,0,225),thickness=2)
        cv2.line(im, (pos[0],pos[1]),(pos[0]+int(20*math.cos(pos[2])),pos[1]+int(20*math.sin(pos[2]))),(0,0,255),thickness=3)
        im2=cv2.resize(im, (400,400))
        cv2.imshow("Your GMAP", im2)
        cv2.waitKey(0)
        return im

        
#main. THIS IS FOR TESTING
#FINLEY WILL GIVE ME: ROBOT POS (X,Y,RADIANS) AND PICTURE RESOLUTION
"""if __name__ == '__main__':
    gmapFile = sys.argv[1]
    overheadFile = sys.argv[2]
    ol = Overlay(gmapFile, overheadFile) #read both files into class object
    #ol.displayGMAP()
    ol.getOverheadHard(overheadFile)
    ol.overlay()
"""
