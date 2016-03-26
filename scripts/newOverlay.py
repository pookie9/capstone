import sys
import ast
import numpy as np
import cv2
import cv2.cv as cv
import math
from matplotlib import pyplot as plt
from operator import mul
from itertools import imap
from scipy import ndimage

import scipy
import Image
import Locator
debug=False
class Overlay:
    gmapRes = 0.0
    gmapPos = (0, 0, 0.0)
    gArray2d = []
    gmapCV = None
    gmapCV_bot = None
    
    overheadPic = None
    overheadPos = None
    distLights = .24 #meters
    overheadRes= None

    degreeDifference = 0
    radsDifference = 0
    
    #constructor... filler for now
    def __init__(self, gmap, overheadPic,gPos,overheadPos,gRes):
        (cols, rows) = np.shape(gmap)
        self.gmapCV = np.zeros((cols,rows))   #this may be a bug... I think it should be rows, columns, 3
        self.gmapCV[gmap >= 50] = 255
        self.gmapCV[(gmap >= 0) & (gmap < 50)] = 0

        im = self.gmapCV.copy()
        #draw the ROBOT!!! with an arrow...
#        cv2.circle(im, (gPos[0], gPos[1]), 5, (0,0,225),thickness=2)
#        cv2.line(im, (gPos[0],gPos[1]),(gPos[0]+int(20*math.cos(gPos[2])),gPos[1]+int(20*math.sin(gPos[2]))),(0,0,255),thickness=3)
        self.gmapCV_bot = im

        self.gPos=(gPos[1], gPos[0], gPos[2]*-1) #KING!!! HACKER!!! HACKETTY HACKETTY HACK!
        if debug:
            print "GPOS: "+str(self.gPos)
        self.gRes=1.0/gRes
        self.overheadPos=overheadPos[:3]
        self.overheadPic=overheadPic
        self.overheadRes=overheadPos[3]

    def readOverhead(self, filename):
        pass
        
    def euclidDist(self, p1, p2):
        return ((p1[0]-p2[0])**2.0+(p1[1]-p2[1])**2.0)**.50

    #Returns the midpoint of two points... the average of the two
    def getMidPoint(self, p1, p2):
        return ((p1[0]+p2[0])/2, (p1[1]+p2[1])/2.0)

    #takes angle in degrees!
    #may be backwards
    def rotateImage(self, gmapImage):
        #find center
        centerPoint = (gmapImage.shape[1]//2, gmapImage.shape[0]//2) #(columns, rows)
        #how many degrees
        if debug:
            print('anglegmap: ', self.gPos[2])
            print('angleOH: ', self.overheadPos[2])
            print('anglegmapDegrees: ', (180.0/math.pi)*self.gPos[2])
            print('angleOH: ', (180.0/math.pi)*self.overheadPos[2])
        radsToRotate = -1*(self.gPos[2] - self.overheadPos[2])
        degreesToRotate = (180.0/math.pi)*radsToRotate
        self.radsDifference = radsToRotate
        self.degreeDifference = degreesToRotate
        if debug: 
            print('rotating this many degrees ', degreesToRotate, radsToRotate)
        return ndimage.interpolation.rotate(gmapImage, degreesToRotate, reshape=False)
        
        #rotate about center
                
        #M = cv2.getRotationMatrix2D(centerPoint, (180.0/math.pi)*radsToRotate, 1.0) #GOOGLE WHAT SCALE PARAMETER IS!!!
        #return cv2.warpAffine(gmapImage, M, (gmapImage.shape[0], gmapImage.shape[1])) #is this resized/correct???
        #return cv2.warpAffine(gmapImage, M, (image.shape[0], image.shape[1])) #is this resized/correct???

    def biggerPicture(self, im):
        rows = im.shape[0]
        cols = im.shape[1]
        diagonal = (rows**2+cols**2)**.5
        ratio = float(cols)/rows
        newRows = int(diagonal)#10 + diagonal*math.sin(45)
        newCols = int(diagonal*ratio)#10 + diagonal*math.cos(45)
        newIm = np.zeros((newRows, newCols)) #could just do diagonals for bot
        newCenter = (newIm.shape[0]//2, newIm.shape[1]//2)
        oldCenter = (im.shape[0]//2, im.shape[1]//2)
        
        dRow = newCenter[0]-oldCenter[0] 
        dCol = newCenter[1]-oldCenter[1]
        print('shapes: ', newIm.shape, im.shape)
        for xx,row in enumerate(im):
            for kk,pixel in enumerate(row):
                newIm[xx+dRow][kk+dCol] = pixel

        distFromMiddle = self.euclidDist((self.gPos[0], self.gPos[1]), oldCenter) 
        dY = oldCenter[1] - self.gPos[1] 
        dX = oldCenter[0] - self.gPos[0]
        gamma = math.atan2(dY,dX)
        
        newGmapPos = (int(newCenter[1]-distFromMiddle*math.sin(gamma)), newCenter[0]-int(distFromMiddle*math.cos(gamma)))

        self.gPos = (newGmapPos[0], newGmapPos[1], self.gPos[2])
        return (distFromMiddle,gamma,newIm)
    
    def overlay(self):
        #resize
        #tmpGmap1 = self.gmapCV.copy() #returns a copy of the gmap
        tmpGmap1 = self.gmapCV_bot.copy() #returns a copy of the gmap
        scaleRatio = self.overheadRes/self.gRes
        middle = (tmpGmap1.shape[0]//2, tmpGmap1.shape[1]//2)

        distFromMiddle = self.euclidDist((self.gPos[0], self.gPos[1]), middle)
        if debug:
            print('before dist: ', self.gPos, middle, distFromMiddle)
        if debug:
            cv2.circle(tmpGmap1, (middle[1], middle[0]),int(distFromMiddle),(0, 0, 255),thickness=8)
            cv2.circle(tmpGmap1, (middle[1], middle[0]), 5,(255, 0, 0),thickness=2)
            cv2.circle(tmpGmap1, (self.gPos[1], self.gPos[0]), 5,(255, 0, 0),thickness=2)

            cv2.namedWindow('circly', cv2.WINDOW_NORMAL)
            cv2.imshow('circly', tmpGmap1)
            cv2.waitKey(0)       

            print('cool beanz: ', distFromMiddle, scaleRatio, middle, self.gPos)
        
        newCols = int(tmpGmap1.shape[1]*scaleRatio)
        newRows = int(tmpGmap1.shape[0]*scaleRatio)
        if debug:
            print "overheadRes: "+str(self.overheadRes)
            print "gRes: "+str(self.gRes)
            print "scaleRatio: "+str(scaleRatio)
            print "newCols: "+str(newCols)
            print "newRows: "+str(newRows)
        tmpGmap1 = cv2.resize(tmpGmap1, (newCols, newRows))
        distFromMiddle *= scaleRatio
        self.gPos = (scaleRatio*self.gPos[0], scaleRatio*self.gPos[1], self.gPos[2])
        if debug:
            cv2.namedWindow('resized', cv2.WINDOW_NORMAL)
            cv2.imshow('resized', tmpGmap1)
            cv2.waitKey(0)        


        #rotate
        #put the tmpGmap into a bigger image (black edges)
        (distFromMiddle,gamma,tmpGmap1) = self.biggerPicture(tmpGmap1)
        tmpGmap1 = self.rotateImage(tmpGmap1)
        middle = (tmpGmap1.shape[0]//2, tmpGmap1.shape[1]//2)  #redefine middle
        

        
        newGmapPos = (int(middle[1]-distFromMiddle*math.sin(gamma+self.radsDifference)), middle[0]-int(distFromMiddle*math.cos(gamma+self.radsDifference)))
        if debug:
            print('here comes block of shit distFromMiddle Middle gPos degreeDifference newGmapPos gamma\n')
            print(distFromMiddle, middle, self.gPos, self.degreeDifference, newGmapPos, gamma)



        if debug:
            cv2.circle(tmpGmap1, (newGmapPos[0], newGmapPos[1]),100,(255,255,225),thickness=20)
            cv2.circle(tmpGmap1, (middle[1], middle[0]),int(distFromMiddle),(0, 0, 255),thickness=8)
            cv2.namedWindow('resized, rotated, circled wn', cv2.WINDOW_NORMAL)
            cv2.imshow('resized, rotated, circled wn', tmpGmap1)
            cv2.waitKey(0)        
        #return 1:1 image of overhead(blacked out) and gmap(overhead dimensions)
        oh2 = self.overheadPic.copy()
        ohGmap = self.overheadPic.copy()
        dCol = newGmapPos[0]-self.overheadPos[0] 
        dRow = newGmapPos[1]-self.overheadPos[1]

        rowNum = 0
        for ii,row in enumerate(oh2):
            if debug:
                print rowNum, dRow, dCol
            if rowNum+dRow >= tmpGmap1.shape[0] or rowNum+dRow < 0:
                oh2[ii] = [0 for pixel in row]
                ohGmap[ii] = [0 for pixel in ohGmap[ii]]
                rowNum+=1
                continue
            col = 0
            for kk,pixel in enumerate(row):
                if col+dCol >= tmpGmap1.shape[1]:
                    row[kk] = 0
                    ohGmap[ii][kk] = 0
                elif tmpGmap1[rowNum+dRow][col+dCol] == 0:
                    row[kk] = 0
                    ohGmap[ii][kk] = 0
                else:
                    ohGmap[ii][kk] = tmpGmap1[rowNum+dRow][col+dCol]
                    tmpGmap1[rowNum+dRow][col+dCol] = 255
                col+=1
            rowNum+=1

        #print out new overhead image
        if debug:
            cv2.namedWindow('blackout overhead', cv2.WINDOW_NORMAL)
            cv2.imshow('blackout overhead', oh2)
            cv2.waitKey(0)

            cv2.namedWindow('one to one gmap', cv2.WINDOW_NORMAL)
            cv2.imshow('one to one gmap', ohGmap)
            cv2.waitKey(0)

            cv2.namedWindow('tmp1gmap again', cv2.WINDOW_NORMAL)
            cv2.imshow('tmp1gmap again', tmpGmap1)
            cv2.waitKey(0)
        return ohGmap
