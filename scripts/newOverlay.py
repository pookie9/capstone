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
        self.gmapCV = np.zeros((cols,rows,3))
        self.gmapCV[gmap >= 50] = (255, 0, 0)
        self.gmapCV[(gmap >= 0) & (gmap < 50)] = (0, 255, 0)        

        im = self.gmapCV.copy()
        #draw the ROBOT!!! with an arrow...
        cv2.circle(im, (gPos[0], gPos[1]), 5, (0,0,225),thickness=2)
        cv2.line(im, (gPos[0],gPos[1]),(gPos[0]+int(20*math.cos(gPos[2])),gPos[1]+int(20*math.sin(gPos[2]))),(0,0,255),thickness=3)
        self.gmapCV_bot = im

        self.gPos=(gPos[1], gPos[0], gPos[2]*-1) #KING!!! HACKER!!! HACKETTY HACKETTY HACK!
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
        print('anglegmap: ', self.gPos[2])
        print('angleOH: ', self.overheadPos[2])
        print('anglegmapDegrees: ', (180.0/math.pi)*self.gPos[2])
        print('angleOH: ', (180.0/math.pi)*self.overheadPos[2])
        radsToRotate = -1*(self.gPos[2] - self.overheadPos[2])
        degreesToRotate = (180.0/math.pi)*radsToRotate
        self.radsDifference = radsToRotate
        self.degreeDifference = degreesToRotate
        print('rotating this many degrees ', degreesToRotate, radsToRotate)
        return ndimage.interpolation.rotate(gmapImage, degreesToRotate, reshape=False)
        
        #rotate about center
                
        #M = cv2.getRotationMatrix2D(centerPoint, (180.0/math.pi)*radsToRotate, 1.0) #GOOGLE WHAT SCALE PARAMETER IS!!!
        #return cv2.warpAffine(gmapImage, M, (gmapImage.shape[0], gmapImage.shape[1])) #is this resized/correct???
        #return cv2.warpAffine(gmapImage, M, (image.shape[0], image.shape[1])) #is this resized/correct???
    
    def overlay(self):
        #resize
        #tmpGmap1 = self.gmapCV.copy() #returns a copy of the gmap
        tmpGmap1 = self.gmapCV_bot.copy() #returns a copy of the gmap
        scaleRatio = self.overheadRes/self.gRes
        middle = (tmpGmap1.shape[0]//2, tmpGmap1.shape[1]//2)

        distFromMiddle = self.euclidDist((self.gPos[0], self.gPos[1]), middle)
        print('before dist: ', self.gPos, middle, distFromMiddle)

        cv2.circle(tmpGmap1, (middle[1], middle[0]),int(distFromMiddle),(0, 0, 255),thickness=8)
        cv2.circle(tmpGmap1, (middle[1], middle[0]), 5,(255, 0, 0),thickness=2)
        cv2.circle(tmpGmap1, (self.gPos[1], self.gPos[0]), 5,(255, 0, 0),thickness=2)

        cv2.namedWindow('circly', cv2.WINDOW_NORMAL)
        cv2.imshow('circly', tmpGmap1)
        cv2.waitKey(0)       
 
        print('cool beanz: ', distFromMiddle, scaleRatio, middle, self.gPos)
        
        newCols = int(tmpGmap1.shape[1]*scaleRatio)
        newRows = int(tmpGmap1.shape[0]*scaleRatio)
        print "overheadRes: "+str(self.overheadRes)
        print "gRes: "+str(self.gRes)
        print "scaleRatio: "+str(scaleRatio)
        print "newCols: "+str(newCols)
        print "newRows: "+str(newRows)
        tmpGmap1 = cv2.resize(tmpGmap1, (newCols, newRows))
        distFromMiddle *= scaleRatio
        self.gPos = (scaleRatio*self.gPos[0], scaleRatio*self.gPos[1], self.gPos[2])
        
        cv2.namedWindow('resized', cv2.WINDOW_NORMAL)
        cv2.imshow('resized', tmpGmap1)
        cv2.waitKey(0)        

        #rotate
        print('BEFORE ROTATION ------ ', tmpGmap1.shape)
        tmpGmap1 = self.rotateImage(tmpGmap1)
        print('AFTER ROTATION ------ ', tmpGmap1.shape)
        middle = (tmpGmap1.shape[0]//2, tmpGmap1.shape[1]//2)  #redefine middle
        print('printing rotated image normal')
        #cv2.namedWindow('rotated', cv2.WINDOW_NORMAL)
        #cv2.imshow('rotated', tmpGmap1)
        #plt.imshow(tmpGmap1)
        #plt.show()
        print('printed rotated image normal')
        cv2.waitKey(0)


        #get gmap angle from middle (aka center of the image)
        #math.atan2(dY,dX)
        dY = middle[1] - self.gPos[1]
        dX = middle[0] - self.gPos[0]
        gamma = math.atan2(dY, dX)
#opposite        newGmapPos = (int(middle[1]+distFromMiddle*math.sin(gamma+self.radsDifference)), middle[0]+int(distFromMiddle*math.cos(gamma+self.radsDifference)))
        newGmapPos = (int(middle[1]-distFromMiddle*math.sin(gamma+self.radsDifference)), middle[0]-int(distFromMiddle*math.cos(gamma+self.radsDifference)))

        print('here comes block of shit distFromMiddle Middle gPos degreeDifference newGmapPos gamma\n')
        print(distFromMiddle, middle, self.gPos, self.degreeDifference, newGmapPos, gamma)

        
        #draw a circle on the map.  Find difference in column and row
#close...        cv2.circle(tmpGmap1, (newGmapPos[1], newGmapPos[0]),100,(255,255,225),thickness=20)
        cv2.circle(tmpGmap1, (newGmapPos[0], newGmapPos[1]),100,(255,255,225),thickness=20)

        cv2.circle(tmpGmap1, (middle[1], middle[0]),int(distFromMiddle),(0, 0, 255),thickness=8)
        cv2.namedWindow('resized, rotated, circled wn', cv2.WINDOW_NORMAL)
        cv2.imshow('resized, rotated, circled wn', tmpGmap1)
        cv2.waitKey(0)        
        '''
        cv2.circle(tmpGmap1, newGmapPos,100,(255,255,225),thickness=50)
        plt.imshow(tmpGmap1)
        plt.show()
        plt.imshow(self.overheadPic)
        plt.show()
        '''
        
        #cv2.namedWindow('post flip and resize', cv2.WINDOW_NORMAL)
        #cv2.imshow('post flip and resize', tmpGmap1)
        #cv2.waitKey(0)

        #return 1:1 image of overhead(blacked out) and gmap(overhead dimensions)
        oh2 = self.overheadPic.copy()
        ohGmap = self.overheadPic.copy()
        dCol = newGmapPos[0]-self.overheadPos[0] 
        dRow = newGmapPos[1]-self.overheadPos[1]

        rowNum = 0
        for ii,row in enumerate(oh2):
            print rowNum, dRow, dCol
            if rowNum+dRow >= tmpGmap1.shape[0] or rowNum+dRow < 0:
                oh2[ii] = [[0,0,0] for pixel in row]
                ohGmap[ii] = [[0,0,0] for pixel in ohGmap[ii]]
                rowNum+=1
                continue
            col = 0
            for kk,pixel in enumerate(row):
                if col+dCol >= tmpGmap1.shape[1]:
                    row[kk] = [0,0,0]
                    ohGmap[ii][kk] = [0,0,0]
                elif sum(tmpGmap1[rowNum+dRow][col+dCol]) == 0:
                    row[kk] = [0,0,0]
                    ohGmap[ii][kk] = [0,0,0]
                else:
                    ohGmap[ii][kk] = tmpGmap1[rowNum+dRow][col+dCol]
                    tmpGmap1[rowNum+dRow][col+dCol] = [255,255,255]
                col+=1
            rowNum+=1

        #print out new overhead image
        cv2.namedWindow('blackout overhead', cv2.WINDOW_NORMAL)
        cv2.imshow('blackout overhead', oh2)
        cv2.waitKey(0)

        cv2.namedWindow('one to one gmap', cv2.WINDOW_NORMAL)
        cv2.imshow('one to one gmap', ohGmap)
        cv2.waitKey(0)

        cv2.namedWindow('tmp1gmap again', cv2.WINDOW_NORMAL)
        cv2.imshow('tmp1gmap again', tmpGmap1)
        cv2.waitKey(0)

        
    #read overhead picture and get useful information
    def getOverheadHard(self, filename):
        self.overheadPic = cv2.imread(filename, cv2.IMREAD_COLOR)
        
        p1 = (1815,1839)
        p2 = (1550,2194)
        distance = self.euclidDist(p1, p2)
        dX = p1[0]-p2[0]
        dY = p1[1]-p2[1]
        midPoint = map(int, self.getMidPoint(p1, p2)) #X,Y is returned... opposite of what cv2 deals with
        midPoint = (midPoint[0], midPoint[1])
        orientation = -1.0 * (math.radians(180) + math.atan((dY/2.0)/(dX/2.0)))+ (-.264)  #correction factor for shitty math...
        resolution = distance / self.distLights #pixels per meter
        self.overheadRes = resolution
        print "overheadRes: " + str(resolution)
        print "orientation: " + str(orientation) + "\tdegrees: " + str(math.degrees(orientation))

        #draw line
        endPoint = (midPoint[0]+int(500*math.cos(orientation)), midPoint[1]+int(500*math.sin(orientation)))
        print midPoint
        print endPoint
        self.overheadPos = (midPoint[0], midPoint[1], orientation)
        
        cv2.line(self.overheadPic, midPoint, endPoint, (0,0,255), thickness=10)
        
        plt.imshow(self.overheadPic)
        plt.show()
        #plot hardcoded stuff onto picture to test correctness
        '''
        cv2.circle(self.overhead,(locationInfo[0],locationInfo[1]),25,(0,0,225),thickness=5)
        cv2.namedWindow('overhead', cv2.WINDOW_NORMAL)
        cv2.imshow('overhead', self.overhead)
        cv2.waitKey(0)
        '''

    #show the overhead picture with color
    def displayOverhead(self, filename):
        img = cv2.imread(filename, cv2.IMREAD_COLOR)
        cv2.namedWindow('overhead', cv2.WINDOW_NORMAL)
        cv2.imshow('overhead', img)
        cv2.waitKey(0)

    #read gmap 2d Array.
    #give the path to the file
    #return three tuple ==> (resolution, robot position, Map)
    #resolution is a float. robot position is (int, int, float). Map is list of lists of ints.
    #ROW THEN COLUMN
    def readGMAP(self, filename):
        content = ''
        with open(filename) as fin:
            content = fin.readlines()
        content = map(lambda xx: xx.strip(), content)
        
        #put values into variables
        self.gRes = 1.0/float(content[1]) #gives pixels per meter
        self.gPos = ast.literal_eval(content[3])
        self.gArray2d = ast.literal_eval(content[5])

        #make the cv gmap
        tmpGmap = np.array(self.gArray2d)
        (cols, rows) = np.shape(tmpGmap)
        self.gmapCV = np.zeros((cols,rows,3))
        self.gmapCV[tmpGmap >= 50] = (255, 0, 0)
        self.gmapCV[(tmpGmap >= 0) & (tmpGmap < 50)] = (0, 255, 0)

        return (self.gRes, self.gPos, self.gArray2d, self.gmapCV)

    #display gmap array graphically. THIS TAKES WHAT 'readGMAP' RETURNS!!!
    def displayGMAP(self):
        im = self.gmapCV.copy()
        pos = self.gPos
        #draw the ROBOT!!! with an arrow...
        cv2.circle(im, (pos[0], pos[1]), 5, (0,0,225),thickness=2)
        cv2.line(im, (pos[0],pos[1]),(pos[0]+int(20*math.cos(pos[2])),pos[1]+int(20*math.sin(pos[2]))),(0,0,255),thickness=3)
        self.gmapCV_bot = im
        
        cv2.imshow("Your GMAP", self.gmapCV)
        cv2.waitKey(0)

        cv2.namedWindow("Your gmap but window_normal and drawn bot", cv2.WINDOW_NORMAL)
        cv2.imshow("Your gmap but window_normal and drawn bot", im)
        cv2.waitKey(0)
        

        
#main. THIS IS FOR TESTING
#FINLEY WILL GIVE ME: ROBOT POS (X,Y,RADIANS) AND PICTURE RESOLUTION
if __name__ == '__main__':
    gmapFile = sys.argv[1]
    overheadFile = sys.argv[2]
    ol = Overlay(gmapFile, overheadFile) #read both files into class object
    ol.displayGMAP()
    ol.getOverheadHard(overheadFile)
    ol.overlay()
