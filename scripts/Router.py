# Goutzounis

####JOSH KING ======> SCROLL ALL THE WAY TO THE BOTTOM###

#Input: 
#    2D array of driveability of each pixel
#    position of robot within that 2D array, given as tuple (x,y,angle) angle is in radians
#    position of target within that 2D array, (x,y)
#    radius is size of the robot
#Output: 
#    route to take to reach the target, or decide the target is unreachable by returning False. Note, robot will never be "on the target"
#    The format for the output is going to be a length two tuple, the first element is going to be the total distance of the route, the second element will be a list of coordinates to go to in succession in a straight line
import math
import copy
import heapq
import sys
import numpy as np
import cv2
import time
import Image

from copy import deepcopy


minDriveability = .5 #a tile that is not at least this driveable may as well be a wall
undriveablePixel = (-1,-1) 

#prints extended walls with badPoint
def displayWalls(array, badPoint):
    height = len(array)
    width = len(array[0])
    blank_image = np.zeros((height,width,3))
    for xx in range(len(array)):
        for yy in range(len(array[0])):
            if array[xx][yy] > 1.5:
                blank_image[xx][yy] = (0, 0 ,255)
            else:
                blank_image[xx][yy] = (255, 255 ,255)

    tmpB = (badPoint[1], badPoint[0])
    cv2.circle(blank_image, tmpB, 5, (0,0,0))
#    blank_image[:,0:0.5*width] = (255,0,0)      # (B, G, R)
#    blank_image[:,0.5*width:width] = (0,255,0)
    cv2.imshow("extendedWalls", blank_image)
    cv2.waitKey(0)

#Gives list of neighboring tiles of the node. Only tiles below the driveability threshold are included.
def getNeighbors(node, array, driveabilityThreshold):
    neighbors = set()
    xx = node[0]
    yy = node[1]
    xLen = len(array)-1
    yLen = len(array[0])-1
    neighbors.add((max(0,xx-1), max(0,yy-1)))
    neighbors.add((max(0,xx-1), yy))
    neighbors.add((max(0,xx-1), min(yLen,yy+1)))
    neighbors.add((xx, max(0,yy-1)))
    neighbors.add((xx, min(yLen,yy+1)))
    neighbors.add((min(xLen,xx+1), max(0,yy-1)))
    neighbors.add((min(xLen,xx+1), yy))
    neighbors.add((min(xLen,xx+1), min(yLen,yy+1)))
    neighbors.discard((xx, yy))

    finalNeighbors = []
    for loc in neighbors:
        if array[loc[0]][loc[1]] < driveabilityThreshold:
            finalNeighbors.append((loc[0],loc[1]))
            
    return finalNeighbors

#height and driveability have been kinda interchangeable throughout the code. this should be changed
def extendWalls(driveMap, radius):
    if radius == 0:
        return driveMap
    #calculate new radius
    newRadius = int(math.ceil(float(radius)/(math.cos(2.0*math.pi*(45.0/360.0)))))
    driveMap2 = copy.deepcopy(driveMap)
    for xx in range(len(driveMap2)):
        for yy in range(len(driveMap2[0])):
            locHeight = driveMap2[xx][yy]

            if driveMap[max(0,xx-newRadius)][yy] < locHeight:
                driveMap[max(0,xx-newRadius)][yy] = locHeight
            if driveMap[min(len(driveMap)-1,xx+newRadius)][yy] < locHeight:
                driveMap[min(len(driveMap)-1,xx+newRadius)][yy] = locHeight
            if driveMap[xx][max(0,yy-newRadius)] < locHeight:
                driveMap[xx][max(0,yy-newRadius)] = locHeight
            if driveMap[xx][min(len(driveMap[0])-1, yy+newRadius)] < locHeight:
                driveMap[xx][min(len(driveMap[0])-1, yy+newRadius)] = locHeight

        
"""    yPixels = len(driveMap[0])
    for xx in range(len(driveMap)):
        for yy in range(len(driveMap[0])):
            sys.stdout.write(str(driveMap[xx][yy]))
            if yy != yPixels-1:
                sys.stdout.write(",")
        print ""
    sys.exit()"""


#takes (2d ints) heightmap, (int, int) start, (int, int) finish, and (float/int) driveability threshold
#Will return None if the finish is not connected to the start
#todo: distance doesn't seem to work
def dijk(array, start, finish, driveabilityThreshold, knownDriveable, usingKnownDriveable=False):
    if usingKnownDriveable == True:
        array = deepcopy(array)#is this right?

    for row in array:
        for pixel in row:
            if pixel == -1: pixel = 0

    allVerticies = []
    visited = set()
    visited.add('')
    dist = 0
    prev = {}
    xFinish = finish[0]
    yFinish = finish[1]

    heapq.heappush(allVerticies, (0, start))
    for xx in range(len(array)):
        for yy in range(len(array[0])):
            newVertex = (xx,yy)
            if newVertex != start:
                prev[newVertex] = (None, float('inf'))
            else:
                prev[newVertex] = (None, float('0'))

    while len(allVerticies) > 0:
        node = (0, '')
        while node[1] in visited:
            node = heapq.heappop(allVerticies)
        nodeDist = node[0]
        node = node[1]
        visited.add(node)
        
        #terminate if found finish
        if node == finish:
            path = []
            lastNode = finish
            while lastNode != start:
                path = [lastNode] + path
                lastNode = prev[lastNode][0]
            path = [start] + path
            return (nodeDist, path)
                
        for neighbor in getNeighbors(node, array, driveabilityThreshold):
            xNode = node[0]
            yNode = node[1]
            xNeighbor = neighbor[0]
            yNeighbor = neighbor[1]
            xDistance = -1
            yDistance = 0#math.fabs(array[xNeighbor][yNeighbor] - array[xNode][yNode])
            if xNeighbor != xNode and yNeighbor != yNode:
                xDistance = 2.0**.5
            else:
                xDistance = 1.0
            neighborDist = (xDistance**2 + yDistance**2)**.5
            alt = prev[node][1] + neighborDist
            if alt < prev[neighbor][1]:
                prev[neighbor] = (node, alt)
                euclidDist = ((xNeighbor-xFinish)**2 + (yNeighbor-yFinish)**2)**.5
                heapq.heappush(allVerticies, (alt+euclidDist, neighbor))

    print "FAILED TO FIND THE FINISHING POINT!"
    if usingKnownDriveable == False:
        dijk(array, start, finish, driveabilityThreshold, knownDriveable, True)
    #if failed, look for alternate finish that may reveal more data. Otherwise
    #kill self
    return None

#checks if current point should be set to a waypoint or not. If it's a different angle relative to the previous waypoint, then it should. Otherwise, keep going.
def diffAngle(prevWP, prevPoint, curPoint):
    xPrevWP = prevWP[0]
    yPrevWP = prevWP[1]
    xCurPoint = curPoint[0]
    yCurPoint = curPoint[1]
    adjacent1 = math.fabs(xPrevWP-xCurPoint)
    adjacent2 = math.fabs(yPrevWP-yCurPoint)

#    print "xPrevWP, yPrevWP, xCurPoint, yCurPoint, adj1, adj2: "
#    print xPrevWP
#    print yPrevWP
#    print xCurPoint
#    print yCurPoint
#    print adjacent1
#    print adjacent2
    if adjacent1 <= 1.0 and adjacent2 <= 1.0: 
        return 0.0
    xPrevPoint = prevPoint[0]
    yPrevPoint = prevPoint[1]

    if yPrevWP - yPrevPoint != 0:
        anglePrev = float(xPrevWP-xPrevPoint)/float(yPrevWP-yPrevPoint)
        angleCur = float(xPrevWP-xCurPoint)/float(yPrevWP-yCurPoint)
        if anglePrev == angleCur:
            return 0.0
        return anglePrev - angleCur
    if yPrevWP - yCurPoint != 0:
        return 1.0
    return 0.0

def line(p1, p2):
    x0 = p1[0]
    y0 = p1[1]
    x1 = p2[0]
    y1 = p2[1]
    points_in_line = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            points_in_line.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            points_in_line.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    points_in_line.append((x, y))
    return points_in_line

def isValidPath(array, prevWPs, driveabilityThreshold):
    prevPoint = prevWPs[1][0]
    for point in prevWPs[1][1:]:
        curX = point[0]
        curY = point[1]
        prevX = prevPoint[0]
        prevY = prevPoint[1]
        pointsToCheck = []

        for loc in line(prevPoint, point):
            if array[loc[0]][loc[1]] >= driveabilityThreshold:
                print str(loc) + " height: " + str(array[loc[0]][loc[1]])
                return (point, False)
        prevPoint = point

    return (prevPoint, True)

#Need to work on this one takes points of type (int, int) 
def euclidDist(p1, p2):
    return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**.5

#Gets length of path from start all the way through all the waypoints
def getPathLength(start, wayPoints):
    count = 0
    for point in wayPoints:
        if point == start:
            break
        count += 1

    pathLength = 0
    curPoint = count
    for point in wayPoints[count:-1]:
        pathLength += euclidDist(point, wayPoints[curPoint+1])
        curPoint += 1
    return pathLength

#Gives points in route that are straight lines from each other
def getWayPoints(array, prevWPs, start, finish, driveabilityThreshold, knownDriveable):
    distPath = dijk(array, start, finish, driveabilityThreshold, knownDriveable)
    if distPath==None:
        return None
    wayPoints = []

    lastWP = distPath[1].pop(0) #THE START IS NO LONGER IN PATH
    prevPoint = lastWP
    for point in distPath[1]:
        if diffAngle(lastWP, prevPoint, point) > .05 or  euclidDist(lastWP, point) > 35: #figuring out minimum change in angle
            wayPoints.append(prevPoint)
            lastWP = prevPoint
        prevPoint = point
    wayPoints.append(finish)

    #On first iteration, prevWPs should be none
    if prevWPs[1] == None:
        print "FIRST RUN!!!"
        return (distPath[0], wayPoints)

    #have prevWPs start at current location
    count = 0
    for pp in prevWPs[1]:
        if pp == start:
            break
        count += 1
    prevWPs = (prevWPs[0], prevWPs[1][count:])
    
    #compare distances of the oldpath and the newpath
    curLoc = start
    oldPathDist = getPathLength(curLoc, prevWPs[1])
    newDist = distPath[0]
    minDistReduction = .50
    validNess = isValidPath(array, prevWPs, driveabilityThreshold)

    if oldPathDist*minDistReduction < newDist and validNess[1]:
        print "OLDPATH!!! oldDist " + str(oldPathDist) + "  newDist: " + str(newDist) + "   validNess: " + str(validNess)
        return (oldPathDist, prevWPs[1][prevWPs[1].index(curLoc):])
    else:
        print "NEWPATH!!! oldDist " + str(oldPathDist) + "  newDist: " + str(newDist) + "   validNess: " + str(validNess)
        displayWalls(array, validNess[0])
        return (distPath[0], wayPoints)
    
#    This should be super easy to get working at a basic level, however there are many interesting ways we can expand. Maybe it will decide everything above .5 is driveable and everything below is not driveable and just run djkstras on that. Or maybe it will decide to do most likely to succeed path, e.g. if there are a lot of .50001s in a row you might want to avoid that area. This could be done by computing products (sum of logs really) or some other similar method.


#YOU ONLY GET TWO METHODS!
#The first is for all points and the second is for waypoints

#(2d int, (int, int), (int, int), float, float/int)
#returns (totalDistance, path)
def getRoute(driveable,robotPos,targetPos, radius, driveabilityThreshold):
    #extendWalls(driveable, math.ceil(radius))
    distPoints = dijk(driveable, robotPos, targetPos, driveabilityThreshold)
    return distPoints

#(2d int, (int, int), (int, int), float, True)
#returns list of points. But there is work done to keep track of distances
#knowDriveable is -1=unknown 1=known
def getRouteWP(driveable, prevWPs, robotPos, targetPos, radius, driveabilityThreshold, knownDriveable):
    print "Extending walls"
    extendWalls(driveable, math.ceil(radius))
    print "Getting waypoints"
    print "botLoc: "+str(robotPos)
    print "targetLoc: "+str(targetPos)
    print "driveable shape: "+str((len(driveable),len(driveable[0])))
    print "knownDriveable shape: "+str((len(knownDriveable),len(knownDriveable[0])))
    distPoints = getWayPoints(driveable, prevWPs, robotPos, targetPos, driveabilityThreshold, knownDriveable)
    print "here2"
    return distPoints

