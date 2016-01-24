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
minDriveability = 1 #a tile that is not at least this driveable may as well be a wall

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

    yPixels = len(driveMap[0])
"""    for xx in range(len(driveMap)):
        for yy in range(len(driveMap[0])):
            sys.stdout.write(str(driveMap[xx][yy]))
            if yy != yPixels-1:
                sys.stdout.write(",")
        print ""
    sys.exit()"""
                             
#takes (2d ints) heightmap, (int, int) start, (int, int) finish, and (float/int) driveability threshold
#Will return None if the finish is not connected to the start
#todo: distance doesn't seem to work
def dijk(array, start, finish, driveabilityThreshold):
    allVerticies = []
    visited = set()
    visited.add('')
    dist = {}
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
                prev[newVertex] = (start, float('0'))

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
            return (dist, path)
                
        for neighbor in getNeighbors(node, array, driveabilityThreshold):
            xNode = node[0]
            yNode = node[1]
            xNeighbor = neighbor[0]
            yNeighbor = neighbor[1]
            xDistance = -1
            yDistance = math.fabs(array[xNeighbor][yNeighbor] - array[xNode][yNode])
            if xNeighbor != xNode and yNeighbor != yNode:
                xDistance = 2.0**.5
            else:
                xDistance = 1.0
            neighborDist = (xDistance**2 + yDistance**2)**.5
            alt = prev[node][1] + neighborDist
            if alt < prev[neighbor][1]:
                prev[neighbor] = (node, alt)
                euclidDist = ((xNeighbor-xFinish)**2 + (yNeighbor-yNeighbor)**2)**.5
                heapq.heappush(allVerticies, (alt+euclidDist, neighbor))

    print "FAILED TO FIND THE FINISHING POINT!"
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
    if adjacent1 <= 1.0 and adjacent2 <= 1.0: 
        return False
    xPrevPoint = prevPoint[0]
    yPrevPoint = prevPoint[1]

    if yPrevWP - yPrevPoint != 0:
        anglePrev = float(xPrevWP-xPrevPoint)/float(yPrevWP-yPrevPoint)
        angleCur = float(xPrevWP-xCurPoint)/float(yPrevWP-yCurPoint)
        if anglePrev == angleCur:
            return False
        return True
    if yPrevWP - yCurPoint != 0:
        return True
    return False

#Gives points in route that are straight lines from each other
def getWayPoints(array, start, finish, driveabilityThreshold):
    distPath = dijk(array, start, finish, driveabilityThreshold)
#    print distPath[1]
    start = distPath[1][0]
    wayPoints = []

    prevWP = distPath[1].pop(0) #THE START IS NO LONGER IN PATH
    prevPoint = prevWP
    for point in distPath[1]:
        if diffAngle(prevWP, prevPoint, point):
            prevWP = point
            wayPoints.append(point)
        prevPoint = point
    wayPoints.append(finish)
    #distPath[1].insert(0, start) #START IS BACK IN THE PATH
    return (distPath[0], wayPoints)
    
#    This should be super easy to get working at a basic level, however there are many interesting ways we can expand. Maybe it will decide everything above .5 is driveable and everything below is not driveable and just run djkstras on that. Or maybe it will decide to do most likely to succeed path, e.g. if there are a lot of .50001s in a row you might want to avoid that area. This could be done by computing products (sum of logs really) or some other similar method.


#YOU ONLY GET TWO METHODS!
#The first is for all points and the second is for waypoints

#(2d int, (int, int), (int, int), float, float/int)
#returns (totalDistance, path)
def getRoute(driveable,robotPos,targetPos, radius, driveabilityThreshold):
    extendWalls(driveable, math.ceil(radius))
    distPoints = dijk(driveable, robotPos, targetPos, driveabilityThreshold)
    return distPoints

#(2d int, (int, int), (int, int), float, True)
#returns list of points. But there is work done to keep track of distances
def getRouteWP(driveable,robotPos,targetPos, radius, driveabilityThreshold):
    extendWalls(driveable, math.ceil(radius))
    distPoints = getWayPoints(driveable, robotPos, targetPos, driveabilityThreshold)[1]
    return distPoints

#distPoints=distPoints[1:]
#    t=[]
#    for point in distPoints:
#        point=point.split(',')
#        t.append(map(int,point))
#return t
