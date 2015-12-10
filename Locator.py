#Finley

#Note, we should probably clean this up. It works, but could be made a lot better. Change it to use HSL instead of RGB, and also speed it up. Probably change resolution and change averaging of points so that random stray blue points don't effect the position of the robot.


from PIL import Image;
from math import atan, sin, cos;
targetMarker=0 #red
lowerMarker=2  #blue
upperMarker=1  #green
targetLoc=False #First time we find target then we save it (assumes no moving target)
robotLoc=False
thresh=10 #How close in pixels the robot has to get to the target

#Returns true if the robot is on the target
def onTarget():
    assert robotLoc, "Robot not found yet"
    assert targetLoc, "Target not found yet"
    return abs(robotLoc[0]-targetLoc[0])<thresh and abs(robotLoc[1]-targetLoc[1])<thresh
        
#Returns the x/y coordinates (in pixels) of the center of the robot and the orientation in radians
#input=PIL image
#output=(x,y,orientation)
def locateRobot(im):
    global robotLoc
    lower=locateColor(im, lowerMarker)
    upper=locateColor(im, upperMarker)
    angle=atan(float(upper[1]-lower[1])/(float(upper[0]-lower[0])+.01))
    robotLoc=(lower[0]+(upper[0]-lower[0])/2, lower[1]+(upper[1]-lower[1])/2, angle)
    return robotLoc
#Returns the x/y coordinates (in pixels) of the target
def locateTarget(im):
    global targetLoc
    if not targetLoc:
        targetLoc=locateColor(im, targetMarker)
    return targetLoc


#Returns an image that only shows the "knowledge of the robot, so location and orientation of robot, location of target, and known walls
def getKnowledge(im):
    radius=5
    robot=locateRobot(im)
    target=locateTarget(im)
    pix=im.load()
    for i in range(im.size[0]):#Dimming the original image
        for j in range(im.size[1]):
            pix[i,j]=(pix[i,j][0]/2,pix[i,j][1]/2,pix[i,j][2]/2)

    for i in range(robot[0]-radius, robot[0]+radius):#Marking robot with green
        for j in range(robot[1]-radius, robot[1]+radius):
            pix[i,j]=(0,255,0)
    #Now to mark the angle that the robot is facing in green
    for i in range(40):
        x=robot[0]+i*cos(robot[2])
        y=robot[1]+i*sin(robot[2])
        pix[x,y]=(0,255,0)
    for i in range(target[0]-radius, target[0]+radius): #Marking target with blue
        for j in range(target[1]-radius, target[1]+radius):
            pix[i,j]=(0,0,255)
    return im
#returns the x/y coordinates (in pixels) of the center of mass of the specified color (0 for red, 1 for green, 2 for blue)
def locateColor(im, color):
    lowerLimit=150 #lower limit for target color
    upperLimit=110 #upper limit for non-target color
    pix=im.load()
    matched=[]
    #Finding pixels that match color
    for i in range(im.size[0]):
        for j in range(im.size[1]):
            good=True
            for k in range(3):
                if not ((k==color and pix[i,j][k]>lowerLimit) or (k!=color and pix[i,j][k]<upperLimit)):
                    good=False
            if good:
                matched.append((i,j))
    #Does simple average of x/y coordinates
    sumX=0
    sumY=0
    for pair in matched:
        sumX+=pair[0]
        sumY+=pair[1]
    return (sumX/(len(matched)),sumY/(len(matched)))


#Runs the locator on 4 different images, shows the knowledge found for each one
def locatorExample():
    im=Image.open("Finder1.jpg")
    getKnowledge(im).show()
    if onTarget():
        print "On target"
    else:
        print "Not on target"
    raw_input()
    im=Image.open("Finder2.jpg")
    getKnowledge(im).show()
    if onTarget():
        print "On target"
    else:
        print "Not on target"
    raw_input()
    im=Image.open("Finder3.jpg")
    getKnowledge(im).show()
    if onTarget():
        print "On target"
    else:
        print "Not on target"
    raw_input()
    im=Image.open("Finder4.jpg")
    getKnowledge(im).show()
    if onTarget():
        print "On target"
    else:
        print "Not on target"
