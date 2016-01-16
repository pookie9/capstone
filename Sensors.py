import cv2;
import math;

class Sensors:
    def __init__(self):
        print "Initializing real sensors"

class SimSensors:
    robotPic="robot.png"
    robotSpeed=50 #speed in pixels per second
    maxRange=50
    minRange=1
    scanAngleH=math.pi/4  #Horizontal field of vision of scan in radians
    scanAnglenV=math.pi/4  #Vertical field of vision of scan in radians
    kinectHeight=5
    #mazePic is the path to the picture that is the background, heights is the corresponding 2D array of heights
    #initPos is the tuple of (x,y,orientation) in pixels
    def __init__(self,mazePic, heights, initPos):
        print "Initializing simulated sensors"
        self.robotPic
        #Note that initPos=0 is straight to the right but because of (0,0) being in the top right the angle is incrementer counter-clockwise
        self.pos=initPos 
        self.mazePic=mazePic
        self.heights=heights
    #Returns an opencv picture with the robot overlayed at its position on the mazePic
    def getPic(self):
        rPic=cv2.imread(SimSensors.robotPic)
        curPic=cv2.imread(self.mazePic)
        (robotW,robotH,t)=rPic.shape
        rPic=255-rPic #inverting colors to get rid of black from rotate on corners
        M=cv2.getRotationMatrix2D((robotW/2,robotH/2),math.degrees(self.pos[2]),1)
        rPic=cv2.warpAffine(rPic,M,(robotW,robotH))#Rotating the robot to its correct orientation
        rPic=255-rPic #inverting back to normal
        for i in range(robotW):
            for j in range(robotH):
                if rPic[i,j][0]<200 or rPic[i,j][1]<200 or rPic[i,j][2]<200: #Not just white, actual robot
                    curPic[self.pos[1]+j-robotH/2,self.pos[0]+i-robotW/2]=rPic[i,j] #NOTE, opencv indexes by row then column, or y and then x
        return curPic
    
    #Returns a 3D point cloud, I belive the output of the ros LaserScan can easily be converted to this as described here:
    # http://wiki.ros.org/laser_pipeline/Tutorials/IntroductionToWorkingWithLaserScannerData#Generating_Laser_Scan_Data
    #Note, for now I am just guessing a 3D point cloud is a list of 3D points (x,y,z), should check this later
    def getKinectData(self):
        """
        To figure out which points are in the field of vision I create a cone whose tip is at kinect, and whose center line is
        the orientation of robot, and parallel to the ground you get the field of vision of the robot, note the bottom is rounded
        """
        
        #Figuring out the corner points on the outside of the sphere
        (center, leftPoint,rightPoint,lowest,highest)=self.getFieldOfView()
        """
        To figure out which points are viewable i.e. not blocked by other objects in front of them I make the assumption that
        there are no overhanging objects, this limits the simulator slightly, but is good enough. Then I create rays for each
        point on the surface of the sphere that is inside the field of vision, each ray goes from the center of the sphere to 
        its point on the outside.
        Then for each one I go along its trajectory and keep track of of the up/down angle that it is going at assuming it hits
        the top of objects. Each time I hit a new object I add that to the 3D point cloud
        """

    #Returns a list of five points, in order, center of field of view, leftmost point, rightmost point, lowest, highest
    def getFieldOfView(self):                
        hSpan=SimSensors.maxRange*math.sin(SimSensors.scanAngleH/2) #horizontal peripheral distance from center at maxRange 
        vSpan=SimSensors.maxRange*math.sin(SimSensors.scanAngleH/2) #vertical peripheral distance from center at maxRange 
        center=(int(self.pos[0]+math.cos(self.pos[2])*SimSensors.maxRange), int(self.pos[1]+math.sin(self.pos[2])*SimSensors.maxRange),SimSensors.kinectHeight)
        leftPoint=(int(center[0]+hSpan*math.cos(self.pos[2]-math.pi/2)),int(center[1]+hSpan*math.sin(self.pos[2]-math.pi/2)),SimSensors.kinectHeight)
        rightPoint=(int(center[0]+hSpan*math.cos(self.pos[2]+math.pi/2)),int(center[1]+hSpan*math.sin(self.pos[2]+math.pi/2)),SimSensors.kinectHeight)
        highest=[center[0],center[1],SimSensors.kinectHeight+vSpan]
        lowest=[center[0],center[1],SimSensors.kinectHeight-vSpan]
        return [center,leftPoint,rightPoint,lowest,highest]

    #newPos is the new position, len 2 tuple (x,y)
    #moves the robot to newPos in a straight line, updates self.pos
    #returns the amount of time this would take in seconds
    def move(self, newPos):
        if newPos[0]==self.pos[0]:
            assert newPos[1]!=self.pos[1], "Move to the same place as it currently is..."
            if (newPos[0]>self.pos[0]):
                newOrientation=0
            else:
                newOrientation=math.pi
        else:
            newOrientation=math.atan(float(newPos[1]-self.pos[1])/float((newPos[0]-self.pos[0])))
        #Making sure newOrientation is in the correct quadrant
        if newPos[0]-self.pos[0]<0 and newPos[1]-self.pos[1]<0:
            newOrientation+=math.pi
        if newPos[0]-self.pos[0]<0 and newPos[1]-self.pos[1]>=0:
            newOrientation+=math.pi
        newPos.append(newOrientation)
        #Now updating position and calculating time it would take
        self.pos=newPos
        dist=math.sqrt((self.pos[0]-newPos[0])**2+(self.pos[1]-newPos[1])**2)
        t=dist/SimSensors.robotSpeed
        return t
        
    def showBot(self):
        im=sim.getPic()
        points=sim.getFieldOfView()
        print points
        print points[0][0:2]
        print self.pos[0:2]

        cv2.line(im, (self.pos[0],self.pos[1]), points[0][0:2],(0,0,255),1)
        cv2.line(im, (self.pos[0],self.pos[1]), points[1][0:2],(120,120,120),1)
        cv2.line(im, (self.pos[0],self.pos[1]), points[2][0:2],(120,120,120),1)
        cv2.imshow('image',im)
        cv2.waitKey(0)

sim=SimSensors("BasicMaze.png",[],[200,200,0])
cv2.waitKey(0)
sim.showBot()
sim.move([300,300])
sim.showBot()
sim.move([240,260])
sim.showBot()
cv2.destroyAllWindows()
