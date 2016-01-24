import cv2;
import math;
import numpy as np;

#Eventually will be the real deal
class Sensors:
    def __init__(self):
        print "Initializing real sensors"

#Simulator
class SimSensors:

    #mazePic is the path to the picture that is the background, heights is the corresponding 2D array of heights
    #initPos is the tuple of (x,y,orientation) in pixels
    def __init__(self,mazePic,heights,initPos,kinectHeight=5,scanAngleV=math.pi/4,scanAngleH=math.pi/4,minRange=1,maxRange=50,robotSpeed=50,robotPic="robot.png",targetPos=None,radius=10):
        print "Initializing simulated sensors"
        self.radius=radius
        self.targetPos=targetPos
        self.kinectHeight=kinectHeight #Height of the kinect on the robot
        self.scanAngleV=scanAngleV       #Vertical field of vision of scan in radians
        self.scanAngleH=scanAngleH       #Horizontal field of vision of scan in radians
        self.minRange=minRange #minRange of Kinect
        self.maxRange=maxRange #maxRange of Kinect
        self.robotSpeed=50
        self.robotPic=robotPic
        #Note that initPos=0 is straight to the right but because of (0,0) being in the top right the angle is incrementer counter-clockwise
        self.pos=initPos 
        self.mazePic=mazePic
        self.heights=[]
        f=open(heights,'r')
        for line in f:
            cur=[]
            line=line.split(',')
            for height in line:
                cur.append(int(height))
            self.heights.append(cur)
        self.height=len(self.heights[0])
        self.width=len(self.heights)
    #Returns an opencv picture with the robot overlayed at its position on the mazePic
    def getPic(self):
        rPic=cv2.imread(self.robotPic)
        curPic=cv2.imread(self.mazePic)
        (robotW,robotH,t)=rPic.shape
        rPic=255-rPic #inverting colors to get rid of black from rotate on corners
        M=cv2.getRotationMatrix2D((robotW/2,robotH/2),math.degrees(self.pos[2]),1)
        rPic=cv2.warpAffine(rPic,M,(robotW,robotH))#Rotating the robot to its correct orientation

        rPic=255-rPic #inverting back to normal
        rPic=cv2.resize(rPic,(2*self.radius,self.radius*2))
        (robotW,robotH,t)=rPic.shape
        for i in range(robotW):
            for j in range(robotH):
                if rPic[i,j][0]<200 or rPic[i,j][1]<200 or rPic[i,j][2]<200: #Not just white, actual robot
                    curPic[self.pos[1]+j-robotH/2,self.pos[0]+i-robotW/2]=rPic[i,j] #NOTE, opencv indexes by row then column, or y and then x
        return curPic

    #returns euclidian distance between points a and b
    @staticmethod
    def euclid(a,b):
        return math.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)

    #Returns the angle from point a to point b
    @staticmethod
    def angle(a,b):
        if b[0]==a[0]:
            if b[1]>a[1]:
                return math.pi/2
            else:
                return math.pi/-2
        else:
            res=math.atan(float(b[1]-a[1])/float(b[0]-a[0]))
            if (b[0]-a[0]<=0 and b[1]-a[1]<=0) or (b[0]-a[0]<=0 and b[1]-a[1]>=0):
                res+=math.pi
            return res

    #Returns a 3D point cloud, I belive the output of the ros LaserScan can easily be converted to this as described here:
    # http://wiki.ros.org/laser_pipeline/Tutorials/IntroductionToWorkingWithLaserScannerData#Generating_Laser_Scan_Data
    #Note, for now I am just guessing a 3D point cloud is a list of 3D points (x,y,z), should check this later
    def getKinectData(self):
        """
        To figure out which points are in the field of vision I create a cone whose tip is at kinect, and whose center line is
        the orientation of robot, and parallel to the ground you get the field of vision of the robot, note the bottom is rounded
        """
        #Figuring out the corner points on the outside of the sphere
        (junk, leftPoint,rightPoint,lowest,highest)=self.getFieldOfView()
        #Create the rays that go from the center and hit each of the outside points, treat as 2D, represented as length and slope 
        angle=SimSensors.angle((leftPoint[1],leftPoint[0]),(rightPoint[1],rightPoint[0]))-math.pi/2
        rays=[]
        i=0
        prev=(-1,-1)
        while True:
            i+=1
            (x,y)=(int(leftPoint[0]+math.cos(angle)*i),int(leftPoint[1]-math.sin(angle)*i))
            #print (x,y)
            #import time
            #time.sleep(1)
            if (rightPoint[0]-leftPoint[0])*(x-rightPoint[0])>=0 and (rightPoint[1]-leftPoint[1])*(y-rightPoint[1])>=0:
                break
            if prev[0]!=x or prev[1]!=y:#Checking for duplicates
                rays.append((SimSensors.angle(self.pos,(x,y)),SimSensors.euclid(self.pos,(x,y))))
            prev=[x,y]
        rays.append((SimSensors.angle(self.pos,rightPoint),SimSensors.euclid(self.pos,rightPoint)))
        points=[]
        #Tracing each ray from the beginning...
        for ray in rays:
            lowestAngle=-self.scanAngleV#Keeps track of the angle that it can see that is not blocked by another object
            for i in range(int(ray[1])+1):
                x=int(self.getFrontPos()[0]+i*math.cos(ray[0]))
                y=int(self.getFrontPos()[1]+i*math.sin(ray[0]))
                if min(x,y)<0 or max(x-len(self.heights),y-len(self.heights[0]))>0:
                    continue
                z=self.kinectHeight+i*math.sin(lowestAngle)#Minimum height that can be seen at this point
                if x<0 or x>=len(self.heights) or y<0 or y>=len(self.heights[0]):
                    continue
                if self.heights[x][y]>z:
                    points.append((x,y,self.heights[x][y]))
                    #setting lowestAngle to angle from kinect to top of object
                    if self.kinectHeight>self.heights[x][y]:
                        if i==0:
                            if self.kinectHeight-self.heights[x][y]>0:
                                lowestAngle=math.pi/2
                            else:
                                lowestAngle=math.pi/-2
                        else:
                            lowestAngle=-math.atan(float(self.kinectHeight-self.heights[x][y])/float(i))
                    elif self.kinectHeight<self.heights[x][y]:
                        if i==0:
                            if self.heights[x][y]-self.kinectHeight>0:
                                lowestAngle=math.pi/2
                            else:
                                lowestAngle=math.pi/-2
                        else:
                            lowestAngle=math.atan(float(self.heights[x][y]-self.kinectHeight)/float(i))
                    else:
                        lowestAngle=0.0    
        return points
    def getPos(self):
        return self.pos
    def getTargetPos(self):
        return self.targetPos
    def getFrontPos(self):
        return (int(self.pos[0]+self.radius*math.cos(self.pos[2])),int(self.pos[1]+self.radius*math.sin(self.pos[2])))
    #Returns a height by width array with 1 if a place is driveable, 2 if it is not driveable, 0 if unknown
    def getDriveable(self):
        heights=self.getKinectData()
        driveable=np.zeros((self.height, self.width))
        for h in heights:
            if h[2]>0:
                driveable[h[1],h[0]]=2
            if h[2]==0:
                driveable[h[1],h[0]]=1
        return driveable

    #Returns a list of five points, in order, center of field of view, leftmost point, rightmost point, lowest, highest
    def getFieldOfView(self):                
        hSpan=self.maxRange*math.sin(self.scanAngleH/2) #horizontal peripheral distance from center at maxRange 
        vSpan=self.maxRange*math.sin(self.scanAngleH/2) #vertical peripheral distance from center at maxRange 
        center=(int(self.getFrontPos()[0]+math.cos(self.pos[2])*self.maxRange), int(self.getFrontPos()[1]+math.sin(self.pos[2])*self.maxRange),self.kinectHeight)
        leftPoint=(int(center[0]+hSpan*math.cos(self.pos[2]-math.pi/2)),int(center[1]+hSpan*math.sin(self.pos[2]-math.pi/2)),self.kinectHeight)
        rightPoint=(int(center[0]+hSpan*math.cos(self.pos[2]+math.pi/2)),int(center[1]+hSpan*math.sin(self.pos[2]+math.pi/2)),self.kinectHeight)
        highest=[center[0],center[1],self.kinectHeight+vSpan]
        lowest=[center[0],center[1],self.kinectHeight-vSpan]
        return [center,leftPoint,rightPoint,lowest,highest]

    #newPos is the new position, len 2 tuple (x,y)
    #moves the robot to newPos in a straight line, updates self.pos
    #Also, checks that it can move there, if it can't it will move until it sees the wall that it would run into and stops
    def move(self, newPos):

        newOrientation=SimSensors.angle(self.pos,newPos)
        print "New Orientation: "+str(newOrientation)
        assert newOrientation!=self.pos[2] or newPos[1]!=self.pos[1] or newPos[0]!=self.pos[0], "Move to the same place as it currently is..."
        newPos.append(newOrientation)
        
        #Checking that path there is clear, not, treats it like point. Not true, but good enough for now
        clear=True
        badPoint=None
        for i in range(int(SimSensors.euclid(self.pos,newPos))+1):
            curX=self.getFrontPos()[0]+i*math.cos(newOrientation)
            curY=self.getFrontPos()[1]+i*math.sin(newOrientation)
            if self.heights[int(curX)][int(curY)]>0:
                clear=False
                badPoint=(int(curX),int(curY))

        if not clear:
            print "Obstuction in the way of move"
            #Now figure out where it sees that it is not clear....
            dist=SimSensors.euclid(badPoint, self.pos)
            if dist<self.maxRange: #Can see from curPos
                self.pos=(self.pos[0],self.pos[1],newOrientation)
                return
            else:#Can't see from curPos, move to point where it sees it...
                for i in range(int(SimSensors.euclid(self.pos,newPos)+1)):
                    curX=self.pos[0]+i*math.cos(newOrientation)
                    curY=self.pos[1]+i*math.sin(newOrientation)
                    if SimSensors.euclid((int(curX),int(curY)),badPoint)<self.maxRange:
                        self.pos=(int(curX),int(curY),newOrientation)
                        return
            

        #Now updating position and calculating time it would take
        self.pos=newPos
        dist=math.sqrt((self.pos[0]-newPos[0])**2+(self.pos[1]-newPos[1])**2)
    
    def showHeights(self):
        maxH=50.0
        img=np.zeros((len(self.heights[0]),len(self.heights),3), np.uint8)
        for i in range(len(self.heights)):
            for j in range(len(self.heights[0])):
                if self.heights[i][j]==0:
                    img[j,i]=(255,255,255)
                else:
                    img[j,i]=(0,0,255-(self.heights[i][j])*127/maxH)
        cv2.imshow('img',img)

    def showBot(self,waypoints=None):
        im=self.getPic()
        points=self.getFieldOfView()
        cv2.line(im, (self.getFrontPos()[0],self.getFrontPos()[1]), points[0][0:2],(0,0,255),1)
        cv2.line(im, (self.getFrontPos()[0],self.getFrontPos()[1]), points[1][0:2],(120,120,120),1)
        cv2.line(im, (self.getFrontPos()[0],self.getFrontPos()[1]), points[2][0:2],(120,120,120),1)
        if self.targetPos!=None:
            cv2.circle(im,self.targetPos,10,(0,0,255))
        points=self.getKinectData()
        maxH=50.0
        minH=0.0
        for point in points:
            h=point[2]
            if point[2]<minH:
                h=minH
            if point[2]>maxH:
                h=maxH
            if h==minH:    
                im[point[1],point[0]]=(0,255,0)
            else:
                im[point[1],point[0]]=(0,0,255-int(127*(h-minH)/maxH))
        if waypoints!=None:
            prev=tuple(self.getPos()[0:2])
            for point in waypoints:
                point=(point[1],point[0])
                cv2.line(im,prev,point,(255,0,0),2)
                cv2.circle(im,point,5,(255,0,0))
                prev=point
        cv2.imshow('image',im)
        cv2.waitKey(0)

