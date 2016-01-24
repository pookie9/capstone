import Sensors;
import LRModel;
import Router;
import cv2;
import numpy;
import time;

class Robot:
    """Used to call either work with SimSensor or Sensors"""
    def __init__(self,isSim,heights=None,rPic=None,backgroundPic=None,startPos=None, tPos=None,mRange=None,radius=5):
        self.isSim=isSim
        self.radius=radius
        if isSim:
            f=open(heights,'r')
            self.heights=[]
            for line in f:
                self.heights.append(map(int,line.strip().split(',')))
            self.trueLabels=[]
            for i in range(len(heights[0])):
                cur=[]
                for j in range(len(heights)):
                    if heights[j][i]>0:
                        cur.append(2)
                    else:
                        cur.append(1)
                self.trueLabels.append(cur)
            self.background=cv2.imread(backgroundPic)
            self.sensors=Sensors.SimSensors(backgroundPic,heights,startPos,robotPic=rPic,maxRange=mRange,targetPos=tPos)
            self.knownLabels=None
    @staticmethod
    def createSim(heights,robotPic,backgroundPic,startPos,targetPos,maxRange):
        r= Robot(True,heights,robotPic,backgroundPic,startPos,targetPos,maxRange)
        return r
    @staticmethod
    def createBot():
        return Robot(False)
    
    def learnAndPredict(self):
        newLabels=self.sensors.getDriveable()
        if self.knownLabels==None:
            self.knownLabels=newLabels
        assert newLabels.shape[0]==self.knownLabels.shape[0] and newLabels.shape[1]==self.knownLabels.shape[1], "shapes don't match: "+str(newLabels.shape)+" and "+str(self.knownLabels.shape)
        newKnown=numpy.where(newLabels>0)
        self.knownLabels[newKnown]=newLabels[newKnown]
        model=LRModel.LRModel(self.background,self.knownLabels)
        model.predictAndShow(targetPos=self.sensors.getTargetPos(),selfPos=self.sensors.getPos())
        preds=model.predict(True)
        return preds
    def run(self):
        while True:
            preds=self.learnAndPredict()
            print self.sensors.getPos()
            waypoints=Router.getRouteWP(preds.tolist(),(self.sensors.getTargetPos()[1],self.sensors.getTargetPos()[0]),(self.sensors.getPos()[1],self.sensors.getPos()[0]),self.radius,1.5)
            while waypoints[0][0]==int(self.sensors.getPos()[1]) and waypoints[0][1]==int(self.sensors.getPos()[0]):
                waypoints=waypoints[1:]
            print "Cur Pos"+str(self.sensors.getPos())
            print "Waypoints: "+str(waypoints)
            self.sensors.showBot(waypoints)
            
            self.sensors.move([waypoints[0][1],waypoints[0][0]])
            self.sensors.showBot(waypoints[1:])
            
