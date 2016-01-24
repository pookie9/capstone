import Sensors;
import LRModel;
import Router;
import time;
import cv2;
import numpy;

heights=[]
f=open('BasicMazeHeights','r')
for line in f:
    cur=[]
    heights.append(map(int,line.strip().split(',')))
trueLabels=[]
for i in range(len(heights[0])):
    cur=[]
    for j in range(len(heights)):
        if heights[j][i]>0:
            cur.append(1)
        else:
            cur.append(2)
    trueLabels.append(cur)
trueLabels=numpy.array(trueLabels)
tPos=(794,454)
sim=Sensors.SimSensors("BasicMaze.png","BasicMazeActualHeights",[200,200,0],maxRange=400, targetPos=tPos)
background=cv2.imread("BasicMaze.png")
#targetPos=(794,554)

while True:
    labels=sim.getDriveable()
    
    model=LRModel.LRModel(background,labels)
    preds=model.predict(True)
    pos=sim.getPos()
    
    model.predictWithMetrics(trueLabels)
    model.predictAndShow(trueLabels)
    sim.showBot()
    waypoints=Router.getRouteWP(preds.tolist(),pos,tPos,5,1.0)
    print len(waypoints)
