#roslaunch turtlebot_bringup turtlebot.launch

import mapClient;
import square;
import tfListener;
import rospy;
import Locator;
import cv2;
import math;
import newOverlay;
import LRModel;
import numpy as np;
import Mover;
import Router;
import os;
import time;

def showData(pic,pos1,pos2,wps):
    cv2.circle(pic,(pos1[0],pos1[1]),10,(0,0,255))
    cv2.circle(pic,pos2,10,(0,255,0))
    prev=(pos1[1],pos1[0])
    for wp in wps:
        cv2.circle(pic,(wp[1],wp[0]),10,(255,0,0))
        if prev!=None:
            cv2.line(pic,(prev[1],prev[0]),(wp[1],wp[0]),(255,0,0),3)
        prev=wp
    cv2.imshow("Route",pic)
    cv2.waitKey(0)

def onTarget(robotPos,targetPos,threshold=30):
    dist=math.sqrt((robotPos[0]-targetPos[0])**2+(robotPos[1]-targetPos[1])**2)


radius=20
camera_port=0
ramp_frames=0
cap = cv2.VideoCapture(0)
targetPos=(180,160)
print "Done initializing"
rospy.init_node('Driver')
posListener=tfListener.tfListener()

while True:
#    square.doSquare()


    Mover.spin() #Spins in circle to gather data for gmapping
    time.sleep(3)
    (trans,rot)=posListener.getPos()
    print "(trans,rot) "+str((trans,rot))
    (gmap,gpos, gres)=mapClient.getMapAndPos(trans,rot)
    for i in range(20):
        junk,overheadPic=cap.read()
    overheadPos=Locator.getRobotPos(overheadPic)
    if onTarget(overheadPos,targetPos):
        print "On target"
        os.system('say "on target"')
        break
    scale=overheadPos[3]
    mapClient.showMapAndPos(trans,rot)
    
    cv2.circle(overheadPic,(180,160),10,(0,0,255))
    cv2.imshow("Overhead",overheadPic)
    cv2.waitKey(0)

    o=newOverlay.Overlay(gmap,overheadPic,gpos,overheadPos,gres)
    one2one=o.overlay()
    one2onepic=np.zeros((np.shape(one2one)[0],np.shape(one2one)[1],3))
    one2onepic[one2one==-1]=(0,0,0)#unknown
    one2onepic[one2one==0]=(0,255,0)#Driveable
    one2onepic[one2one==1]=(255,0,0)#wall
    cv2.imshow("Known 3d Data overlayed on overhead",one2onepic)
    cv2.waitKey(0)
    model=LRModel.LRModel(overheadPic,one2one)
    preds=model.predictAndShow(selfPos=overheadPos)
    

    print np.shape(one2one)
    print np.shape(preds)
    retVal=Router.getRouteWP(preds.tolist(),(-1,None),(overheadPos[1],overheadPos[0]),(targetPos[1],targetPos[0]),radius,.5,one2one.tolist())
    if retVal==None:
        print "No path from here to target"
        os.system('say "No path"')
        break
    else:
        (dist,waypoints)=retVal
    showData(overheadPic,overheadPos,targetPos,waypoints)
    Mover.moveTo(overheadPos,(waypoints[0][1],waypoints[0][0]),scale)


