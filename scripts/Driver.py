#roslaunch turtlebot_bringup turtlebot.launch

import mapClient;
import square;
import tfListener;
import rospy;
import Locator;
import cv2;
import math;
import newOverlay;

camera_port=0
ramp_frames=0
cap = cv2.VideoCapture(0)

print "Done initializing"
rospy.init_node('Driver')
posListener=tfListener.tfListener()

while True:
    square.doSquare()
    (trans,rot)=posListener.getPos()
    print "(trans,rot) "+str((trans,rot))

    (gmap,gpos, gres)=mapClient.getMapAndPos(trans,rot)
    for i in range(20):
        junk,overheadPic=cap.read()
    overheadPos=Locator.getRobotPos(overheadPic)

    mapClient.showMapAndPos(trans,rot)
    cv2.imshow("Overhead",overheadPic)
    cv2.waitKey(0)
    o=newOverlay.Overlay(gmap,overheadPic,gpos,overheadPos,gres)
    o.overlay()
