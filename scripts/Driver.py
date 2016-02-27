#roslaunch turtlebot_bringup turtlebot.launch

import mapClient;
import time;
import square;
import tfListener;
import rospy;

rospy.init_node('Driver')
posListener=tfListener.tfListener()
while True:
    (trans,rot)=posListener.getPos()
    print "(trans,rot) "+str((trans,rot))
    mapClient.showMap(trans,rot)
    mapClient.showMapAndPos(trans,rot)
    square.doSquare()
