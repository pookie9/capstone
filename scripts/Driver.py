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
    mapClient.showMap(trans)
    print (trans,rot)
    square.doSquare()
