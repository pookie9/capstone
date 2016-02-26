#!/usr/bin/env python


import rospy
#from std_msgs.msg import nav_msgs/OccupancyGrid
from nav_msgs.msg import OccupancyGrid
import cv2;
import numpy as np;

def callback(data):
    height=data.info.height
    width=data.info.width
    print height
    print width
    data=data.data
    print len(data)
    img=np.zeros((height,width,3), np.uint8)
    for i in range(width):
        for j in range(height):
            img[i,j]=data[i*width+j]
    print "Showing"
 
    cv2.imshow('dst_rt', img)
    cv2.waitKey(0)
    
    
    
def listener():
    rospy.init_node('listener', anonymous=True)
    
    rospy.Subscriber("odom", OccupancyGrid, callback)

    rospy.spin()


if __name__ == '__main__':
    listener()
