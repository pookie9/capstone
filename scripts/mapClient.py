#!/usr/bin/env python

#Need to have rosrun gmapping slam_gmapping 
# and roslaunch turtlebot_stage turtlebot_in_stage.launch

import sys;
import rospy;
from nav_msgs.msg import OccupancyGrid;
from nav_msgs.srv import GetMap;
import numpy as np;
import cv2;


def showMap(trans):
    (width,height,data,info)=getMap()
    
    #grid_x = (unsigned int)((map_x - map.info.origin.position.x) / map.info.resolution)
    #grid_y = (unsigned int)((map_y - map.info.origin.position.y) / map.info.resolution)

    robotX=int((trans[0]-info.origin.position.x)/info.resolution)
    robotY=int((trans[1]-info.origin.position.y)/info.resolution)
    print "(robotX,robotY)"+str((robotX,robotY))
    left=width #Most extreme values....
    right=-1
    top=0
    bottom=height
    data=np.array(data)
    data.resize(height,width)

    known=np.where(data>=0)
    print len(known[0])
    first=(min(known[0]),min(known[1]))
    last=(max(known[0]),max(known[1]))
    first=(min(first[0],robotX),min(first[1],robotY))
    last=(max(last[0],robotX),max(last[1],robotY))
    print first
    print last

    m=data[first[0]:last[0],first[1]:last[1]]
    f=open('map','w+')
    f.write('RESOLUTION:\n'+str(info.resolution)+'\nROBOTPOS:\n('+str(robotX-first[0])+', '+str(robotY-first[1])+')\nMAP:\n')
    f.write(str(list(m)))
    f.close()

    im=np.zeros((height,width,3))

    im[(data>=0) & (data<50)]=(0,255,0)
    im[data>=50]=(255,0,0)

    cv2.circle(im,(robotX,robotY),5,(0,0,255),thickness=2)
    im=im[first[0]:last[0],first[1]:last[1]]
    im=cv2.resize(im,(400,400))
    cv2.imshow("Dynamic Map",im)
    cv2.waitKey(0)
     
     

def getMap():
    print "Requesting map"
    r=dynamic_map_client()

    width=r.map.info.width
    height=r.map.info.height
    data=r.map.data
    info=r.map.info

    print "Info: "+str(info)
    print "Width: "+str(width)
    print "Height: "+str(height)
    return (width,height,data,info)
                        
    
def dynamic_map_client():
    rospy.wait_for_service('dynamic_map')
    try:
        dynamic_map = rospy.ServiceProxy('dynamic_map', GetMap)
        resp1 = dynamic_map()
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
