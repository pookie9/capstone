#!/usr/bin/env python

#Need to have rosrun gmapping slam_gmapping 
# and roslaunch turtlebot_stage turtlebot_in_stage.launch

import sys;
import rospy;
from nav_msgs.msg import OccupancyGrid;
from nav_msgs.srv import GetMap;
import numpy as np;
import cv2;
import math;

def showMapAndPos(trans,rot):
    (data,pos,resolution)=getMapAndPos(trans,rot)
    (width,height)=np.shape(data)

#    f=open('map','w+')
#    f.write('RESOLUTION:\n'+str(resolution)+'\nROBOTPOS:\n'+str(pos)+'\nMAP:\n')
#    f.write(str(list(map(list,data))))
#    f.write('\n')
#    f.close()

    im=np.zeros((width,height,3))

    im[(data>=0) & (data<50)]=(0,255,0)
    im[data>=50]=(255,0,0)

    cv2.circle(im,(pos[0],pos[1]),5,(0,0,255),thickness=2)
    
    cv2.line(im, (pos[0],pos[1]),(pos[0]+int(20*math.cos(pos[2])),pos[1]+int(20*math.sin(pos[2]))),(0,0,255),thickness=3)
    #im=im[first[0]:last[0],first[1]:last[1]]
    im=cv2.resize(im,(400,400))
    cv2.imshow("Dynamic Map2",im)
    cv2.waitKey(0)
     
     

def getMapAndPos(trans,rot):
    r=dynamic_map_client()

    width=r.map.info.width
    height=r.map.info.height
    data=r.map.data
    info=r.map.info

    robotX=int((trans[0]-info.origin.position.x)/info.resolution)
    robotY=int((trans[1]-info.origin.position.y)/info.resolution)
    
    print "NEW before (robotX,robotY) "+str((robotX,robotY))
    left=width #Most extreme values....
    right=-1
    top=0
    bottom=height
    data=np.array(data)
    data.resize(height,width)

    known=np.where(data>=0)
    first=(min(known[0]),min(known[1]))
    last=(max(known[0]),max(known[1]))
    print "NEW temp first/last: "+str((first,last))
    first=(min(first[0],robotX),min(first[1],robotY))
    last=(max(last[0],robotX),max(last[1],robotY))
    print "NEW real first/last: "+str((first,last))
    data=data[first[0]:last[0],first[1]:last[1]]
    robotX=robotX-first[0]
    robotY=robotY-first[1]
    print "NEW first,last "+str((first,last))
    print "NEW after (robotX,robotY) "+str((robotX,robotY))
    return (data,(robotX,robotY,rot[2]),info.resolution)
                        
    
def dynamic_map_client():
    rospy.wait_for_service('dynamic_map')
    try:
        dynamic_map = rospy.ServiceProxy('dynamic_map', GetMap)
        resp1 = dynamic_map()
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e



def showMap(trans,rot):
    (width,height,data,info)=getMap()
    
    #grid_x = (unsigned int)((map_x - map.info.origin.position.x) / map.info.resolution)
    #grid_y = (unsigned int)((map_y - map.info.origin.position.y) / map.info.resolution)

    robotX=int((trans[0]-info.origin.position.x)/info.resolution)
    robotY=int((trans[1]-info.origin.position.y)/info.resolution)
    left=width #Most extreme values....
    right=-1
    top=0
    bottom=height
    data=np.array(data)
    data.resize(height,width)

###PETER START READING HERE

    known=np.where(data>=0)
    first=(min(known[0]),min(known[1]))
    last=(max(known[0]),max(known[1]))
    first=(min(first[0],robotX),min(first[1],robotY))
    last=(max(last[0],robotX),max(last[1],robotY))
    print "OLD first/last" +str((first,last))
    print "OLD robotX/robotY"+str((robotX,robotY))
    print "OLD origin: "+str(info.origin.position)
    im=np.zeros((height,width,3))

    im[(data>=0) & (data<50)]=(0,255,0)
    im[data>=50]=(255,0,0)

    cv2.circle(im,(robotX,robotY),5,(0,0,255),thickness=2)#CIRCLE 1
    cv2.line(im, (robotX,robotY),(robotX+int(20*math.cos(rot[2])),robotY+int(20*math.sin(rot[2]))),(0,0,255),thickness=3)
    print "OLD "+str((robotX-first[0],robotY-first[1]))
    im=im[first[0]:last[0],first[1]:last[1]]
    cv2.circle(im,(robotX-first[0],robotY-first[1]),5,(255,0,0),thickness=2)###THIS CIRCLE IS NOT THE SAME AS CIRCLE1
    im=cv2.resize(im,(400,400))

####PETER STOP READING HERE
    cv2.imshow("Dynamic Map",im)
    cv2.waitKey(0)
     
     

def getMap():
    r=dynamic_map_client()

    width=r.map.info.width
    height=r.map.info.height
    data=r.map.data
    info=r.map.info
    return (width,height,data,info)
