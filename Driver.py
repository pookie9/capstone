#Phillips, King

#Calls all of the other methods. Note, this will have to be replaced once we start using ROS, but this allows us to get started on the individual parts much more easily

from PIL import Image;
from freenect import sync_get_depth as get_depth, sync_get_video as get_video;
import numpy as np;
import Locator,Classifier,Router,Kinect;

img=Image.open("Finder1.jpg")#Images will eventually draw from webcam
robotPos=Locator.locateRobot(img)
targetPos=Locator.locateTarget(img)

#(depths,_)=get_depth() #need Kinect hooked up for this to work. Moore, can you create some simulations of the output of this?
depths=[[0]*img.size[0] for i in range(img.size[1])]
knownHeights=Kinect.getHeights(img,depths,robotPos)

drivable=Classifier.getDriveable(img, knownHeights)

(dist,waypoints)=Router.getRoute(drivable,robotPos,targetPos)
