#Moore and Finley
#Mapping 3D Kinect data onto 2D picture.
#input:
#     2D picture as a OpenCV Image
#     Kinect data as output of ROS (need to figure out what that is)
#     robot position and orientation as a tuple of (x,y,angle). Angle is in radians, x and y correspond to pixel values of the center of the robot.
#returns 2D array of height value where the size of the array directly matches the size of the overhead picture, so each height value corresponds to one pixels height. Give a value of None to unknown heights.
import cv2;
def getHeights(img, depth, robotPos):
    heights=[[None]*img.size[0] for i in range(img.size[1])]#Need to update to cv2 from PIL 
    return heights

