#King, Moore


import cv2;
#use machine learning to take in picture + output of Subsystem 1 (height values for each picture) to produce a classification of each part of the map as driveable or not driveable.

#Input
#    Overhead picture as OpenCV Image
#    2D array of heights where each element in the array corresponds to the height of an individual pixel
#Output
#    2D array of floating points between 0 and 1 where 0 is not driveable and 1 is driveable, each element corresponds to the driveability of each pixel
def getDriveable(img, heights):
    drivable=[[1]*img.size[0] for i in range(img.size[1])]
    return drivable
