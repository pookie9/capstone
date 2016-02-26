#!/usr/bin/env python  

import rospy;
import tf;
import geometry_msgs.msg;
import roslib;

#Should be equivalent to rosrun tf tf_echo /map /base_link
class tfListener:
    def __init__(self):
        self.listener=tf.TransformListener()
        
    def getPos(self):
        rate=rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.listener.lookupTransform('map', 'odom', rospy.Time(0))
                return (trans,rot)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep()
