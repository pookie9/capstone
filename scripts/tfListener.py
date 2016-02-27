#!/usr/bin/env python  

import rospy;
import tf;
import geometry_msgs.msg;
import roslib;
import transformations;

#Should be equivalent to rosrun tf tf_echo /map /base_link
class tfListener:
    def __init__(self):
        self.listener=tf.TransformListener()
        
    def getPos(self):
        rate=rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                #Pulled from here: http://answers.ros.org/question/10268/where-am-i-in-the-map/
#                (trans,rot) = self.listener.lookupTransform('map', 'odom', rospy.Time(0))
                (trans,rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
                rot=transformations.euler_from_quaternion(rot)
                return (trans,rot)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep()
