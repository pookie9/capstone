
#In another terminal run  roslaunch kobuki_node minimal.launch --screen

import rospy;
from geometry_msgs.msg import Twist;
import time;

def MoveLeg():
        rate=5
        Pub = rospy.Publisher('mobile_base/commands/velocity' , Twist , queue_size=20)

        command = Twist()
        command.linear.x = .2
        command.linear.y = 0
        command.linear.z = 0
        command.angular.x = 0
        command.angular.y = 0
        command.angular.z = 0

        r = rospy.Rate(rate)
        for i in range(rate*2):
                Pub.publish(command)        
                r.sleep()
                
def Turn():
        rate=10
        Pub = rospy.Publisher('mobile_base/commands/velocity' , Twist , queue_size=20)
        command = Twist()
        command.linear.x = 0
        command.linear.y = 0
        command.linear.z = 0
        command.angular.x = 0
        command.angular.y = 0
        command.angular.z = 1
        r = rospy.Rate(rate)
        for i in range(rate*2):
                Pub.publish(command)        
                r.sleep()


def doSquare():
        for i in range(4):
                MoveLeg()
                Turn()
                
