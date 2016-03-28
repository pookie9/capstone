
#In another terminal run  roslaunch kobuki_node minimal.launch --screen

import rospy;
from geometry_msgs.msg import Twist;
import time;
import math;
from kobuki_msgs.msg import BumperEvent;
import os;

def bumperCallback(data):
        global bumperState
        global lastBumper

        if data.state==1:
                #os.system('say "ouch"')
                bumperState=data.bumper
                lastBumper=data.bumper
                time.sleep(1)
                bumerState=0
                lastbumper=-1


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
#Turns in place the specified angle                
def Turn():
        rate=10
        Pub = rospy.Publisher('mobile_base/commands/velocity' , Twist , queue_size=20)
        command = Twist()
        command.angular.z = 1
        r = rospy.Rate(rate)
        for i in range(rate*2):
                Pub.publish(command)        
                r.sleep()

def spin():
        #Spins in a circle once
        rate=10
        Pub = rospy.Publisher('mobile_base/commands/velocity' , Twist , queue_size=20)        
        #Turning
        command = Twist()
        command.angular.z = 1
        r = rospy.Rate(rate)
        ANGLECONST=12.4
        angle=math.pi*2
        numSteps=int(angle*ANGLECONST)
        print "Numsteps: "+str(numSteps)
        for i in range(numSteps):
                Pub.publish(command)        
                r.sleep()

def moveTo(curPos,desiredPos,scale):
        #Turning to point to the desired point
        turnAngle=math.atan2(desiredPos[1]-curPos[1],desiredPos[0]-curPos[0])
        print "Initial turn angle:"+str(turnAngle)
        turnAngle*=-1
        print "Angle to it: "+str(turnAngle)
        turnAngle=turnAngle-curPos[2]
        if turnAngle<0:
                if turnAngle>-math.pi:
                        turnDirection=-1
                else:
                        turnDirection=1
                        turnAngle=math.pi*2+turnAngle
        elif turnAngle>math.pi:
                turnAngle-=math.pi*2
                turnDirection=-1
        else:
                turnDirection=1
        print "Turning: "+str(turnAngle)
        rate=10
        Pub = rospy.Publisher('mobile_base/commands/velocity' , Twist , queue_size=20)
        
        #Turning
        command = Twist()
        command.angular.z = turnDirection
        r = rospy.Rate(rate)
        ANGLECONST=16
        numSteps=int(abs(turnAngle*ANGLECONST))
        print "Numsteps: "+str(numSteps)
        for i in range(numSteps):
                Pub.publish(command)        
                r.sleep()
        
        #moving
        distance=math.sqrt((curPos[0]-desiredPos[0])**2+(curPos[1]-desiredPos[1])**2)
        print "MyPos: "+str(curPos)
        print "Desired pos: "+str(desiredPos)
        print "Scale: "+str(scale)
        print "distance: "+str(distance)
        distance=distance/scale
        print "distance in m: "+str(distance)
        command = Twist()
        command.linear.x = .2
        r = rospy.Rate(rate)
        LINEARCONST=60
        numSteps=int(LINEARCONST*distance)
        print "numLinearSteps: "+str(numSteps)
        global bumperState
        for i in range(numSteps+1):
                if bumperState!=-1:
                        bumperState=-1
                        return
                Pub.publish(command)        
                r.sleep()

bump=rospy.Subscriber('/mobile_base/events/bumper',BumperEvent,bumperCallback)
lastBumper=-1
bumperState=-1
