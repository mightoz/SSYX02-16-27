#!/usr/bin/env python
PKG = 'numpy'
import roslib; roslib.load_manifest(PKG)

import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np
import math
lastPos = np.array([0,0], dtype = np.float32);

#Updates lastPos with the last retrieved position from Measure.py
def callback(data):
    global lastPos
    lastPos = data.data
#Function that moves robot to pos.
def moveTo(pos):

    pub = rospy.Publisher("RosAria/cmd_vel", Twist, queue_size=10)
    
    
    rospy.loginfo("Sleeping for 2 sec.")
    rospy.sleep(2);
    

    global lastPos
    firstPos = lastPos
    targetPos = pos.data
    print rospy.get_name(), "First position: %s"%str(firstPos)
    print rospy.get_name(), "Target position: %s"%str(targetPos)

    
    twist = Twist()
    pub.publish(twist)
    
    rospy.loginfo("Sleeping for 2 sec.")
    rospy.sleep(2);

    twist.linear.x = 0.1

    rospy.loginfo("Moving the robot forward 50 cm.")
    pub.publish(twist)
    rospy.sleep(5);

    twist = Twist()
    pub.publish(twist)
    rospy.loginfo("Calculating next target, sleeping.")
    

    secondPos = lastPos
    
    dotProd = (secondPos[0]-firstPos[0])*(targetPos[0]-firstPos[0])+(secondPos[1]-firstPos[1])*(targetPos[1]-firstPos[1])

    lengthA = 0.5
    lengthB = math.sqrt(math.pow((targetPos[0]-firstPos[0]),2)+math.pow((targetPos[1]-firstPos[1]),2))

    lengthToTarget =  math.sqrt(math.pow((targetPos[0]-secondPos[0]),2)+math.pow((targetPos[1]-secondPos[1]),2))

    print rospy.get_name(), "LengthToTarget: %s"%str(lengthToTarget)

    theta = math.acos(dotProd/(lengthA*lengthB))  
    
    print rospy.get_name(), "Theta: %s"%str(theta)

    turningDegree = (np.pi - math.asin(lengthB*(math.sin(theta)/lengthToTarget)))

    print rospy.get_name(), "Second position: %s"%str(secondPos)
    rospy.sleep(2);
    print rospy.get_name(), "Rotating robot: %s"%str(turningDegree)
    twist = Twist()
    twist.angular.z = (turningDegree/2)
    pub.publish(twist)
    rospy.sleep(2);
    
    rospy.loginfo("Stopping.")
    twist = Twist()
    pub.publish(twist)

    twist.linear.x = (lengthToTarget/10)
    print rospy.get_name(), "Moving forward: %s"%str(lengthToTarget)
    pub.publish(twist)
    rospy.sleep(10);

    rospy.loginfo("Stopping.")
    twist = Twist()
    pub.publish(twist)
    
    
#Initializes 2 subscribers, 'floats' and 'moveTo'.
def listener():
    rospy.init_node('listener')
    rospy.Subscriber("floats", numpy_msg(Floats), callback)
    rospy.Subscriber("moveTo", numpy_msg(Floats), moveTo)
    rospy.spin()


if __name__ == '__main__':
    listener()


