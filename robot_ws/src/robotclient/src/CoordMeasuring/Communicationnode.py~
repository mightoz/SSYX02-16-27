#!/usr/bin/env python
PKG = 'numpy'
import roslib; roslib.load_manifest(PKG)

import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np


lastPos = np.array([0,0], dtype = np.float32);

#Updates lastPos with the last retrieved position from Measure.py
#Send lastPos to the Base

def passPos(data):
    global lastPos
    lastPos = data.data
    print rospy.get_name(), "Current position: %s"%str(lastPos)
    pos = rospy.Publisher("robot1pos", numpy_msg(Floats), queue_size=10)
    pos.publish(data)



#Function that moves robot given command.
def commandMove(cmd):
    pub = rospy.Publisher("RosAria/cmd_vel", Twist, queue_size=10)    

    rospy.loginfo("Sleeping for 2 sec to ensure stopped last motion.")
    rospy.sleep(2);
    
    twist = Twist()
    pub.publish(twist)
    rospy.loginfo("Sleeping for 2 sec to make sure it's stopped.")
    rospy.sleep(2);

    twist = cmd

    rospy.loginfo("Moving robot.")
    pub.publish(twist)
    rospy.sleep(2); #2?


    rospy.loginfo("Stopping.")
    twist = Twist()
    pub.publish(twist)



    
    
#Initializes 2 subscribers, 'floats' and 'moveTo'.
def listener():
    rospy.init_node('listener')
    rospy.Subscriber("floats", numpy_msg(Floats), passPos)
    rospy.Subscriber("command", Twist, commandMove)
    rospy.spin()


if __name__ == '__main__':
    listener()

