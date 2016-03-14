#!/usr/bin/env python
PKG = 'robotclient'
import roslib; roslib.load_manifest(PKG)

from robotclient.srv import *

import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np



#Function that moves robot given command.
def handle_rotateRobot(req):
    ack = 0 #Currently executing
    print rospy.get_name(), "Rotatating: %s"%str(req.deg)

    pub = rospy.Publisher("RosAria/cmd_vel", Twist, queue_size=10)    

    rospy.loginfo("Sleeping for 2 sec to ensure stopped last motion.")
    rospy.sleep(2);
    
    twist = Twist()
    pub.publish(twist)
    rospy.loginfo("Sleeping for 2 sec to make sure it's stopped.")
    rospy.sleep(2);


    twist.angular.z = (req.deg/2)
    rospy.loginfo("Rotatating robot.")
    pub.publish(twist)
    rospy.sleep(2);

    rospy.loginfo("Stopping.")
    twist = Twist()
    pub.publish(twist)
    rospy.sleep(2);

    print rospy.get_name(), "Finished Rotatating"

    ack = 1
    return RotateRobotResponse(ack)


    
def rotateRobot_server():
    rospy.init_node('rotateRobot_server1')
    s=rospy.Service('rotateRobot1', RotateRobot, handle_rotateRobot)
    print rospy.get_name(), "Ready to rotate"
    rospy.spin()


if __name__ == "__main__":
    rotateRobot_server()

