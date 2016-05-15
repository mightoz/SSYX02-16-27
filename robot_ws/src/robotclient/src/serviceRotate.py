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
    #RosAria/cmd_vel should be changed when starting service with multiple robots
    pub = rospy.Publisher("RosAria/cmd_vel", Twist, queue_size=10)    

#Sleep for 0.01s to ensure ready for rotate       
    twist = Twist()
    pub.publish(twist)
    rospy.loginfo("Sleeping for 2 sec to make sure it's stopped.")
    rospy.sleep(0.01);

#Fixed rotating speed, sleeptime changes depending on deg
    z = 0.5
    twist.angular.z = z*np.sign(req.deg)
    print (np.abs(req.deg/0.5))
    rospy.loginfo("Rotatating robot.")
    pub.publish(twist)
    rospy.sleep(np.abs(req.deg/z));#np.abs(req.deg/0.5));

    rospy.loginfo("Stopping.")
    twist = Twist()
    pub.publish(twist)
    rospy.sleep(0.01);

    print rospy.get_name(), "Finished Rotatating"

    ack = 1
    return RotateRobotResponse(ack)


    
def rotateRobot_server():
    #rotateRobot_service & rotateRobot should be changed when starting with multiple robots
    rospy.init_node('rotateRobot_service')
    s=rospy.Service('rotateRobot', RotateRobot, handle_rotateRobot)
    print rospy.get_name(), "Ready to rotate"
    rospy.spin()


if __name__ == "__main__":
    rotateRobot_server()

