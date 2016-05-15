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
def handle_moveRobot(req):
    ack = 0 #Currently executing
    print rospy.get_name(), "Moving: %s forward "%str(req.length)

    pub = rospy.Publisher("RosAria/cmd_vel", Twist, queue_size=10)    


#Sleeping for 0.01s to ensure ready for movement    
    twist = Twist()
    pub.publish(twist)
    rospy.loginfo("Sleeping for 2 sec to make sure it's stopped.")
    rospy.sleep(0.01);


    twist.linear.x = (req.length)
    rospy.loginfo("Moving robot.")
    pub.publish(twist)
    rospy.sleep(1);

    rospy.loginfo("Stopping.")
    twist = Twist()
    pub.publish(twist)
    rospy.sleep(0.01);

    print rospy.get_name(), "Finished Moving"

    ack = 1
    return MoveRobotResponse(ack)


    
def moveRobot_server():
    rospy.init_node('moveRobot_server')
    s=rospy.Service('moveRobot', MoveRobot, handle_moveRobot)
    print rospy.get_name(), "Ready to move"
    rospy.spin()


if __name__ == "__main__":
    moveRobot_server()

