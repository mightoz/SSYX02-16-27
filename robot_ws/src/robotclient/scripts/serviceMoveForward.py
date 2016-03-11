#!/usr/bin/env python
PKG = 'numpy'
import roslib; roslib.load_manifest(PKG)

import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np



#Function that moves robot given command.
def handle_moveRobot(req):
    int ack = 0 #Currently executing
    print rospy.get_name(), "Moving: %s forward "%str(req.deg)

    pub = rospy.Publisher("RosAria/cmd_vel", Twist, queue_size=10)    

    rospy.loginfo("Sleeping for 2 sec to ensure stopped last motion.")
    rospy.sleep(2);
    
    twist = Twist()
    pub.publish(twist)
    rospy.loginfo("Sleeping for 2 sec to make sure it's stopped.")
    rospy.sleep(2);


    twist.linear.x = (req.length/10)
    rospy.loginfo("Moving robot.")
    pub.publish(twist)
    rospy.sleep(10);

    rospy.loginfo("Stopping.")
    twist = Twist()
    pub.publish(twist)
    rospy.sleep(2);

    print rospy.get_name(), "Finished Moving"

    ack = 1
    return RotationResponse(ack)


    
def moveRobot_server():
    rospy.init_node('moveRobot_server')
    s=rospy.Service('moveRobot', Rotation, handle_moveRobot)
    print rospy.get_name(), "Ready to move"
    rospy.spin()


if __name__ == '__main__':
    serviceRotate()

