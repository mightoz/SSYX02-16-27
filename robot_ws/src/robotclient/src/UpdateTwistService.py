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

def handle_update_twist(data):
    ack = 0
    inpt = np.array(data.data.data, dtype=np.float32)
    x = inpt[0]
    z = inpt[1]

    twist = Twist()

    twist.linear.x = x
    twist.angular.z = z

    pub = rospy.Publisher("RosAria/cmd_vel", Twist, queue_size=10)
    pub.publish(twist)
    ack = 1
    return UpdateTwistResponse(ack)

def update_twist_server():
    rospy.init_node('update_twist_server_')
    s = rospy.Service('updateTwist', UpdateTwist, handle_update_twist)
    print rospy.get_name(), "Ready to update twist"
    rospy.spin()


if __name__ == "__main__":
    update_twist_server()
