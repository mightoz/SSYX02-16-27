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
    """
    Moves robot a given control command
    :param data: Message type containing control command for rosaria
    :return: Acknowledgement Msg
    """
    inpt = np.array(data.data.data, dtype=np.float32)  # The control is wrapped in Floats() and wrapped in ROS messages
    x = inpt[0]  # Linear Velocity
    z = inpt[1]  # Angular Velocity

    # Update twist
    twist = Twist()
    twist.linear.x = x
    twist.angular.z = z

    # Publisher to rosaria node
    pub = rospy.Publisher("RosAria/cmd_vel", Twist, queue_size=10)
    pub.publish(twist)
    ack = 1
    return UpdateTwistResponse(ack)

def update_twist_server():
    """
    Initation of service
    :return: none
    """
    rospy.init_node('update_twist_server_')
    s = rospy.Service('updateTwist', UpdateTwist, handle_update_twist)
    print rospy.get_name(), "Ready to update twist"
    rospy.spin()


if __name__ == "__main__":
    update_twist_server()
