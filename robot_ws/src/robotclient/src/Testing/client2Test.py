#!/usr/bin/env python
PKG = 'numpy'
import roslib; roslib.load_manifest(PKG)

import sys
from robotclient.srv import *

import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np

def test2_client():

    rospy.wait_for_service('rotateRobot')
    try:
        turn = rospy.ServiceProxy('rotateRobot', RotateRobot)
	b = np.float64(2)        
	resp2 = turn(b)
        return resp2.ack
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
	


if __name__ == "__main__":
   test2_client()