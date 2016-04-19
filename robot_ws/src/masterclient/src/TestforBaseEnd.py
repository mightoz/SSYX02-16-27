#!/usr/bin/env python

# import roslib; roslib.load_manifest(PKG)
import rospy

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import Twist

import numpy as np
import math
import matplotlib.pyplot as plt
import Node

from robotclient.srv import *
from masterclient.srv import *


def test():
    tmpPos = np.empty([], dtype=np.float32)	
    rospy.wait_for_service('get_coord0')
    basepos = rospy.ServiceProxy('get_coord0', GetCoord)
    f = Floats()
    f = basepos()
    tmpPos = f.data.data
    print "Base station is", tmpPos
    rospy.wait_for_service('get_coord3')
    endpos = rospy.ServiceProxy('get_coord3', GetCoord)
    f = Floats()
    f = endpos()
    tmpPos = f.data.data 
    print "End station is", tmpPos



if __name__ == '__main__':
    rospy.init_node('testnode')
    test()
