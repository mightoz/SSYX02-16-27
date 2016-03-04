#!/usr/bin/env python
PKG = 'numpy'
import roslib; roslib.load_manifest(PKG)

import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String

#Simple publisher that does not do anything by itself. Only initialized so that custom position messages can be published on the its topic. Example usage in terminal: 'rostopic pub /moveTo rospy_tutorials/Floats [0,0]' Moves robot to position [0,0].
def coordListener():
     rospy.init_node('positionHandler', anonymous=True)
     pub = rospy.Publisher("moveTo", numpy_msg(Floats), queue_size=10)
     r = rospy.Rate(10)
     while not rospy.is_shutdown():
           r.sleep()

if __name__ == '__main__':
        coordListener()
