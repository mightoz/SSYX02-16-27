#!/usr/bin/env python

import rospy
from std_msgs.msg import String

from masterclient.srv import *

def talker():
    #pub = rospy.Publisher("iterator", String, queue_size=10)
    rospy.wait_for_service('iterator')
    #rospy.init_node('mainstarup', anonymous = True)
    #rate = rospy.Rate(10)
    #while not rospy.is_shutdown():
    #	pub.publish("align2")
    #	rate.sleep()
    try:
	iterator = rospy.ServiceProxy('iterator', Iterator)
	while(1):
		s = String()
		s.data = "align2"	
		resp1 = iterator(s)
    except rospy.ServiceException, e:
	print "Service could not be called %s" %e


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
	pass
