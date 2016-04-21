#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher("iterator", String, queue_size=10)
    rospy.init_node('mainstarup', anonymous = True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
	pub.publish("align2")
	rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
	pass
