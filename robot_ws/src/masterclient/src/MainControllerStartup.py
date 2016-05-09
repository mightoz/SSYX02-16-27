#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import time

from masterclient.srv import *

def talker():
    #pub = rospy.Publisher("iterator", String, queue_size=10)
    #rospy.wait_for_service('iterator')
    #rospy.init_node('mainstarup', anonymous = True)
    #rate = rospy.Rate(10)
    #while not rospy.is_shutdown():
    #	pub.publish("align2")
    #	rate.sleep()
    try:
        rospy.init_node('robot_iterator')
        iterator = rospy.ServiceProxy('iterator', Iterator)
        while(1):
            start = time.time()
            s = String()
            s.data = rospy.get_param(rospy.get_name()+'/cordfunc')
            #s.data = "align1"
            resp1 = iterator(s)
            stop = time.time()
            print "Time for iterator:", stop-start
            if s.data == "align2":
                ##TODO ADD PARAM FOR TIME
                if (0.25 - (stop-start)) > 0:
                    time.sleep(0.25-(stop-start))
            print "Total time iterator + delay:", time.time()-start
    except rospy.ServiceException, e:
        print "Service could not be called %s" %e


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
