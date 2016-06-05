#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import time
from masterclient.srv import *

def talker():
    """
    Iterator for Maincontroller.py
    :return:
    """
    # Iteration time step, (recommended minimum = 0.25, have to be lower than t_x, and t_z in Controls.py)
    runtime = 0.25
    try:
        rospy.init_node('robot_iterator')
        iterator = rospy.ServiceProxy('iterator', Iterator)
        while(1):
            start = time.time()
            s = String()
            s.data = rospy.get_param(rospy.get_name()+'/cordfunc')
            resp1 = iterator(s)
            stop = time.time()
            if s.data == "align2":
                if (runtime - (stop-start)) > 0:
                    time.sleep(runtime-(stop-start))
    except rospy.ServiceException, e:
        print "Service could not be called %s" %e


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
