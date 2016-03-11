#!/usr/bin/env python

import sys
import rospy
from robotclient.srv import *

def get_coord_client():
    rospy.wait_for_service('get_coord')
    try:
        get_coord = rospy.ServiceProxy('get_coord', GetCoord)
        resp1 = get_coord(1)
        return resp1.data
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    print "Requesting pos"
    print 'Robot pos: ', get_coord_client()
