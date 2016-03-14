#!/usr/bin/env python

from robotclient.srv import *
from robotclient.msg import *
import Measure
import rospy
from rospy.numpy_msg import numpy_msg

def handle_get_coord(req):
    pos = Measure.main()
    return GetCoordResponse(pos)

def get_coord_server():
    rospy.init_node('get_coord_server')
    s = rospy.Service('get_coord', GetCoord, handle_get_coord)
    print "Ready to Get Coords!"
    rospy.spin()

if __name__ == "__main__":
    get_coord_server()
