#!/usr/bin/env python

from robotclient.srv import *
from robotclient.msg import *
import Measure
import rospy
import MessageHandler
import Anchor
from rospy.numpy_msg import numpy_msg


runner = None
def handle_get_coord(req):
    global runner
    pos = runner.main()
    return GetCoordResponse(pos)

def get_coord_server():
    rospy.init_node('get_coord_server')
    global runner
    runner = Measure.Measure(rospy.get_param(rospy.get_name()+'/ip_of_uwb'))  # IP of UWB tranciver on ROBOT
    runner.__open_sock__()
    s = rospy.Service('get_coord', GetCoord, handle_get_coord)
    print "Ready to Get Coords!"
    rospy.spin()

if __name__ == "__main__":
    get_coord_server()
