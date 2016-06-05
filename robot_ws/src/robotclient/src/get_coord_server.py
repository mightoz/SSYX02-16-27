#!/usr/bin/env python
PKG = 'robotclient'
import roslib; roslib.load_manifest(PKG)
from robotclient.srv import *
from robotclient.msg import *
import Measure
import rospy
import MessageHandler
import Anchor
from rospy.numpy_msg import numpy_msg


runner = None #Global variable might not be necessary.
def handle_get_coord(req):
    """
    Measures and returns position using Measure.py
    :param req: empty message
    :return:Position of robot
    """
    global runner
    # Create measure instance
    runner = Measure.Measure(rospy.get_param(rospy.get_name()+'/ip_of_uwb'))  # IP of UWB transceiver on ROBOT
    runner.__open_sock__()
    pos = runner.main()  # Measure position
    runner.__close_sock__()
    return GetCoordResponse(pos)

def get_coord_server():
    """
    Initation of service
    :return: none
    """
    rospy.init_node('get_coord_server')
    global runner
    s = rospy.Service('get_coord', GetCoord, handle_get_coord)
    print "Ready to Get Coords!"
    rospy.spin()

if __name__ == "__main__":
    get_coord_server()
