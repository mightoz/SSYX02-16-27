#!/usr/bin/env python

from masterclient.srv import *
from masterclient.msg import *

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg

pos = np.array([], dtype=np.float32)

def handle_get_end(req):
    y = Floats(pos)
    return GetCoordResponse(y)

def callback(data):
    print "Reached here"
    global pos
    endpos = data.data
    pos = endpos

def get_coord_server():
    global pos
    pos = np.array([0,-2], dtype=np.float32)
    s = rospy.Service('get_coord3', GetCoord, handle_get_end)
    rospy.Subscriber("getEnd", Floats, callback)
    print "Ready to Get EndCords!"
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node('get_coord_server_end')
    get_coord_server()
