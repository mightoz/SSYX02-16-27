#!/usr/bin/env python

from masterclient.srv import *
from masterclient.msg import *

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg

pos = np.array([], dtype=np.float32)

def handle_get_base(req):
    y = Floats(pos)
    return GetCoordResponse(y)

def callback(data):
    print "Reached here"
    global pos
    basepos = data.data
    pos = basepos

def get_coord_server():
    global pos
    pos = np.array([0,3], dtype=np.float32)
    s = rospy.Service('get_coord0', GetCoord, handle_get_base)
    rospy.Subscriber("getBase", Floats, callback)
    print "Ready to Get BaseCords!"
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node('get_coord_server_base')
    get_coord_server()
