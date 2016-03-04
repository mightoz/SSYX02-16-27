#!/usr/bin/env python
PKG = 'robotclient'

import roslib; roslib.load_manifest(PKG)
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

import ConnectionRequest
import RunLocateRobot
import numpy as np
import matplotlib.pyplot as plt
#Initializes node and creates publisher. 
def talker():
    pub = rospy.Publisher('coordinates', numpy_msg(Floats),queue_size=10)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        a = main()
        pub.publish(a)
	r.sleep()
#Runs scripts to retrieve coordinates for robot.
def main():   
    s, reqIp = ConnectionRequest.connectReq(101, 0)  # connect to the RCM that is connected via ethernet cable

#Params: UWB-transceiver ip and coordinates for each transceiver.
    pos = RunLocateRobot.RunLocRob(s, reqIp, np.array([[106], [114], [109]]),
                               np.array([[-1, 1, 1], [0, 3, -2]]), 4, 1e-6, 1e-6, 0)
    pos = np.array(pos, dtype = np.float32)

    ConnectionRequest.dcReq(s, 0)  # close the socket that was opened above.

    return pos


if __name__ == '__main__':
    talker()
