#!/usr/bin/env python
PKG = 'robotclient'

import roslib;

roslib.load_manifest(PKG)
import rospy
from rospy.numpy_msg import numpy_msg
from robotclient.msg import *

import ConnectionRequest
import Anchor
import RunLocateRobot
import numpy as np
import matplotlib.pyplot as plt


class Measure(object):

    def __init__(self):
        self.nbr_of_measurements = 1
        self.tol = [1e-6, 1e-6]
        self.connect_request = ConnectionRequest.ConnectionRequest()
        self.anchors = [Anchor.Anchor(), Anchor.Anchor(), Anchor.Anchor()]

    def set_nbr_of_measurements(self, val):
        self.nbr_of_measurements = np.int(np.abs(val))

    def set_tol(self, abs_tol, rel_tol):
        self.tol[0] = np.abs(abs_tol)
        self.tol[1] = np.abs(rel_tol)

    def set_ip(self, val):
        if val[0:10] == '192.168.1.':
            self.connect_request.set__rcm_ip(val)
        else:
            print 'Invalid IP'

    def set_anchor(self, anchor_id, ip, x, y):
        self.anchors[anchor_id].set_ip(ip)
        self.anchors[anchor_id].set_pos(x, y)

    def get_nbr_of_measurements(self):
        return self.nbr_of_measurements

    def get_tol(self):
        return self.tol

    def get_ip(self):
        return self.connect_request.get_rcm_ip()

    def get_anchor(self, anchor_id):
        return self.anchors[anchor_id]

    # Initializes node and creates publisher.
    def talker(self):
        pub = rospy.Publisher('coordinates', numpy_msg(Floats), queue_size=10)
        rospy.init_node('talker', anonymous=True)
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            a = self.main()
            pub.publish(a)
            r.sleep()

    # Runs scripts to retrieve coordinates for robot.
    def main(self):
        self.connect_request.connect_req(0)  # connect to the RCM that is connected via ethernet cable

        # Params: UWB-transceiver ip and coordinates for each transceiver.
        pos = RunLocateRobot.run_loc_rob(self.connect_request.get_socket(),
                                         self.connect_request.get_rcm_ip(),
                                         self.anchors, self.nbr_of_measurements,
                                         self.tol, 0)
        pos_np = np.array(pos, dtype=np.float32)

        f = Floats()
        f.data = pos_np

        self.connect_request.dc_req(0)  # close the socket that was opened above.

        return f


if __name__ == '__main__':
    Measure.talker()
