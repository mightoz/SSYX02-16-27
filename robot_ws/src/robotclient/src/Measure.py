#!/usr/bin/env python
PKG = 'robotclient'
"""
import roslib;

roslib.load_manifest(PKG)
import rospy
from rospy.numpy_msg import numpy_msg
from robotclient.msg import *
"""
import ConnectionRequest
import Anchor
import RunLocateRobot
import numpy as np
import matplotlib.pyplot as plt


class Measure(object):

    def __init__(self, nbr_of_anchors):
        self.nbr_of_measurements = 1
        self.tol = [1e-6, 1e-6]
        self.connect_request = ConnectionRequest.ConnectionRequest()
        self.nbr_of_anchors = nbr_of_anchors
        self.anchors = []
        for i in range(0, self.nbr_of_anchors):
            self.anchors += [Anchor.Anchor()]

    def set_nbr_of_measurements(self, val):
        self.nbr_of_measurements = np.int(np.abs(val))

    def set_tol(self, abs_tol, rel_tol):
        self.tol[0] = np.abs(abs_tol)
        self.tol[1] = np.abs(rel_tol)

    def set_ip(self, val):
        self.connect_request.set__rcm_ip(val)

    def set_anchor(self, anchor_id, ip, x, y):
        anchor_id = np.abs(np.int(anchor_id))
        if 0 <= anchor_id < self.nbr_of_anchors:
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

    """
    # Initializes node and creates publisher.
    def talker(self):
        pub = rospy.Publisher('coordinates', numpy_msg(Floats), queue_size=10)
        rospy.init_node('talker', anonymous=True)
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            a = self.main()
            pub.publish(a)
            r.sleep()
    """
    # Runs scripts to retrieve coordinates for robot.
    def main(self):
        status = self.connect_request.connect_req(0)  # connect to the RCM that is connected via ethernet cable
        if status == -1:
            print 'Could not connect to the UWB radio'
            self.connect_request.dc_req(0)  # close the socket
            return
        # Params: UWB-transceiver ip and coordinates for each transceiver.
        pos = RunLocateRobot.run_loc_rob(self.connect_request.get_socket(),
                                         self.connect_request.get_rcm_ip(),
                                         self.anchors, self.nbr_of_measurements,
                                         self.tol, 0)
        if pos is None:
            pos = np.array([0, 0, -1])
        if np.size(pos) != 2:
            pos = np.array([0, 0, -1])
        pos_np = np.array(pos, dtype=np.float32)

        #f = Floats()
        #f.data = pos_np

        self.connect_request.dc_req(0)  # close th socket
        return pos_np
        #return f

"""
if __name__ == '__main__':
    Measure.talker()
"""
runner = Measure(3)
runner.set_ip(101)
runner.set_anchor(0, 106, -1, 1)
runner.set_anchor(1, 114, 2, 0)
runner.set_anchor(2, 108, 1, -2)
runner.set_nbr_of_measurements(1)
print runner.main()
plt.show()
