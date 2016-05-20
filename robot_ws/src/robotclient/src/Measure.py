#!/usr/bin/env python
PKG = 'robotclient'

import roslib;
roslib.load_manifest(PKG)
import rospy
from rospy.numpy_msg import numpy_msg
from robotclient.msg import *
import MessageHandler
import Anchor
import numpy as np
import matplotlib.pyplot as plt
import time


class Measure(object):

    def __init__(self, ip):
        self.nbr_of_measurements = 1
        self.tol = [1e-6, 1e-6]
        self.msg_handler = MessageHandler.MessageHandler()
        self.msg_handler.set_ip(ip)
        self.nbr_of_anchors = 3  # same as len(self.anchors)
        self.anchors = []
        for j in range(0, self.nbr_of_anchors):
            self.anchors += [Anchor.Anchor()]
        self.__set_anchor__(0, 106, -3, 2)
        self.__set_anchor__(1, 114, 3, 0)
        self.__set_anchor__(2, 108, -2, -3)

    def set_nbr_of_measurements(self, val):
        self.nbr_of_measurements = np.int(np.abs(val))

    def set_tol(self, abs_tol, rel_tol):
        self.tol[0] = np.abs(abs_tol)
        self.tol[1] = np.abs(rel_tol)

    def get_nbr_of_measurements(self):
        return self.nbr_of_measurements

    def get_tol(self):
        return self.tol

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
        # Params: UWB-transceiver ip and coordinates for each transceiver.
        pos = self.msg_handler.run_loc_rob(self.anchors, self.nbr_of_measurements, self.tol, False)
        print pos
        if pos is None or np.size(pos) != 2:
            pos = np.array([0, 0, -1], dtype=np.float32)
        pos_np = np.array(pos, dtype=np.float32)

        f = Floats()
        f.data = pos_np
        print pos_np

        #return pos_np
        return f

    def __open_sock__(self):
        status = self.msg_handler.connect_req(0)  # connect to the RCM that is connected via ethernet cable
        if status == -1:
            print 'Could not connect to the UWB radio'
            self.__close_sock__()  # close the socket
            return np.array([0, 0, -1], dtype=np.float32)

    def __close_sock__(self):
        self.msg_handler.dc_req(0)  # close the socket

    def __set_anchor__(self, anchor_id, ip, x, y):
        anchor_id = np.abs(np.int(anchor_id))
        if 0 <= anchor_id < self.nbr_of_anchors:
            self.anchors[anchor_id].set_ip(ip)
            self.anchors[anchor_id].set_pos(x, y)

"""
if __name__ == '__main__':
    Measure.talker()
"""
"""
runner = Measure(103)
runner.set_nbr_of_measurements(1)
runner.__open_sock__()
for i in range(0, 10):
    print runner.main()
    time.sleep(1)
runner.__close_sock__()
plt.show()
"""
