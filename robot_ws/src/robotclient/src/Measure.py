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
        """

        :param ip: Ip-address of RCM conected to computer.
        :return:
        """
        self.nbr_of_measurements = 1
        self.tol = [1e-6, 1e-6]  # [abs_tol, rel_tol]
        self.msg_handler = MessageHandler.MessageHandler()
        self.msg_handler.set_ip(ip)
        self.nbr_of_anchors = 3  # same as len(self.anchors)
        self.anchors = []
        for j in range(0, self.nbr_of_anchors):
            self.anchors += [Anchor.Anchor()]
        """
        These are the instatioations of the Anchors (see Anchors.py)
        Change these whenever anchors are moved/replaced.
        """
        self.__set_anchor__(0, 106, -1.0, -2.0)
        self.__set_anchor__(1, 114, -1.0, 2.0)
        self.__set_anchor__(2, 108, 2.0, 0.0)

    def set_nbr_of_measurements(self, val):
        self.nbr_of_measurements = np.int(np.abs(val))

    def set_tol(self, abs_tol, rel_tol):
        """
        
        :param abs_tol: Absolute tolerance used when calculating
        position using measured distances. This is the absolute
        error.
        :param rel_tol: Relative tolerance used when calculating
        position using measured distanced. This is the difference
        in the residual between two iterations.
        """
        self.tol[0] = np.abs(abs_tol)
        self.tol[1] = np.abs(rel_tol)

    def get_nbr_of_measurements(self):
        return self.nbr_of_measurements

    def get_tol(self):
        return self.tol

    def get_anchor(self, anchor_id):
        return self.anchors[anchor_id]

    def main(self):
        """
        Runs scripts to retrieve coordinates for robot.

        :return: Position of the robot wrapped in a Floats().
        """
        pos = self.msg_handler.run_loc_rob(self.anchors,
                                           self.nbr_of_measurements,
                                           self.tol, False)
        if pos is None or np.size(pos) != 2:
            pos = np.array([0, 0, -1], dtype=np.float32)
        pos_np = np.array(pos, dtype=np.float32)

        f = Floats()
        f.data = pos_np
        return f

    def __open_sock__(self):
        """
        connect to the RCM that is connected via ethernet cable
        """
        status = self.msg_handler.connect_req(0)
        if status == -1:
            print 'Could not connect to the UWB radio'
            self.__close_sock__()  # close the socket
            return np.array([0, 0, -1], dtype=np.float32)

    def __close_sock__(self):
        """
        close the socket
        """
        self.msg_handler.dc_req(0)

    def __set_anchor__(self, anchor_id, ip, x, y):
        anchor_id = np.abs(np.int(anchor_id))
        if 0 <= anchor_id < self.nbr_of_anchors:
            self.anchors[anchor_id].set_ip(ip)
            self.anchors[anchor_id].set_pos(x, y)
