#!/usr/bin/env python

# import roslib; roslib.load_manifest(PKG)
import rospy
import numpy as np

from rospy.numpy_msg import numpy_msg
from robotclient.msg import Floats
from geometry_msgs.msg import Twist
# from robotclient.msg import *

import Kalman
import Controls

from robotclient.srv import *


# TODO - neighbors might be better if they were actual objects perhaps
class Node(object):
    def __init__(self, num, node_type):

        if node_type == "Base":
            self.type = "Base"
            self.right_neighbor = num + 1
            self.left_neighbor = -1
        elif node_type == "End":
            self.type = "End"
            self.right_neighbor = -1
            self.left_neighbor = num - 1
        else:
            self.type = "Robot"
            self.left_neighbor = num - 1
            self.right_neighbor = num + 1

        self.node = num
        self.recorded_positions = np.array([], dtype=np.float32)
        self.recorded_x_positions = np.array([], dtype=np.float32)
        self.recorded_y_positions = np.array([], dtype=np.float32)

        self.x = 0  # temp for testing
        self.z = 0  # temp for testing
        self.theta = 0  # temp for testing
        self.pos = np.array([0, 0], dtype=np.float32)  # temp for testing
        self.__axlen = 0.43

        self.kalman = Kalman.Kalman(0.5, 0, 0, 0)
        self.controls = Controls.Controls(0, 0, 0, 0, 0, 2, 2, 0)

    def set_kalman(self, sigma_meas, sigma_x, sigma_z, dt):
        self.kalman.set_sigma_meas(sigma_meas)
        self.kalman.set_sigma_x(sigma_x)
        self.kalman.set_sigma_z(sigma_z)
        self.kalman.set_time_step(dt)

    def set_controls(self, x_min, x_max, z_min, z_max, k, t_x, t_z, ok_dist):
        self.controls.set_x_min(x_min)
        self.controls.set_x_max(x_max)
        self.controls.set_z_min(z_min)
        self.controls.set_z_max(z_max)
        self.controls.set_k(k)
        self.controls.set_t_x(t_x)
        self.controls.set_t_z(t_z)
        self.controls.set_ok_dist(ok_dist)

    def set_x(self, val):
        self.x = val

    def set_z(self, val):
        self.z = val

    def set_theta(self, val):
        self.theta = val

    def set_pos(self, val):
        self.pos = val

    def get_x(self):
        return self.x

    def get_z(self):
        return self.z

    def get_theta(self):
        return self.theta

    def get_axlen(self):
        return self.__axlen

    def get_pos(self):
        return self.pos

    def get_kalman(self):
        return self.kalman

    def get_controls(self):
        return self.controls

    def measure_coordinates(self):
        # Perhaps not empty, returns weirds
        tmp_pos = np.empty([], dtype=np.float32)
        if ((self.type == "Base") or (self.type == "End")):
            tmp_pos = self.pos
        else :
            srv = 'get_coord' + str(self.node)
            rospy.wait_for_service(srv)
            get_coords = rospy.ServiceProxy(srv, GetCoord)
            try:
                f = Floats()
                f = get_coords(1)
                tmp_pos = f.data.data
                self.recorded_positions = np.append(self.recorded_positions, tmp_pos)
                self.recorded_x_positions = np.append(self.recorded_x_positions, tmp_pos[0])
                self.recorded_y_positions = np.append(self.recorded_y_positions, tmp_pos[1])
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
        return tmp_pos

    def get_recorded_positions(self):
        return self.recorded_positions

    def get_recorded_x_positions(self):
        return self.recorded_x_positions

    def get_recorded_y_positions(self):
        return self.recorded_y_positions

    def get_left_neighbor(self):
        return self.left_neighbor

    def get_right_neighbor(self):
        return self.right_neighbor

    def drive_forward(self, length):

        srv = '/moveRobot' + str(self.node)
        rospy.wait_for_service(srv)
        mv_robot = rospy.ServiceProxy(srv, MoveRobot)
        try:
            x = mv_robot(length)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def rotate(self, angle):
        srv = '/rotateRobot' + str(self.node)
        rospy.wait_for_service(srv)
        mv_robot = rospy.ServiceProxy(srv, RotateRobot)
        try:
            x = mv_robot(angle)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def update_twist(self):
        if self.type != "Robot":
            print "Cannot publish twist messages to", str(self.type)
        else:
            srv = '/updateTwist' + str(self.node)
            rospy.wait_for_service(srv)
            update_twist = rospy.ServiceProxy(srv, UpdateTwist)
            try:
		f = Floats()
		f.data = np.array([self.x, self.z], dtype=np.float32) 
                a = update_twist(f)
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
