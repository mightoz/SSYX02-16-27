#!/usr/bin/env python

# import roslib; roslib.load_manifest(PKG)
import rospy
import numpy as np

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import Twist
# from robotclient.msg import *

from robotclient.srv import *


# TODO - neighbors might be better if they were actual objects perhaps
class Node(object):
    def __init__(self, num, type):

        if type == "Base":
            self.type = "Base"
            self.right_neighbor = num + 1
            self.left_neighbor = -1
        elif type == "End":
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

    def measure_coordinates(self):
        tmp_pos = np.empty([], dtype=np.float32)
        if (self.type == "Robot"):
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
        elif (self.type == "Base"):
            tmp_pos = np.array([0, 3], dtype=np.float32)
        elif (self.type == "End"):
            tmp_pos = np.array([0, -2], dtype=np.float32)
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
