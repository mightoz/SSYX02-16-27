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

rate = 0


# TODO - neighbors might be better if they were actual objects perhaps
class Node(object):
    def __init__(self, num, node_type):
        """

        :param num: Number of nodes including base and end nodes
        :param node_type: Type of node: Base, End, Robot
        :return:
        """
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

        self.kalman = Kalman.Kalman()
        self.controls = Controls.Controls()

        """
        #TODO: PLACE WHERE IT SHOULD BE
        :param sigma_meas: standard deviation of measurement noise. sqrt(2) times the gauss radius
        (where the function has decreased by a factor exp(-1)) of the control
        :param sigma_x: standard deviation of translational noise relative to 1 (m/s for example). sqrt(2) times
        the gauss radius (where the function has decreased by a factor exp(-1)) of the control
        :param sigma_z: standard deviation of rotational noise relative to 1 (rad/s for example). sqrt(2) times
        the gauss radius (where the function has decreased by a factor exp(-1)) of the control
        :return:

        self.kalman.set_sigma_x(sigma_x)
        self.kalman.set_sigma_z(sigma_z)
        self.kalman.set_sigma_meas(sigma_meas)
        """

        """
        #TODO: ADD TO APPROPRIATE PLACE
        :param x_min: minimum velocity [m/s]
        :param x_max: maximum velocity [m/s]
        :param z_min: minimum angular velocity [rad/s]
        :param z_max: maximum angular velocity [rad/s]
        :param k: gradient descent coefficient (0<k<1)
        :param t_x: time to reach target pos if as if the robot was facing its target pos
        :param t_z: time to rotate to face target pos
        :param ok_dist: minimum distance to target pos that will make the robot move
        :return:

        """
    def set_x(self, val):
        """

        :param val: new velocity
        :return:
        """
        self.x = val

    def set_z(self, val):
        """

        :param val:  new angular velocity
        :return:
        """
        self.z = val

    def set_theta(self, val):
        """

        :param val: new orientation
        :return:
        """
        self.theta = val

    def set_pos(self, val):
        """

        :param val: new position
        :return:
        """
        self.pos = val
        #Might not be needed if Base is measured with UWB
        #if (self.type == "Base" or self.type == "End"):
        #    self.recorded_positions = val
        #    self.recorded_x_positions = np.array([val[0]], dtype=np.float32)
        #    self.recorded_y_positions = np.array([val[1]], dtype=np.float32)

    def get_type(self):
        return self.type 

    def get_x(self):
        """

        :return: current velocity
        """
        return self.x

    def get_z(self):
        """

        :return: current angular velocity
        """
        return self.z

    def get_theta(self):
        """

        :return: current orientation
        """
        return self.theta

    def get_pos(self):
        """

        :return: current position
        """
        return self.pos

    def get_kalman(self):
        """

        :return: the instance of Kalman associated with this node
        """
        return self.kalman

    def get_controls(self):
        """

        :return: the instance of Controls associated with this node
        """
        return self.controls

    def measure_coordinates(self):
        """

        :return: measured position if the node
        """
        # Perhaps not empty, returns weirds
        tmp_pos = np.empty([], dtype=np.float32)
        #Comment this out if END has UWB
        if self.type == "End": #self.type == "Base" or
            tmp_pos = self.pos
        else:
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
        """

        :return:
        """
        return self.recorded_positions

    def get_recorded_x_positions(self):
        """

        :return:
        """
        return self.recorded_x_positions

    def get_recorded_y_positions(self):
        """

        :return:
        """
        return self.recorded_y_positions

    def get_left_neighbor(self):
        """

        :return:
        """
        return self.left_neighbor

    def get_right_neighbor(self):
        """

        :return:
        """
        return self.right_neighbor

    def drive_forward(self, length):
        """

        :param length:
        :return:
        """
        srv = '/moveRobot' + str(self.node)
        rospy.wait_for_service(srv)
        mv_robot = rospy.ServiceProxy(srv, MoveRobot)
        try:
            mv_robot(length)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def rotate(self, angle):
        """

        :param angle:
        :return:
        """
        srv = '/rotateRobot' + str(self.node)
        rospy.wait_for_service(srv)
        mv_robot = rospy.ServiceProxy(srv, RotateRobot)
        try:
            mv_robot(angle)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def update_twist(self):
        """

        :return:
        """
        if self.type != "Robot":
            print "Cannot publish twist messages to", str(self.type)
        else:
            srv = '/updateTwist' + str(self.node)
            rospy.wait_for_service(srv)
            update_twist = rospy.ServiceProxy(srv, UpdateTwist)
            try:
                f = Floats()
                f.data = np.array([self.x, self.z], dtype=np.float32)
                update_twist(f)
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
