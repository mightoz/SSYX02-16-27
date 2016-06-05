#!/usr/bin/env python


import rospy
import numpy as np

from rospy.numpy_msg import numpy_msg
from robotclient.msg import Floats
from geometry_msgs.msg import Twist

import Kalman
import Controls

from robotclient.srv import *


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
        # All positions measured with UWB-radios.
        self.measured_positions = np.array([], dtype=np.float32)
        self.measured_x_positions = np.array([], dtype=np.float32)
        self.measured_y_positions = np.array([], dtype=np.float32)

        # All positions corrected with kalman + measurements.
        self.corrected_positions = np.array([], dtype=np.float32)
        self.corrected_x_positions = np.array([], dtype=np.float32)
        self.corrected_y_positions = np.array([], dtype=np.float32)

        # Target positions storage, for align1.0 the node knows the target positions
        self.target_positions = np.array([], dtype=np.float32)
        self.target_x_positions = np.array([], dtype=np.float32)
        self.target_y_positions = np.array([], dtype=np.float32)

        # Controls storage
        self.contplot_x = np.array([], dtype=np.float64)
        self.contplot_z = np.array([], dtype=np.float64)

        self.x = 0  # velocity
        self.z = 0  # angular velocity
        self.state = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [
            0.0]])  # state = x_position, x_velocity, y_position, y_velocity, theta, theta_velocity(rotation velocity)

        self.kalman = Kalman.Kalman()
        self.controls = Controls.Controls()

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

    def set_state(self, state):
        if self.type == "Robot":
            self.state = state
        else:  # If base or end, only update position. Rest of params should be immutable.
            self.state[0, 0] = state[0, 0]
            self.state[2, 0] = state[2, 0]

        self.corrected_positions = np.append(self.corrected_positions, self.get_pos())
        self.corrected_x_positions = np.append(self.corrected_x_positions, self.get_x_pos())
        self.corrected_y_positions = np.append(self.corrected_y_positions, self.get_y_pos())

    def set_pos(self, pos):
        """
        Changes position of node
        :param pos: position
        :return:
        """
        pos64 = np.array(pos, dtype=np.float64)
        self.state[0, 0] = pos64[0]
        self.state[2, 0] = pos64[1]
        # Log position of end node (Robot and base are logged when they change state).
        if self.type == "End":
            self.measured_positions = np.append(self.measured_positions, self.get_pos())
            self.measured_x_positions = np.append(self.measured_x_positions, self.get_x_pos())
            self.measured_y_positions = np.append(self.measured_y_positions, self.get_y_pos())

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

    def get_state(self):
        """

        :return: state
        """
        return self.state

    def get_x_pos(self):
        """

        :return: x_pos
        """
        return self.state[0, 0]

    def get_x_vel(self):
        """

        :return: x_vel
        """
        return self.state[1, 0]

    def get_y_pos(self):
        """

        :return: y_pos
        """
        return self.state[2, 0]

    def get_y_vel(self):
        """

        :return: y_vel
        """
        return self.state[3, 0]

    def get_theta(self):
        """

        :return: theta
        """
        return self.state[4, 0]

    def get_theta_vel(self):
        """

        :return: theta_vel
        """
        return self.state[5, 0]

    def get_pos(self):
        """

        :return: pos
        """
        return np.array([self.get_x_pos(), self.get_y_pos()])

    def get_type(self):
        """

        :return: type
        """
        return self.type

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

        :return: if not end node, measures position with UWB-radios and returns it. If end, returns current position.
        """

        tmp_pos = np.array([],
                           dtype=np.float32)
        if self.type == "End":
            tmp_pos = self.get_pos()
        else:
            srv = 'get_coord' + str(self.node)
            rospy.wait_for_service(srv)
            get_coords = rospy.ServiceProxy(srv, GetCoord)
            try:
                f = Floats()
                f = get_coords(1)
                tmp_pos = f.data.data
                # If measurement was successful, add it to list of all measurements
                if len(tmp_pos) == 2:
                    self.measured_positions = np.append(self.measured_positions, tmp_pos)
                    self.measured_x_positions = np.append(self.measured_x_positions, tmp_pos[0])
                    self.measured_y_positions = np.append(self.measured_y_positions, tmp_pos[1])
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
        return tmp_pos

    def get_measured_positions(self):
        """

        :return: measured positions
        """
        return self.measured_positions

    def get_measured_x_positions(self):
        """

        :return: measured x positions
        """
        return self.measured_x_positions

    def get_measured_y_positions(self):
        """

        :return: measured y positions
        """
        return self.measured_y_positions

    def get_corrected_positions(self):
        """

        :return: corrected positions
        """
        return self.corrected_positions

    def get_corrected_x_positions(self):
        """

        :return: corrected x positions
        """
        return self.corrected_x_positions

    def get_corrected_y_positions(self):
        """

        :return: corrected y positions
        """
        return self.corrected_y_positions

    # For align1.0 the node knows the target positions
    def get_target_positions(self):
        """

        :return: corrected positions
        """
        return self.target_positions

    def get_target_x_positions(self):
        """

        :return: corrected x positions
        """
        return self.target_x_positions

    def get_target_y_positions(self):
        """

        :return: corrected y positions
        """
        return self.target_y_positions

    def set_target_positions(self, target):
        """

        :return: corrected positions
        """

        self.target_positions = np.append(self.target_positions, target)
        self.target_x_positions = np.append(self.target_x_positions, target[0])
        self.target_y_positions = np.append(self.target_y_positions, target[1])
        return

    # For align2 information on target is located in controls
    def get_target_x_positions_from_controls(self):
        """

        :return: corrected x positions
        """
        return self.controls.get_target_x_positions()

    def get_target_y_positions_from_controls(self):
        """

        :return: corrected y positions
        """
        return self.controls.get_target_y_positions()

    def get_control_x_history(self):
        """

        :return: corrected x positions
        """
        return self.contplot_x

    def get_control_z_history(self):
        """

        :return: corrected y positions
        """
        return self.contplot_z

    def append_control_z_history(self, cont):
        """

        :return: corrected positions
        """

        self.contplot_z = np.append(self.contplot_z, cont)
        return

    def append_control_x_history(self, cont):
        """

        :return: corrected positions
        """

        self.contplot_x = np.append(self.contplot_x, cont)
        return

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
