#!/usr/bin/env python

# import roslib; roslib.load_manifest(PKG)
import rospy

from rospy.numpy_msg import numpy_msg
#from rospy_tutorials.msg import Floats
from masterclient.msg import Floats
from geometry_msgs.msg import Twist
from std_msgs.msg import String


import numpy as np
import math
import matplotlib.pyplot as plt
import Node

from robotclient.srv import *
from masterclient.srv import *
from masterclient.msg import *

class MainController():

############################################################################################################

    def handle_get_base(self, req):
	#f = Floats()
	self.f = np.array([], dtype=np.float32)  # temp for testing                
	f = req
        basepos = f.data#.data
	self.nodes[0].set_pos(basepos)
	print basepos
        return BaseEndGetCoordResponse(1)

    def handle_get_end(self, req):
	#f = Floats()
	self.f = np.array([], dtype=np.float32)  # temp for testing        
	f = req
        endpos = f.data#.data
	self.nodes[self.nbr_of_nodes-1].set_pos(endpos) 
	print endpos       
        return BaseEndGetCoordResponse(1)

##################################################################################################################

    def __init__(self, nbr_of_nodes):
        rospy.init_node('robot_coordinator')
        rospy.Subscriber("iterator", String, self.align_robots)	
	s = rospy.Service('get_coordEnd', BaseEndGetCoord, self.handle_get_end)
	s = rospy.Service('get_coordBase', BaseEndGetCoord, self.handle_get_base)
    
        # Why is this necessary? it terminates fine as it is.
        # rospy.Subscriber("terminator", None, self.terminate)

        # Some inital values for kalman and controls
        x_max = 1  # Maximum speed forwards
        x_min = 0.05  # Minimm speed forwards
        z_max = 1  # Maximum rotation speed, absolute value
        z_min = 0  # Minimum rotation speed, absolute value
        sigma_x = 0.05  # Standard deviation for speed, percentage
        sigma_z = 0.025  # Standard deviation for rotation, percentage
        sigma_meas = 0.05  # Standard deviation for UWB measurements, NOT percentage
        dt = 0.5  # Timesteps for loop, used in kalmanpredict
        k = 0.25  # Gradient step
        t_x = 2  # Speed factor forward, lower factor = higher speed, !=0
        t_z = 2  # Speed factor rotation, -||-  !=0
        ok_dist = 0.05  # Minimum distance to next targetpos, k affects this

        self.nbr_of_nodes = nbr_of_nodes
        self.nodes = []
        for i in range(0, self.nbr_of_nodes):
            if i == 0:
                self.nodes += [Node.Node(i, "Base")]
            elif i == self.nbr_of_nodes - 1:
                self.nodes += [Node.Node(i, "End")]
            else:
                self.nodes += [Node.Node(i, "Robot")]
                self.nodes[i].set_kalman(sigma_meas, sigma_x, sigma_z, dt)
                self.nodes[i].set_controls(x_min, x_max, z_min, z_max, k, t_x, t_z, ok_dist)

        self.calls = 0  # Increase after every iteration
        rospy.spin()
        rospy.on_shutdown(self.terminator)

    def align_robots(self, data):
        # Add update Base/End position?
        if (data.data == "align1"):
            self.align_robots_1()
        elif (data.data == "align2"):
            self.align_robots_2()
        self.calls += 1

    def align_robots_1(self):
        # Choose number of self.nodes
        """number_of_nodes = 4
        for i in range(0, number_of_nodes):
            self.nodes.append(Node.Node(i))  # create instances of all the Nodes
        else:
            pass"""

        # Set while condition too True.
        not_finished = True
        # while (not_finished):
        # Loop through all of the robots
        for i in range(1, self.nbr_of_nodes - 1):
            # Calulate correctposition for robot
            print i
            possible_next_position = self.correct_pos(self.nodes[i])
            print possible_next_position
            # Check if position is within a radius of 0.1m of possible_next_position
            if (np.linalg.norm(self.nodes[i].measure_coordinates() - possible_next_position) > 0.1):
                print "Tries to move"
                self.move_a_to_b(self.nodes[i], possible_next_position)  # Otherwise move
                # TODO: ONLY CHECK AND THEN MOVE.

    def align_robots_2(self):
        print "mainfunciton"
        corr_idx = np.mod(self.calls, self.nbr_of_nodes - 2)  # Decide which robot should correct its position
        for i in range(1, self.nbr_of_nodes - 1):  # Calculate/Estimate new state
            if i != corr_idx:
                x2, v2 = self.nodes[i].get_kalman().predict(self.nodes[i].get_pos(), self.nodes[i].get_theta(),
                                                            self.nodes[i].get_x(), self.nodes[i].get_z())
                self.nodes[i].set_theta(v2)
                self.nodes[i].set_pos(np.array([x2[0, 0], x2[2, 0]]))
            else:
                # We should have a method call that measures the robot's position here
                # meas_pos = 2*np.random.rand(2)-1
                meas_pos = self.nodes[i].measure_coordinates()
                x2, v2 = self.nodes[i].get_kalman().correct(self.nodes[i].get_pos(), self.nodes[i].get_theta(),
                                                            meas_pos, self.nodes[i].get_x(), self.nodes[i].get_z())
                self.nodes[i].set_theta(v2)
                self.nodes[i].set_pos(np.array([x2[0, 0], x2[2, 0]]))

        for i in range(1, self.nbr_of_nodes - 1):  # Calculate new controls at time k
            x3, v3 = self.nodes[i].get_controls().calc_controls(self.nodes[i].get_theta(), self.nodes[i].get_pos(),
                                                                self.nodes[self.nodes[i].get_left_neighbor()].get_pos(),
                                                                self.nodes[
                                                                    self.nodes[i].get_right_neighbor()].get_pos(),
                                                                self.nodes[i].get_axlen())
            self.nodes[i].set_x(x3)
            self.nodes[i].set_z(v3)
            self.nodes[i].update_twist()

    def terminator(self):
        # For printing, colors hardcoded
        colors = ['gx', 'ro', 'bo', 'gx']
        for i in range(0, self.nbr_of_nodes):
            # print "Recorded positions for Node %s are %s" % (self.nodes[i], self.nodes[i].get_recorded_positions())
            # print "Recorded X positions for Node %s are %s" % (self.nodes[i], self.nodes[i].get_recorded_x_positions())
            # print "Recorded Y positions for Node %s are %s" % (self.nodes[i], self.nodes[i].get_recorded_y_positions())
            plt.plot(self.nodes[i].get_recorded_x_positions(), self.nodes[i].get_recorded_y_positions(), colors[i])
        else:
            pass
        # Hardcoded positions of RCM?
        plt.plot(2, 0, 'kx')
        plt.plot(1, -2, 'kx')
        plt.plot(-1, 1, 'kx')
        plt.axis([-2, 3.5, -3, 3.5])
        plt.show()

   
    ############################################################################################################



    # Find correctposition for robot
    def correct_pos(self, robot):
        correct_position = np.array([], dtype=np.float32)
        left = robot.get_left_neighbor()
        right = robot.get_right_neighbor()
        print left
        print right
        correct_position = (self.nodes[left].measure_coordinates() + self.nodes[
            right].measure_coordinates()) / 2  # Calculates correct position
        return correct_position

    # Move a robot from point A to B
    def move_a_to_b(self, robot, target):
        print "Moving , activerobot: ", robot
        initial_pos = robot.measure_coordinates()
        robot.drive_forward(0.1)
        self.run_next_segment(robot, initial_pos, target)

        return robot.get_recorded_positions

    def run_next_segment(self, robot, previous_pos, target_pos):
        target = target_pos
        current = robot.measure_coordinates()

        if (not (((np.absolute(target[0] - current[0])) <= 0.15) & (
                    (np.absolute(target[1] - current[1])) <= 0.15))):
            length = math.sqrt(math.pow((target[0] - current[0]), 2) + math.pow((target[1] - current[1]), 2))
            deg = self.calculate_angle(previous_pos, current, target)
            if length <= 0.1:
                robot.rotate(deg)
                robot.drive_forward(length)
            else:
                robot.rotate(deg)
                robot.drive_forward(0.1)
            self.run_next_segment(robot, current, target)
        else:
            return robot

    def calculate_angle(self, previous_pos, current_pos, target_pos):
        previous = previous_pos
        current = current_pos
        target = target_pos
        print "Previous position: ", previous
        print "Current position: ", current
        print "Target: ", target

        direction = 0
        # Calculate if robot should turn right or left
        k_targ = (previous[1] - target[1]) / (previous[0] - target[0])  # (yl-yt)/(xl-xt)
        k_move = (previous[1] - current[1]) / (previous[0] - current[0])  # (yc-yl)/(xc-xl)
        if (previous[0] < 0 < previous[1]) or (previous[0] > 0 > previous[1]):
            if k_move >= k_targ:
                direction = -1
            else:
                direction = 1
        else:
            if k_move < k_targ:
                direction = 1
            else:
                direction = -1
        """
        # Calculate degrees to turn
        theta = np.arccos(np.sum((target-fst)*(snd-fst))/(np.sqrt(np.sum((target-fst)**2))*np.sqrt(np.sum((snd-fst)**2))))
        return direction*theta
        """
        dot_prod = (current[0] - previous[0]) * (target[0] - previous[0]) + (current[1] - previous[1]) * (
            target[1] - previous[1])
        lengthA = math.sqrt(math.pow((current[0] - previous[0]), 2) + math.pow((current[1] - previous[1]), 2))
        lengthB = math.sqrt(math.pow((target[0] - previous[0]), 2) + math.pow((target[1] - previous[1]), 2))

        # Calculate degrees to turn
        theta = math.acos(dot_prod / (lengthA * lengthB))

        print rospy.get_name(), "Turningdegree: ", theta
        print rospy.get_name(), "Direction: ", direction

        turning_degree = direction * theta  # (np.pi - math.asin(lengthB*(math.sin(theta)/lengthToTarget)))

        return turning_degree


if __name__ == '__main__':
    # print rospy.get_caller_id()

    try:
        ne = MainController(4)
    except rospy.ROSInterruptException:
        pass
