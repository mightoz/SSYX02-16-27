#!/usr/bin/env python

import rospy

from rospy.numpy_msg import numpy_msg
from masterclient.msg import Floats
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import Node
import time
import CollisionAvoidance

from robotclient.srv import *
from masterclient.srv import *
from masterclient.msg import *


class MainController():
    def handle_get_base(self, req):

        # f = Floats()
        basepos = req.data
        f = np.array([basepos[0], basepos[1]], dtype=np.float32)
        self.nodes[0].set_pos(f)
        return BaseEndGetCoordResponse(1)

    def handle_get_end(self, req):
        # f = Floats()
        endpos = req.data
        f = np.array([endpos[0], endpos[1]], dtype=np.float32)
        self.nodes[self.nbr_of_nodes - 1].set_pos(f)
        return BaseEndGetCoordResponse(1)

    def __init__(self, nbr_of_nodes):

        # Runtime variables
        self.calls = 0
        self.finishedalign2 = 0
        self.reachedalign2 = 0
        self.exectimestuff = []
        self.alignMethod = -1

        # Instantiate Nodes
        self.nbr_of_nodes = nbr_of_nodes
        self.nodes = []

        for i in range(0, self.nbr_of_nodes):
            if i == 0:
                self.nodes += [Node.Node(i, "Base")]
            elif i == self.nbr_of_nodes - 1:
                self.nodes += [Node.Node(i, "End")]
            else:
                self.nodes += [Node.Node(i, "Robot")]

        # Set inital Base/End position to defualt if
        initend = np.array([0, -2], dtype=np.float32)
        initbase = np.array([0, 3], dtype=np.float32)
        self.nodes[0].set_pos(initbase)
        self.nodes[self.nbr_of_nodes - 1].set_pos(initend)

        # Instantiation for collision avoidance
        safedist = 1
        relsafedist = 0.01
        self.ca = CollisionAvoidance.CollisionAvoidance(safedist, relsafedist)

        # s = rospy.Service('get_coordEnd', BaseEndGetCoord, self.handle_get_end) #Use this service to change End Coords manually
        # s = rospy.Service('get_coordBase', BaseEndGetCoord, self.handle_get_base)  #Use this service to change Base Coords manually
        self.iteratorCalled = time.time()
        self.iteratorFinished = time.time()
        service = rospy.Service('iterator', Iterator,
                                self.align_robots)  # Iterator service called whenever an iteration of the program is wanted
        rospy.spin()
        rospy.on_shutdown(self.terminator)  # On CTRL+C function call.

    def calibrate(self):
        """
        Calibrates robots in area and updates inital position and theta accordingly. AKA Startup Sequence
        """

        # Init for inital orientation and position
        for i in range(0, self.nbr_of_nodes - 1):  # Remove -1 if UWB connected to END
            print "For robot", i
            first_pos = np.empty([], dtype=np.float32)
            second_pos = np.empty([], dtype=np.float32)
            if i != 0 and i != self.nbr_of_nodes - 1:
                # First measurement and movement only required of RobotNodes
                # Measure inital position
                srv = 'get_coord' + str(i)
                rospy.wait_for_service(srv)
                get_coords = rospy.ServiceProxy(srv, GetCoord)
                try:
                    # Retry measuring inital position if recieved position errored
                    errorpos = True
                    while errorpos:
                        f = Floats()
                        f = get_coords(1)
                        first_pos = np.array(f.data.data, dtype=np.float32)
                        if len(first_pos) == 2:  # If len(first_pos) > 2 then positon is faulty
                            errorpos = False
                        else:
                            time.sleep(0.5)  # Sleep to make sure UWB rests
                            print "Failed to get inital position for robot:", i
                except rospy.ServiceException as exc:
                    print("Service did not process request: " + str(exc))
                print "First pos: ", first_pos
                # Movment for robots before second inital measurement
                srv = '/moveRobot' + str(i)
                rospy.wait_for_service(srv)
                mv_robot = rospy.ServiceProxy(srv, MoveRobot)
                try:
                    x = mv_robot(0.2)  # Move forward 20cm
                except rospy.ServiceException as exc:
                    print("Service did not process request: " + str(exc))

            # Measure second position for robots, and Base/End inital position
            srv = 'get_coord' + str(i)
            rospy.wait_for_service(srv)
            get_coords = rospy.ServiceProxy(srv, GetCoord)
            try:
                errorpos = True
                while errorpos:
                    f = Floats()
                    f = get_coords(1)
                    second_pos = np.array(f.data.data, dtype=np.float32)
                    if len(second_pos) == 2:  # If len(first_pos) > 2 then positon is faulty
                        errorpos = False
                    else:
                        time.sleep(0.5)  # Sleep to make sure UWB rests
                        print "Failed to get inital position for robot:", i
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
            print "Second pos:", second_pos

            # If a robot, calculate the orientation in the room
            if i != 0 and i != self.nbr_of_nodes - 1:
                A = second_pos - first_pos
                B = np.array([1, 0], dtype=np.float32)
                if np.linalg.norm(A) > 1e-40:
                    phi = np.arccos(np.dot(A, B) / (np.linalg.norm(A) * np.linalg.norm(B)))
                if second_pos[1] >= first_pos[1]:
                    angle = phi
                else:
                    angle = 2 * np.pi - phi
                print "Initial orientation: ", angle
                # update state
                self.nodes[i].set_state(np.array([[second_pos[0]], [self.nodes[i].get_x_vel()],
                                                  [second_pos[1]], [self.nodes[i].get_y_vel()],
                                                  [angle], [self.nodes[i].get_theta_vel()]]))
            else:
                # update pos
                self.nodes[i].set_pos(second_pos)

    def align_robots(self, data):
        self.alignMethod = int(data.data.data[len(data.data.data) - 1])  # Saves which align method is going to be used
        if self.calls == 0:
            self.calibrate()
        if data.data.data == "align1":
            self.align_robots_1()
            self.calls += 1
        elif data.data.data == "align2":
            self.align_robots_2()
            self.calls += 1

        # Measure coordinate of base (Used only with UWB)
        base_pos = np.array(self.nodes[0].measure_coordinates(), dtype=np.float32)
        if np.size(base_pos) == 2:
            self.nodes[0].set_pos(base_pos)  # Update position of base

        # Measure coordinate of end node (Used only with UWB)
        """
        end_pos = np.array(self.nodes[self.nbr_of_nodes-1].measure_coordinates(), dtype=np.float32)
        if (np.size(end_pos) == 2):
            self.nodes[0].set_pos(end_pos) #Update position of end"""
        return IteratorResponse(1)

    def align_robots_1(self):
        # Loop through all of the robots
        for i in range(1, self.nbr_of_nodes - 1):
            # Calculate correct position for robot
            possible_next_position = self.correct_pos(self.nodes[i])
            self.nodes[i].set_target_positions(possible_next_position)
            # Check if position is within a radius of 0.1m of possible_next_position
            if np.linalg.norm(self.nodes[i].measure_coordinates() - possible_next_position) > 0.1:
                self.move_a_to_b(self.nodes[i], possible_next_position)  # Otherwise move

    def align_robots_2(self):
        self.iteratorFinished = self.iteratorCalled
        self.iteratorCalled = time.time()
        exectime = self.iteratorCalled - self.iteratorFinished
        corr_idx = 1 + np.mod(self.calls, self.nbr_of_nodes - 2)  # Decide which robot is allowed to measure
        for i in range(1, self.nbr_of_nodes - 1):  # Calculate/Estimate new state for all robots
            if i != corr_idx:  # If robot is not allowed to measure,a update its state with kalman.
                new_state2 = self.nodes[i].get_kalman().predict(self.nodes[i].get_state(), self.nodes[i].get_x(),
                                                                self.nodes[i].get_z(),
                                                                exectime)
                self.nodes[i].set_state(new_state2)
                print "Robot %s predicts" % i
            else:
                meas_pos = np.array(self.nodes[i].measure_coordinates(),
                                    dtype=np.float32)  # Measure coordinates of robot
                if np.size(meas_pos) != 2:  # If measurements failed, update state with kalman
                    new_state2 = self.nodes[i].get_kalman().predict(self.nodes[i].get_state(), self.nodes[i].get_x(),
                                                                    self.nodes[i].get_z(),
                                                                    exectime)
                    print "Robot %s predicts since it failed to measure position" % i
                else:
                    print "this was measured position", meas_pos
                    # Update state with correction of measurement + kalman
                    new_state2 = self.nodes[i].get_kalman().correct(self.nodes[i].get_state(), meas_pos,
                                                                    self.nodes[i].get_x(),
                                                                    self.nodes[i].get_z(),
                                                                    exectime)
                self.nodes[i].set_state(new_state2)
                print "Robot %s corrects" % i

        for i in range(1, self.nbr_of_nodes - 1):  # Calculate new controls at time k
            x3 = self.nodes[i].get_controls().calc_controls(self.nodes[i].get_theta(), self.nodes[i].get_pos(),
                                                            self.nodes[self.nodes[i].get_left_neighbor()].get_pos(),
                                                            self.nodes[self.nodes[i].get_right_neighbor()].get_pos())

            # Update controls
            self.nodes[i].set_x(x3[0])
            self.nodes[i].set_z(x3[1])

        for i in range(1, self.nbr_of_nodes - 2):
            for j in range(i + 1, self.nbr_of_nodes - 1):
                x1, z1, x2, z2 = self.ca.calc_new_controls(self.nodes[i].get_pos(), self.nodes[i].get_theta(),
                                                           self.nodes[i].get_x(), self.nodes[i].get_z(),
                                                           self.nodes[j].get_pos(), self.nodes[j].get_theta(),
                                                           self.nodes[j].get_x(), self.nodes[j].get_z(),
                                                           exectime)
                # Update controls after collision avoidance
                self.nodes[i].set_x(x1)
                self.nodes[i].set_z(z1)
                self.nodes[j].set_x(x2)
                self.nodes[j].set_z(z2)
        # Send controls to robots
        for i in range(1, self.nbr_of_nodes - 1):
            self.nodes[i].update_twist()
            # For plotting controls, save in array of controls
            self.nodes[i].append_control_x_history(self.nodes[i].get_x())
            self.nodes[i].append_control_z_history(self.nodes[i].get_z())

        # Updates array containing time instances for control updates (for plotting)
        if self.calls == 0:
            self.exectimestuff += [exectime]
        else:
            self.exectimestuff += [self.exectimestuff[len(self.exectimestuff) - 1] + exectime]

    def plot_on_termn(self, alignnbr):
        """
        Note that this function should only be called when two robots are used.
        Todo: Make generic plot function for n-number of robots.
        :param alignnbr: plot iterative(0) or simultaneous(1)
        :return: none
        """
        if alignnbr == 1:
            plt.figure(1)  # Plot for path
            name = "%s position" % self.nodes[1].get_type()
            name1 = "%s position" % "Target"
            path = "%s path" % self.nodes[1].get_type()
            # For robot #1, x-coordinate, y-coordinate and target(labels included).
            plt.plot(self.nodes[1].get_measured_x_positions(), self.nodes[1].get_measured_y_positions(), 'r',
                     label=path)
            plt.plot(self.nodes[1].get_measured_x_positions(), self.nodes[1].get_measured_y_positions(), "ko",
                     label=name)
            plt.plot(self.nodes[1].get_target_x_positions(), self.nodes[1].get_target_y_positions(), "o", mfc='none',
                     mec='#00bb00', label=name1)

            # For robot #2, x-coordinate, y-coordinate and target (without labels). For-loop this.
            plt.plot(self.nodes[2].get_measured_x_positions(), self.nodes[2].get_measured_y_positions(), 'r')
            plt.plot(self.nodes[2].get_measured_x_positions(), self.nodes[2].get_measured_y_positions(), "ko")
            plt.plot(self.nodes[2].get_target_x_positions(), self.nodes[2].get_target_y_positions(), "o", mfc='none',
                     mec='#00bb00')

            # Plotting final positions. For loop this.
            if (len(self.nodes[1].get_measured_x_positions())) > 0:
                plt.plot(self.nodes[1].get_measured_x_positions()[len(self.nodes[1].get_measured_x_positions()) - 1],
                         self.nodes[1].get_measured_y_positions()[len(self.nodes[1].get_measured_x_positions()) - 1],
                         "yo",
                         label='Final position')
            if (len(self.nodes[2].get_measured_x_positions())) > 0:
                plt.plot(self.nodes[2].get_measured_x_positions()[len(self.nodes[2].get_measured_x_positions()) - 1],
                         self.nodes[2].get_measured_y_positions()[len(self.nodes[2].get_measured_x_positions()) - 1],
                         "yo")
            # Plot base and end positions
            name = "%s position" % self.nodes[0].get_type()
            plt.plot(self.nodes[0].get_measured_x_positions(), self.nodes[0].get_measured_y_positions(), "mo",
                     label=name)
            name = "%s position" % self.nodes[3].get_type()
            plt.plot(self.nodes[3].get_measured_x_positions(), self.nodes[3].get_measured_y_positions(), "co",
                     label=name)

            # Final position line plot
            xs = [self.nodes[0].get_x_pos(), self.nodes[3].get_x_pos()]
            ys = [self.nodes[0].get_y_pos(), self.nodes[3].get_y_pos()]
            plt.plot(xs, ys, 'b', label='Target line')

            # Plot Anchor Positions
            plt.plot(-3, 2, 'x', label="Anchor position", color='r', mew=4, ms=7)  # number 1
            plt.plot(3, 0, 'x', color='r', mew=4, ms=7)  # number 2
            plt.plot(-2, -3, 'x', color='r', mew=5, ms=7)  # number 3
            plt.axis([-5, 5, -5, 6], aspect=1)
            plt.legend(loc='lower right', numpoints=1)

        elif alignnbr == 2:
            plt.figure(1)  # Plot for path.
            name = "%s position" % self.nodes[1].get_type()
            name1 = "%s position" % "Target"
            path = "%s path" % self.nodes[1].get_type()
            # For robot #1, x-coordinate and y-coordinate, labels included.
            plt.plot(self.nodes[1].get_corrected_x_positions(), self.nodes[1].get_corrected_y_positions(), 'r',
                     label=path)
            plt.plot(self.nodes[1].get_corrected_x_positions(), self.nodes[1].get_corrected_y_positions(), "ko",
                     label=name)
            # For robot #2, x-coordinate and y-coordinate, without labels.
            plt.plot(self.nodes[2].get_corrected_x_positions(), self.nodes[2].get_corrected_y_positions(),
                     'r')
            plt.plot(self.nodes[2].get_corrected_x_positions(), self.nodes[2].get_corrected_y_positions(), "ko")
            # Plot target positions
            plt.plot(self.nodes[1].get_target_x_positions_from_controls(),
                     self.nodes[1].get_target_y_positions_from_controls(), "o", mfc='none', mec='#00bb00', label=name1)
            plt.plot(self.nodes[2].get_target_x_positions_from_controls(),
                     self.nodes[2].get_target_y_positions_from_controls(), "o", mfc='none', mec='#00bb00')
            # Plotting final positions
            if (len(self.nodes[1].get_corrected_x_positions())) > 0:
                plt.plot(self.nodes[1].get_corrected_x_positions()[len(self.nodes[1].get_corrected_x_positions()) - 1],
                         self.nodes[1].get_corrected_y_positions()[len(self.nodes[1].get_corrected_x_positions()) - 1],
                         "yo", label='Final position')
            if (len(self.nodes[2].get_corrected_x_positions())) > 0:
                plt.plot(self.nodes[2].get_corrected_x_positions()[len(self.nodes[2].get_corrected_x_positions()) - 1],
                         self.nodes[2].get_corrected_y_positions()[len(self.nodes[2].get_corrected_x_positions()) - 1],
                         "yo")

            # Plot base and end positions
            name = "%s position" % self.nodes[0].get_type()
            plt.plot(self.nodes[0].get_measured_x_positions(), self.nodes[0].get_measured_y_positions(), "mo",
                     label=name)
            name = "%s position" % self.nodes[3].get_type()
            plt.plot(self.nodes[3].get_measured_x_positions(), self.nodes[3].get_measured_y_positions(), "co",
                     label=name)

            # Final position line plot
            xs = [self.nodes[0].get_x_pos(), self.nodes[3].get_x_pos()]
            ys = [self.nodes[0].get_y_pos(), self.nodes[3].get_y_pos()]
            plt.plot(xs, ys, 'b', label='Target line')

            # Plot Anchor positions
            plt.plot(-3, 2, 'x', label="Anchor position", color='r', mew=4, ms=7)  # number 1
            plt.plot(3, 0, 'x', color='r', mew=4, ms=7)  # number 2
            plt.plot(-2, -3, 'x', color='r', mew=5, ms=7)  # number 3
            plt.axis([-5, 5, -5, 6], aspect=1)
            plt.legend(loc='lower right', numpoints=1)

            plt.figure(2)  # Plot for controls
            # Exectimesstuff might not always match dimensions of controls and will not then be plotted.
            # (Fix this by making a new terminating function that makes sure that the iteration is completed before exit
            if len(self.exectimestuff) == len(self.nodes[1].get_control_x_history()):
                plt.plot(self.exectimestuff, self.nodes[1].get_control_x_history(), 'r', label='Velocity')
                plt.plot(self.exectimestuff, self.nodes[1].get_control_x_history(), 'rx')
                plt.plot(self.exectimestuff, self.nodes[1].get_control_z_history(), 'b', label='Angular velocity')
                plt.plot(self.exectimestuff, self.nodes[1].get_control_z_history(), 'bx')
            if len(self.exectimestuff) == len(self.nodes[2].get_control_x_history()):
                plt.plot(self.exectimestuff, self.nodes[2].get_control_x_history(), 'r')
                plt.plot(self.exectimestuff, self.nodes[2].get_control_x_history(), 'rx')
                plt.plot(self.exectimestuff, self.nodes[2].get_control_z_history(), 'b')
                plt.plot(self.exectimestuff, self.nodes[2].get_control_x_history(), 'bx')
            plt.legend()

        plt.show()

    def terminator(self):
        """
        On Ctrl-C terminate
        :return:
        """
        self.plot_on_termn(self.alignMethod)  # Plot on terminate

    def correct_pos(self, robot):
        """
        Find correct_pos for given robot. This is the target function for align 1(Iterative coordination algorithm).
        :param robot:
        :return:
        """
        correct_position = np.array([], dtype=np.float32)
        left = robot.get_left_neighbor()
        right = robot.get_right_neighbor()
        okposleft = False
        okposright = False
        while not okposleft:
            lneigh = np.array(self.nodes[left].measure_coordinates(), dtype=np.float32)
            if np.size(lneigh) == 2:
                okposleft = True

        while not okposright:
            rneigh = np.array(self.nodes[right].measure_coordinates(), dtype=np.float32)
            if np.size(rneigh) == 2:
                okposright = True

        correct_position = (lneigh + rneigh) / 2  # Calculates correct position
        return correct_position

    def move_a_to_b(self, robot, target):
        """
        Move a robot from point A to B. This is the sequential movement algorithm.
        :param robot:
        :param target:
        :return: Recorded positions, used for plotting.
        """
        print "Moving , activerobot: ", robot
        initial_pos = np.array(robot.measure_coordinates(), dtype=np.float32)
        robot.drive_forward(0.2)
        self.run_next_segment(robot, initial_pos, target)

        return robot.get_corrected_positions

    def run_next_segment(self, robot, previous_pos, target_pos):
        """
        Moves a robot in a segment.
        :param robot:
        :param previous_pos:
        :param target_pos:
        :return:
        """

        target = target_pos
        current = np.array(robot.measure_coordinates(), dtype=np.float32)

        # If not close enough to target
        if (not (((np.absolute(target[0] - current[0])) <= 0.1) & (
                    (np.absolute(target[1] - current[1])) <= 0.1))):
            length = math.sqrt(math.pow((target[0] - current[0]), 2) + math.pow((target[1] - current[1]), 2))
            deg = self.calculate_angle(previous_pos, current, target)
            if length <= 0.2:
                robot.rotate(deg)
                robot.drive_forward(length)
            else:
                robot.rotate(deg)
                robot.drive_forward(0.2)
            self.run_next_segment(robot, current, target)
        else:
            return robot

    # Calculates angle that a robot should turn, given target, current and previous pos.
    def calculate_angle(self, previous_pos, current_pos, target_pos):
        previous = previous_pos
        current = current_pos
        target = target_pos
        print "Previous position: ", previous
        print "Current position: ", current
        print "Target: ", target

        direction = 0
        # Calculate if robot should turn right or left
        if np.abs(previous[0] - target[0]) > 1e-40 or np.abs(previous[0] - current[0]) > 1e-40:
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

        dot_prod = (current[0] - previous[0]) * (target[0] - previous[0]) + (current[1] - previous[1]) * (
            target[1] - previous[1])
        lengthA = math.sqrt(math.pow((current[0] - previous[0]), 2) + math.pow((current[1] - previous[1]), 2))
        lengthB = math.sqrt(math.pow((target[0] - previous[0]), 2) + math.pow((target[1] - previous[1]), 2))

        # Calculate degrees to turn
        theta = math.acos(dot_prod / (lengthA * lengthB))

        print rospy.get_name(), "Turningdegree: ", theta
        print rospy.get_name(), "Direction: ", direction

        turning_degree = direction * theta

        return turning_degree


if __name__ == '__main__':
    rospy.init_node('robot_coordinator')
    nb = rospy.get_param(rospy.get_name() + '/nbr')
    try:
        ne = MainController(nb)
    except rospy.ROSInterruptException:
        pass
