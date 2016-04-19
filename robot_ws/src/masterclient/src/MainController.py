#!/usr/bin/env python

# import roslib; roslib.load_manifest(PKG)
import rospy

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import Twist

import numpy as np
import math
import matplotlib.pyplot as plt
import Node

from robotclient.srv import *


class MainController():
    def __init__(self):
        # alignrobot1.0

        # Choose number of self.nodes
        numberofNodes = 4
        self.nodes = []  # Array containing pointers to instances of all the Nodes
        for i in range(0, numberofNodes):
            self.nodes.append(Node.Node(i))  # create instances of all the Nodes
        else:
            pass

        # Set while condition too True.
        notFinished = True
        while (notFinished):
            # Loop through all of the robots
            for i in range(1, numberofNodes - 1):
                # nextPosition = np.array([], dtype=np.float32)
                # Calulate correctposition for robot

                possiblenextPosition = self.CorrectPos2(self.nodes[i])
                print possiblenextPosition
                # Check if position is within a radius of 0.1m of possiblenextPosition
                if (np.linalg.norm(self.nodes[i].GetCoords() - possiblenextPosition) > 0.1):
                    print "Tries to move"
                    self.MoveAToB2(self.nodes[i], possiblenextPosition)  # Otherwise move
                    # TODO: ONLY CHECK AND THEN MOVE.
            else:
                pass
                # For printing
        colors = ['gx', 'ro', 'bo', 'gx']
        for i in (0, numberofNodes):
            print "Recorded positions for Node %s are %s" % (self.nodes[i], self.nodes[i].GetRecordedPositions())
            print "Recorded X positions for Node %s are %s" % (self.nodes[i], self.nodes[i].getRecordedXPositions())
            print "Recorded Y positions for Node %s are %s" % (self.nodes[i], self.nodes[i].getRecordedYPositions())
            plt.plot(self.nodes[i].getRecordedXPositions(), self.nodes[i].getRecordedYPositions(), colors[i])
        else:
            pass
        plt.plot(2, 0, 'kx')
        plt.plot(1, -2, 'kx')
        plt.plot(-1, 1, 'kx')
        plt.axis([-2, 3.5, -3, 3.5])
        plt.show()

    # Find correctposition for robot
    def correct_pos_2(self, robot):
        correctPosition = np.array([], dtype=np.float32)
        left = robot.GetLeftNeighbor()
        right = robot.GetRightNeighbor()
        correctPosition = (self.nodes[left].GetCoords() + self.nodes[
            right].GetCoords()) / 2  # Calculates correct position
        return correctPosition

    # Move a robot from point A to B
    def MoveAToB2(self, robot, target):
        print "Moving , activerobot: ", robot
        initialPos = robot.GetCoords()
        robot.DriveForward(0.1)
        self.RunNextSegment2(robot, initialPos, target)

        return robot.GetRecordedPositions

    def RunNextSegment2(self, robot, previousPos, targetPos):
        target = targetPos
        current = robot.GetCoords()

        if (not (((np.absolute(target[0] - current[0])) <= 0.15) & (
                    (np.absolute(target[1] - current[1])) <= 0.15))):
            length = math.sqrt(math.pow((target[0] - current[0]), 2) + math.pow((target[1] - current[1]), 2))
            deg = self.calculateAngle2(previousPos, current, target)
            if length <= 0.1:
                robot.Rotate(deg)
                robot.DriveForward(length)
            else:
                robot.Rotate(deg)
                robot.DriveForward(0.1)
            self.RunNextSegment2(robot, current, target)
        else:
            return robot

    def calculateAngle2(self, previousPos, currentPos, targetPos):
        previous = previousPos
        current = currentPos
        target = targetPos
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
        dotProd = (current[0] - previous[0]) * (target[0] - previous[0]) + (current[1] - previous[1]) * (
        target[1] - previous[1])
        lengthA = math.sqrt(math.pow((current[0] - previous[0]), 2) + math.pow((current[1] - previous[1]), 2))
        lengthB = math.sqrt(math.pow((target[0] - previous[0]), 2) + math.pow((target[1] - previous[1]), 2))

        # Calculate degrees to turn
        theta = math.acos(dotProd / (lengthA * lengthB))

        print rospy.get_name(), "Turningdegree: ", theta
        print rospy.get_name(), "Direction: ", direction

        turningDegree = direction * theta  # (np.pi - math.asin(lengthB*(math.sin(theta)/lengthToTarget)))

        return turningDegree


if __name__ == '__main__':
    rospy.init_node('masternode')
    # print rospy.get_caller_id()

    try:
        ne = MainController()
    except rospy.ROSInterruptException:
        pass
