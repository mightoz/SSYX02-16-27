#!/usr/bin/env python

# import roslib; roslib.load_manifest(PKG)
import rospy

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import Twist

import numpy as np
import math
import matplotlib.pyplot as plt

from robotclient.srv import *


class MainController():
 def __init__(self):
        #alignrobot1.0

        #Choose number of nodes
        self.numberofNodes = 3
        nodes = []  #Array containing pointers to instances of all the Nodes
        for i in (0, numberofNodes):
            nodes.append(Node(i)) #create instances of all the Nodes
        else:
            pass
        #Doublecheck if size is correct    
        print "Check if size is correct" , (len(nodes) == (numberofNodes+1))


        #Set while condition too True.
        notFinished = True
        while (notFinished):
            #Loop through all of the robots                
            for i in (1, numberofNodes-1):
                #nextPosition = np.array([], dtype=np.float32)
                #Calulate correctposition for robot
                possiblenextPosition = self.CorrectPos2(nodes[i])
                #Check if position is within a radius of 0.1m of possiblenextPosition
                if (np.linalg.norm(nodes[i].GetCoords() - possiblenextPosition) > 0.1):
                    self.MoveAToB2(nodes[i], possiblenextPosition) #Otherwise move
                    #TODO: ONLY CHECK AND THEN MOVE.
            else:
                pass 
        #For printing
        colors = ['gx','ro','bo','gx']
        for i in (0, numberofNodes):
            print "Recorded positions for Node %s are %s" % (nodes[i], nodes[i].GetRecordedPositions())
            print "Recorded X positions for Node %s are %s" % (nodes[i], nodes[i].getRecordedXPositions())
            print "Recorded Y positions for Node %s are %s" % (nodes[i], nodes[i].getRecordedYPositions())
            plt.plot(nodes[i].getRecordedXPositions(), nodes[i].getRecordedYPositions(), colors[i])
        else:
            pass
        plt.plot(2, 0, 'kx')
        plt.plot(1, -2, 'kx')
        plt.plot(-1, 1, 'kx')
        plt.axis([-2, 3.5, -3, 3.5])
        plt.show()

    #Find correctposition for robot
    def CorrectPos2(self, robot):
        correctPosition = np.array([], dtype=np.float32)
        left = robot.GetLeftNeighbor()
        right = robot.GetRightNeighbor()
        correctPosition = (left.GetCoords() + right.GetCoords())/2 #Calculates correct position
        return correctPosition



    #Move a robot from point A to B
    def MoveAToB2(robot, target):
        print "Moving , activerobot: ", robot
        initialPos = robot.GetCoords()
        robot.moveForward(0.1)
        RunNextSegment2(robot, initialPos, target)

        return robot.GetRecordedPositions


    def RunNextSegment2(robot, previousPos, targetPos):
        target = targetPos
        current = robot.GetCoords()        

        if (not (((np.absolute(target[0] - current[0])) <= 0.15) & (
                    (np.absolute(target[1] - current[1])) <= 0.15))):
            length = math.sqrt(math.pow((target[0] - current[0]), 2) + math.pow((target[1] - current[1]), 2))
            deg = robot.calculateAngle(previousPos, current, target)
            if length <= 0.1:
                robot.rotate(deg)
                robot.moveForward(length)
            else:
                robot.rotate(deg)
                robot.moveForward(0.1)
            RunNextSegment2(robot, current, target)
        else:
            return robot

    def calculateAngle2(previousPos, currentPos, targetPos):
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
        dotProd = (current[0] - previous[0]) * (target[0] - previous[0]) + (current[1] - previous[1]) * (target[1] - previous[1])
        lengthA = math.sqrt(math.pow((current[0] - previous[0]), 2) + math.pow((current[1] - previous[1]), 2))
        lengthB = math.sqrt(math.pow((target[0] - previous[0]), 2) + math.pow((target[1] - previous[1]), 2))

        
        # Calculate degrees to turn
        theta = math.acos(dotProd / (lengthA * lengthB))

        print rospy.get_name(), "Turningdegree: ", theta
        print rospy.get_name(), "Direction: " , direction 

        turningDegree = direction * theta  # (np.pi - math.asin(lengthB*(math.sin(theta)/lengthToTarget)))

        return turningDegree

def get_rot_dir(self, theta, startpos, endpos):
     """

     :param theta: angle
     :param startpos: [xstart,ystart]
     :param endpos: [xend,yend]
     :return:
     """
     # calculate the angle between the x-axis and the line intersecting endpos and startpos
     phi = 0
     if np.abs(startpos[0]-endpos[0]) > 1e-30:
         z1 = startpos-endpos
         z2 = np.array([1, 0])
         if startpos[1] >= endpos[1]:  # 0 <= phi <= pi
             phi = np.arccos(np.dot(z1, z2)/(np.linalg.norm(z1)*np.linalg.norm(z2)))
         else:  # pi < phi < 2*pi
             phi = 2*np.pi - np.arccos(np.dot(z1, z2)/(np.linalg.norm(z1)*np.linalg.norm(z2)))
     else:
         phi = np.sign(startpos[1]-endpos[1])*np.pi/2

     # calculate the effective angle needed to determine how to change Z
     angle = np.mod(theta - np.pi, 2*np.pi)

     # determine the rotation direction (+1 ccw, -1 cw)
     if 0 <= phi <= np.pi:
         if phi <= angle < (phi + np.pi):
             return -1
         else:
             return 1
     else:
         if np.mod(phi+np.pi, 2*np.pi) < angle < phi:
             return 1
         else:
             return -1
#################################################################################################################################################
#    def __init__(self):
#        self.allCoords = np.array([],dtype=(np.float32,2))
#        self.activeRobot = 0
#        self.numberofRobots = 2
#        initCoordListener()
#        self.lengthToTarget = 1
#        robot_1_coords = np.array([], dtype=(np.float32, 2))  # Logged coordinates for robot 1
#        self.robot_1_xcoords = np.array([], dtype=np.float32)  # Logged x-coordinates for robot 1
#        self.robot_1_ycoords = np.array([], dtype=np.float32)  # Logged y-coordinates for robot 1
#        robot_2_coords = np.array([], dtype=(np.float32, 2))  # Logged coordinates for robot 2
#        self.robot_2_xcoords = np.array([], dtype=np.float32)  # Logged x-coordinates for robot 2
#        self.robot_2_ycoords = np.array([], dtype=np.float32)  # Logged y-coordinates for robot 2
#
#        # Get starting coordinates
#        activeRobot = 1
#
#        self.currentpos1 = np.array([], dtype=np.float32)
#        print "Trying to get coords for robot #1."
#        self.currentpos1 = self.getCoords(1)
#        print "currentpos1: ", (self.currentpos1)
#        robot_1_coords = np.append(robot_1_coords, [(self.currentpos1[0], self.currentpos1[1])])
#        self.robot_1_xcoords = np.append(self.robot_1_xcoords, [self.currentpos1[0]])
#        self.robot_1_ycoords = np.append(self.robot_1_ycoords, [self.currentpos1[1]])
#
#        self.currentpos2 = np.array([], dtype=np.float32)
#        self.currentpos2 = self.getCoords(2)
#        print "currentpos2: ", (self.currentpos2)
#        robot_2_coords = np.append(robot_2_coords, ([self.currentpos2[0]], [self.currentpos2[1]]))
#
#        # Set positions for end nodes
#        self.end_node = np.array([0, -2], dtype=np.float32)
#        self.master_node = np.array([0, 3], dtype=np.float32)
#
#        # Direction vector calculation
#        v_x = (self.master_node[0] - self.end_node[0])
#        v_y = (self.master_node[1] - self.end_node[1])
#        v = np.array([v_x, v_y], dtype=np.float32)
#
#        # While distance from either first or robot to perfect line is further away than 10 centimeters, execute the move
#        while ((np.absolute(np.cross(v, np.array([(self.master_node[0] - self.currentpos1[0]),
#                                                  (self.master_node[1] - self.currentpos1[1])]))) / np.absolute(
#                v[0] + 1j * v[1]) > 0.1) |
#                   (np.absolute(np.cross(v, np.array([(self.master_node[0] - self.currentpos2[0]),
#                                                      (self.master_node[1] - self.currentpos2[1])]))) / np.absolute(
#                           v[0] + 1j * v[1]) > 0.1)):
#            # Test each robot and see if they're on the correct 'optimal' position, if not move and append the logged positions from moveTo to the coordinates for each robot
#            print "Robot 1 not in place: ", (np.absolute(np.cross(v, np.array(
#                [(self.master_node[0] - self.currentpos1[0]),
#                 (self.master_node[1] - self.currentpos1[1])]))) / np.absolute(v[0] + 1j * v[1]) > 0.1)
#            print "Robot 2 not in place: ", (np.absolute(np.cross(v, np.array(
#                [(self.master_node[0] - self.currentpos2[0]),
#                 (self.master_node[1] - self.currentpos2[1])]))) / np.absolute(v[0] + 1j * v[1]) > 0.1)
#            for i in (1, numberofRobots):
#                nextPosition = np.array([], dtype=np.float32)
#                nextPosition = self.correctPos(i)
#                print "Calculated correct pos for robot #", i
#                print "Correct pos: ", nextPosition
#                print "Abovementionend robot close enough to correct pos: ", (
#                    np.sum(np.abs(self.getCoords(i) - nextPosition)) <= 0.1)
#                if (np.sum(np.abs(self.getCoords(i) - nextPosition)) <= 0.1):  # TODO-
#                    pass
#                else:
#                    self.activeRobot = i
#                    robotPositions = self.moveTo(nextPosition)
#                    if self.activeRobot == 1:
#                        self.currentpos1 = self.getCoords(1)  # TODO
#                    elif self.activeRobot == 2:
#                        self.currentpos2 = self.getCoords(2)  # TODO
#            else:
#                pass
#
#
#    def correctPos(self, robot):
#        # Calculates correct position for robot depending on active robot
#        correctPosition = np.array([], dtype=np.float32)
#        if (robot == 1):
#            nextCoordx = (self.master_node[0] + self.currentpos2[0]) / 2
#            nextCoordy = (self.master_node[1] + self.currentpos2[1]) / 2
#            correctPosition = np.append(correctPosition, ([nextCoordx], [nextCoordy]))
#        elif (robot == 2):
#            nextCoordx = (self.currentpos1[0] + self.end_node[0]) / 2
#            nextCoordy = (self.currentpos1[1] + self.end_node[1]) / 2
#            correctPosition = np.append(correctPosition, ([nextCoordx], [nextCoordy]))
#        else:
#            pass
#        return correctPosition
#
#    # Moves active robot to specified position
#    def moveTo(self, coord):
#
#        print "Entered moveTo, activerobot: ", self.activeRobot
#        self.recordedPositions = np.array([], dtype=(np.float32, np.float32))  # All recorded positions
#        self.recordedxPositions = np.array([], dtype=np.float32)
#        self.recordedyPositions = np.array([], dtype=np.float32)
#        self.targetPos = coord
#        self.firstPos = self.getCoords(self.activeRobot)
#        self.secondPos = self.firstPos
#
#        print "Targetpos: ", self.targetPos
#        print "Secondpos: ", self.secondPos
#
#        # Check if robot is already close enough to target position, else run first segment.
#        if (not (((np.absolute(self.targetPos[0] - self.secondPos[0])) <= 0.15) & (
#                    (np.absolute(self.targetPos[1] - self.secondPos[1])) <= 0.15))):
#            self.driveForward()
#            self.secondPos = self.getCoords(self.activeRobot)
#            self.recordedxPositions = np.append(self.recordedxPositions, [self.secondPos[0]])
#            self.recordedyPositions = np.append(self.recordedyPositions, [self.secondPos[1]])
#            print "This is the position in x for moveTo:", self.recordedxPositions
#            print "This is the position in y for moveTo:", self.recordedyPositions
#            print "And return:", (self.recordedxPositions, self.recordedyPositions)
#            self.recordedPositions = np.append(self.recordedPositions, (
#                [self.secondPos[0]], [self.secondPos[1]]))  # Add position after movement to recorded positions
#            print "This is recorded pos:", self.recordedPositions
#            if (not (((np.absolute(self.targetPos[0] - self.secondPos[0])) <= 0.15) & (
#                        (np.absolute(self.targetPos[1] - self.secondPos[1])) <= 0.15))):
#                self.runNextSegment()
#        else:
#            pass
#        return self.recordedPositions
#
#    # Reruns the next segment of total path until robot is close enough to target position.
#    def runNextSegment(self):
#        angle = self.calculateAngle()
#        self.rotate(angle)
#        self.driveForward()
#        self.firstPos = self.secondPos
#        self.secondPos = self.getCoords(self.activeRobot)
#
#        self.recordedPositions = np.append(self.recordedPositions, ([self.secondPos[0]], [self.secondPos[1]]))
#        self.recordedxPositions = np.append(self.recordedxPositions, [self.secondPos[0]])
#        self.recordedyPositions = np.append(self.recordedyPositions, [self.secondPos[1]])
#        print "This is the position in x for nextSegment:", self.recordedxPositions
#        print "This is the position in y for nextSegment:", self.recordedyPositions
#
#        if (not (((np.absolute(self.targetPos[0] - self.secondPos[0])) <= 0.15) & (
#                    (np.absolute(self.targetPos[1] - self.secondPos[1])) <= 0.15))):
#            self.runNextSegment()
#        else:
#            if self.activeRobot == 1:
#                self.robot_1_xcoords = np.append(self.robot_1_xcoords, self.recordedxPositions)
#                self.robot_1_ycoords = np.append(self.robot_1_ycoords, self.recordedyPositions)
#            elif self.activeRobot == 2:
#                self.robot_2_xcoords = np.append(self.robot_2_xcoords, self.recordedxPositions)
#                self.robot_2_ycoords = np.append(self.robot_2_ycoords, self.recordedyPositions)
#            else:
#                pass
#            return None
#
#    def rotate(self, angle):
#        srv = '/rotateRobot' + str(self.activeRobot)
#        rospy.wait_for_service(srv)
#        mvRobot = rospy.ServiceProxy(srv, RotateRobot)
#        try:
#            x = mvRobot(angle)
#        except rospy.ServiceException as exc:
#            print("Service did not process request: " + str(exc))
#
#    def driveForward(self):
#        srv = '/moveRobot' + str(self.activeRobot)
#        if self.lengthToTarget <= 0.2:
#            length = self.lengthToTarget
#        else:
#            length = 0.2
#        rospy.wait_for_service(srv)
#        mvRobot = rospy.ServiceProxy(srv, MoveRobot)
#        try:
#            x = mvRobot(length)
#        except rospy.ServiceException as exc:
#            print("Service did not process request: " + str(exc))
#
#    def getCoords(self, robotNbr):
#        tmpPos = np.empty([], dtype=np.float32)
#        srv = 'get_coord' + str(robotNbr)
#        rospy.wait_for_service(srv)
#        get_coords = rospy.ServiceProxy(srv, GetCoord)
#        try:
#            f = Floats()
#            f = get_coords()
#            tmpPos = f.data.data
#        except rospy.ServiceException as exc:
#            print("Service did not process request: " + str(exc))
#        return tmpPos
#
#    # Calculates what angle the robot should turn for next segment
#    def calculateAngle(self):
#        fst = self.firstPos
#        snd = self.secondPos
#        target = self.targetPos
#        print "First position: ", fst
#        print "Second position: ", snd
#        print "Current target: ", target
#
#        direction = 0
#        # Calculate if robot should turn right or left
#        k_targ = (fst[1] - target[1]) / (fst[0] - target[0])  # (yl-yt)/(xl-xt)
#        k_move = (fst[1] - snd[1]) / (fst[0] - snd[0])  # (yc-yl)/(xc-xl)
#        if (fst[0] < 0 < fst[1]) or (fst[0] > 0 > fst[1]):
#            if k_move >= k_targ:
#                direction = -1
#            else:
#                direction = 1
#        else:
#            if k_move < k_targ:
#                direction = 1
#            else:
#                direction = -1
#        """
#        # Calculate degrees to turn
#        theta = np.arccos(np.sum((target-fst)*(snd-fst))/(np.sqrt(np.sum((target-fst)**2))*np.sqrt(np.sum((snd-fst)**2))))
#        return direction*theta
#        """
#        # Calculate degrees to turn
#        dotProd = (snd[0] - fst[0]) * (target[0] - fst[0]) + (snd[1] - fst[1]) * (target[1] - fst[1])
#
#        lengthA = math.sqrt(math.pow((snd[0] - fst[0]), 2) + math.pow((snd[1] - fst[1]), 2))
#        lengthB = math.sqrt(math.pow((target[0] - fst[0]), 2) + math.pow((target[1] - fst[1]), 2))
#
#        self.lengthToTarget = math.sqrt(math.pow((target[0] - snd[0]), 2) + math.pow((target[1] - snd[1]), 2))
#
#        print rospy.get_name(), "LengthToTarget: %s" % str(self.lengthToTarget)
#
#        theta = math.acos(dotProd / (lengthA * lengthB))
#
#        print rospy.get_name(), "Turningdegree: %s" % str(theta)
#        print rospy.get_name(), "Direction: %s" % str(direction)
#
#        turningDegree = direction * theta  # (np.pi - math.asin(lengthB*(math.sin(theta)/lengthToTarget)))
#
#        return turningDegree
#

if __name__ == '__main__':
    rospy.init_node('masternode')

    try:
        ne = MainController()
    except rospy.ROSInterruptException:
        pass
