#!/usr/bin/env python
import rospy

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import Twist

import numpy as np

class Node():
    def __init__(self, num):
        if num == 0 :
            self.type = "Base"
            self.LeftNeighbor = -1
            self.RightNeighbor = 1
        elif num == 1 :
            self.type = "Robot"
            self.LeftNeighbor = 0
            self.RightNeighbor = 2
        elif num == 2 :
            self.type = "Robot"
            self.LeftNeighbor = 1
            self.RightNeighbor = 3
        elif num == 3 :
            self.type = "End"
            self.LeftNeighbor = 2
            self.RightNeighbor = -1

        self.node = num
        self.recordedPositions = np.array([], dtype=np.float32)
        self.recordedXPositions = np.array([], dtype=np.float32)
        self.recordedYPositions = np.array([], dtype=np.float32)


    def GetCoords(self):
        tmpPos = np.empty([], dtype=np.float32)
        srv = 'get_coord' + str(self.node)
        rospy.wait_for_service(srv)
        get_coords = rospy.ServiceProxy(srv, GetCoord)
        try:
            f = Floats()
            f = get_coords()
            tmpPos = f.data.data
            self.recordedPositions = np.array(self.recordedPositions, tmpPos)
            self.recordedXPositions = np.array(self.recordedXPositions, tmpPos[0])
            self.recordedYPositions = np.array(self.recordedYPositions, tmpPos[1])
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        return tmpPos

    def GetRecordedPositions(self):
        return self.recordedPositions

    def GetRecordedXPositions(self):
        return self.recordedXPositions

    def GetRecordedYPositions(self):
        return self.recordedYPositions

    def GetLeftNeighbor(self):
        return self.LeftNeighbor

    def GetRightNeighbor(self):
        return self.RightNeighbor


    def DriveForward(self, length):

        srv = '/moveRobot' + str(self.node)
        rospy.wait_for_service(srv)
        mvRobot = rospy.ServiceProxy(srv, MoveRobot)
        try:
            x = mvRobot(length)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def Rotate(self, angle):
        srv = '/rotateRobot' + str(self.robot)
        rospy.wait_for_service(srv)
        mvRobot = rospy.ServiceProxy(srv, RotateRobot)
        try:
            x = mvRobot(angle)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

