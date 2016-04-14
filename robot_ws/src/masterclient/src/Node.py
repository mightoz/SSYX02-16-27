#!/usr/bin/env python
import rospy

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import Twist

import numpy as np

class Node():
    def __init__(self, num):
        if num < 1 :
            self.type = "Base"
        elif num > 2 :
            self.type = "End"
        else :
            self.type = "Robot"

        self.node = num
        self.recordedPositions = np.array([], dtype=np.float32)


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
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        return tmpPos

    def GetRecordedPositions(self):
        return self.recordedPositions

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