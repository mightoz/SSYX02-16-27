#!/usr/bin/env python

# import roslib; roslib.load_manifest(PKG)
import rospy
import numpy as np


from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import Twist
#from robotclient.msg import *

from robotclient.srv import *


#TODO - neighbors might be better if they were actual objects perhaps
class Node(object):
    def __init__(self, num, type):

        if type == "Base":
            self.type = "Base"
            self.right_neighbor = num+1
            self.left_neighbor = -1
        elif type == "End":
            self.type = "End"
            self.right_neighbor = -1
            self.left_neighbor = num-1
        else:
            self.type = "Robot"
            self.left_neighbor = num-1
            self.right_neighbor = num+1

        self.node = num
        self.recorded_positions = np.array([], dtype=np.float32)
        self.recorded_x_positions = np.array([], dtype=np.float32)
        self.recorded_y_positions = np.array([], dtype=np.float32)


    def GetCoords(self):
    tmpPos = np.empty([], dtype=np.float32)
    if (self.type == "Robot"):
            srv = 'get_coord' + str(self.node)
            rospy.wait_for_service(srv)
            get_coords = rospy.ServiceProxy(srv, GetCoord)
            try:
                f = Floats()
                f = get_coords(1)
                tmpPos = f.data.data
                self.recorded_positions = np.append(self.recorded_positions, tmpPos)
                self.recorded_x_positions = np.append(self.recorded_x_positions, tmpPos[0])
                self.recorded_y_positions = np.append(self.recorded_y_positions, tmpPos[1])
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
    elif (self.type == "Base"):
        tmpPos = np.array([0,3], dtype=np.float32)
    elif (self.type == "End"):
        tmpPos = np.array([0,-2], dtype=np.float32)
        return tmpPos

    def GetRecordedPositions(self):
        return self.recorded_positions

    def GetRecordedXPositions(self):
        return self.recorded_x_positions

    def GetRecordedYPositions(self):
        return self.recorded_y_positions

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
        srv = '/rotateRobot' + str(self.node)
        rospy.wait_for_service(srv)
        mvRobot = rospy.ServiceProxy(srv, RotateRobot)
        try:
            x = mvRobot(angle)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

