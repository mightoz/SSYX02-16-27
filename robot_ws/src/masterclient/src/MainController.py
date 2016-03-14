#!/usr/bin/env python

#import roslib; roslib.load_manifest(PKG)
import rospy

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import Twist

import numpy as np
import math


class MainController():

    def __init__(self):


        robot1coords = np.array([], dtype=np.float32);
        atGoal = False
        activeRobot = 0

        rospy.Subscriber("testmove", numpy_msg(Floats), self.moveTo)
        rospy.spin()



    def moveTo(self, coord):

        #ar = self.activeRobot
        recordedPositions = np.array([], dtype=(np.float32, np.float32))
        self.targetPos = coord
        self.firstPos = self.getCoords()  # Temporary initialization
        self.secondPos = self.firstPos  # Temporary initialization

        if (not (np.absolute(self.targetPos[0] - self.secondPos[0]) <= 0.1 & np.absolute(self.targetPos[1] - self.secondPos[1] <= 0.1))):
            self.driveForward()
            self.secondPos = self.getCoords()
            recordedPositions.add(self.secondPos[0],self.secondPos[1])
            if (not (np.absolute(self.targetPos[0] - self.secondPos[0]) <= 0.1 & np.absolute(self.targetPos[1] - self.secondPos[1] <= 0.1))):
                self.runNextSegment()
        else:
            return recordedPositions;



    def runNextSegment(self):
        angle = self.calculateAngle(self.firstPos, self.secondPos, self.targetPos)
        self.rotate(angle)
        self.driveForward()
        self.firstPos = self.secondPos
        self.secondPos = self.getCoords()

        self.recordedPositions.append(self.secondPos[0],self.secondPos[1])
        if (not (np.absolute(self.targetPos[0] - self.secondPos[0]) <= 0.1 & np.absolute(self.targetPos[1] - self.secondPos[1] <= 0.1))):
            self.runNextSegment()
        else:
            return self.recordedPositions

    def rotate(self,angle):
        srv = 'rotateRobot'+self.activeRobot
        rospy.wait_for_service(srv)
        mvRobot = rospy.ServiceProxy(srv, robotclient.srv.RotateRobot)
        try:
            x = mvRobot(angle)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def driveForward(self):
        srv = 'moveRobot'+self.activeRobot
        rospy.wait_for_service(srv)
        mvRobot = rospy.ServiceProxy(srv, robotclient.srv.MoveRobot)
        try:
            x = mvRobot(0.2)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))


    def getCoords(self):
        srv = 'get_coords'+self.activeRobot
        rospy.wait_for_service(srv)
        get_coords = rospy.ServiceProxy(srv, robotclient.srv.GetCoord)
        try:
            pos = get_coords()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        return pos


    #Calculates what angle the robot should turn for next segment
    def calculateAngle(self):
        fst = self.firstPos
        snd = self.secondPos
        target = self.targetPos

        direction = 0
        #Calculate if robot should turn right or left
        k_targ = (fst[1]-target[1])/(fst[0]-target[0])  # (yl-yt)/(xl-xt)
        k_move = (fst[1]-snd[1])/(fst[0]-snd[0])  # (yc-yl)/(xc-xl)
        if (fst[0] < 0 and fst[1] > 0) or (fst[0] > 0 and fst[1] < 0):
            if (k_move >= k_targ):
                direction = 1
            else:
                direction = -1
        else:
            if (k_move < k_targ):
                direction = 1
            else:
                direction = -1
        #Calculate degrees to turn
        dotProd = (snd[0]-fst[0])*(target[0]-fst[0])+(snd[1]-fst[1])*(target[1]-fst[1])

        lengthA = math.sqrt(math.pow((snd[0]-fst[0]),2)+math.pow((snd[1]-fst[1]),2))
        lengthB = math.sqrt(math.pow((target[0]-fst[0]),2)+math.pow((target[1]-fst[1]),2))

        lengthToTarget =  math.sqrt(math.pow((target[0]-snd[0]),2)+math.pow((target[1]-snd[1]),2))

        print rospy.get_name(), "LengthToTarget: %s"%str(lengthToTarget)

        theta = math.acos(dotProd/(lengthA*lengthB))

        print rospy.get_name(), "Theta: %s"%str(theta)

        turningDegree = direction*(np.pi - math.asin(lengthB*(math.sin(theta)/lengthToTarget)))

        return turningDegree



if __name__ == '__main__':
    rospy.init_node('masternode')

    try:
        ne = MainController()
    except rospy.ROSInterruptException: pass
