#!/usr/bin/env python

#import roslib; roslib.load_manifest(PKG)
import rospy

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import Twist

import numpy as np
import math


class MainNode():

    robot1coords = np.array([], dtype=np.float32);
    atGoal = False
    activeRobot = 1

    def main(self):
        rospy.init_node('masternode')
        rospy.Subscriber("testmove", numpy_msg(Floats), moveTo)
        rospy.spin()

    def moveTo(coord):
        global activeRobot
        recordedPositions = np.array([], dtype=(np.float32, np.float32))
        targetPos = coord
        firstPos = getCoords()  # Temporary initialization
        secondPos = firstPos  # Temporary initialization
        if (not (np.absolute(targetPos[0] - secondPos[0]) <= 0.1 & np.absolute(targetPos[1] - secondPos[1] <= 0.1))):
            driveForward()
            secondPos = getCoords()
            recordedPositions.add(secondPos[0],secondPos[1])
            if (not (np.absolute(targetPos[0] - secondPos[0]) <= 0.1 & np.absolute(targetPos[1] - secondPos[1] <= 0.1))):
                runNextSegment()
        else:
            return recordedPositions;



        def runNextSegment():
            angle = calculateAngle(firstPos, secondPos, targetPos)
            rotate(angle)
            driveForward()
            firstPos = secondPos
            secondPos = getCoords()
            recordedPositions.add(secondPos[0],secondPos[1])
            if (not (np.absolute(targetPos[0] - secondPos[0]) <= 0.1 & np.absolute(targetPos[1] - secondPos[1] <= 0.1))):
                runNextSegment()
            else:
                return recordedPositions

        def rotate(angle):
            srv = 'rotateRobot'+activeRobot
            rospy.wait_for_service(srv)
            mvRobot = rospy.ServiceProxy(srv, robotclient.srv.RotateRobot)
            try:
                x = mvRobot(angle)
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))

        def driveForward():
            srv = 'moveRobot'+activeRobot
            rospy.wait_for_service(srv)
            mvRobot = rospy.ServiceProxy(srv, robotclient.srv.MoveRobot)
            try:
                x = mvRobot(0.2)
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))


        def getCoords():
                srv = 'get_coords'+activeRobot
                rospy.wait_for_service(srv)
                get_coords = rospy.ServiceProxy(srv, robotclient.srv.GetCoord)
                try:
                    pos = get_coords()
                    recordedPositions.add(firstPos[0],firstPos[1])
                except rospy.ServiceException as exc:
                    print("Service did not process request: " + str(exc))
                return pos


    #Calculates what angle the robot should turn for next segment
    def calculateAngle(fst,snd,target):
        firstPos = fst
        secondPos = snd
        targetPos = target

        direction = 0
        #Calculate if robot should turn right or left
        k_targ = (firstPos[1]-targetPos[1])/(firstPos[0]-targetPos[0])  # (yl-yt)/(xl-xt)
        k_move = (firstPos[1]-secondPos[1])/(firstPos[0]-secondPos[0])  # (yc-yl)/(xc-xl)
        if (firstPos[0] < 0 and firstPos[1] > 0) or (firstPos[0] > 0 and firstPos[1] < 0):
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
        dotProd = (secondPos[0]-firstPos[0])*(targetPos[0]-firstPos[0])+(secondPos[1]-firstPos[1])*(targetPos[1]-firstPos[1])

        lengthA = math.sqrt(math.pow((secondPos[0]-firstPos[0]),2)+math.pow((secondPos[1]-firstPos[1]),2))
        lengthB = math.sqrt(math.pow((targetPos[0]-firstPos[0]),2)+math.pow((targetPos[1]-firstPos[1]),2))

        lengthToTarget =  math.sqrt(math.pow((targetPos[0]-secondPos[0]),2)+math.pow((targetPos[1]-secondPos[1]),2))

        print rospy.get_name(), "LengthToTarget: %s"%str(lengthToTarget)

        theta = math.acos(dotProd/(lengthA*lengthB))

        print rospy.get_name(), "Theta: %s"%str(theta)

        turningDegree = direction*(np.pi - math.asin(lengthB*(math.sin(theta)/lengthToTarget)))

        return turningDegree



if __name__ == '__main__':
     main()
