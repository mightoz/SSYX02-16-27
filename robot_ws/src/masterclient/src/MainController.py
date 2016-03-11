#!/usr/bin/env python

#import roslib; roslib.load_manifest(PKG)
import rospy

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import Twist

import numpy as np
import math

robot1coords = np.array([], dtype = np.float32);
atGoal = False
#robot2coords = np.array([], dtype = np.float32);

#robot1twists = np.array([], dtype = Twist);
 
#robot1twist = np.array(twist); #Array of sent twist messages to robot 1

#Adds last recorded coordinate measurement for robot 1. When 100 measurements have been made, update twist message
def updateCoords1(pos):
    global robot1coords

    print pos.data;
# --pseudocode--
#    robot1coords.add(pos.data)
#    count ++ 
#    if count == 100(
#   	 updaterobot(1)
# 	 count = 0
#	)
#def updateCoords2(pos):
#    global robot2coords
#    robot2coords.add(pos.data)

#Defines next twist message to send to robot based on previous twist message and 
#def updaterobot(robot)
#    robotcoords
#    robottwists
#    if robot == 1:
#	global robot1coords
#        robotcoords = robot1coords
# 	global robot1twists
#	robottwists = robot1twists

# --pseudocode--
#    switch robot
#	case 1:
#		calculate projected point from robot1coords and last sent twist message
#	case 2:
#	case 3:
#	case 4:

def main():
    rospy.init_node('masternode')
    rospy.Subscriber("robot1pos", numpy_msg(Floats), updateCoords1)
    rospy.spin()

def moveTo(coord):
    recordedPositions = np.array([],dtype=(np.float32,np.float32))
    targetPos = np.array([], dtype=np.float32)
    targetPos = coord
    firstPos = getCoords() #Temporary initialization
    secondPos = firstPos #Temporary initialization


    if (not(np.absolute(targetPos[0]-secondPos[0]) <= 0.1 & np.absolute(targetPos[1]-secondPos[1] <= 0.1))):
        driveForward()
        secondPos = getCoords()
        recordedPositions.add(secondPos)
        if (not(np.absolute(targetPos[0]-secondPos[0]) <= 0.1 & np.absolute(targetPos[1]-secondPos[1] <= 0.1))):
            runNextSegment()
        else:
            return recordedPositions
    else:
        return recordedPositions


    def runNextSegment():
        angle = calculateAngle(firstPos,secondPos,targetPos)
        rotate(angle)
        driveForward()
        firstPos = secondPos
        secondPos = getCoords()
        recordedPositions.add(secondPos)
        if (not(np.absolute(targetPos[0]-secondPos[0]) <= 0.1 & np.absolute(targetPos[1]-secondPos[1] <= 0.1))):
            runNextSegment()
        else:
            return recordedPositions


    def rotate(angle):
        rospy.wait_for_service('rotateRobot')
        mvRobot = rospy.ServiceProxy('rotateRobot', robotclient.srv.RotateRobot)
        try:
            x = mvRobot(angle)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def driveForward():
        rospy.wait_for_service('moveRobot')
        mvRobot = rospy.ServiceProxy('moveRobot', robotclient.srv.MoveRobot)
        try:
            x = mvRobot(0.2)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def getCoords():
        rospy.wait_for_service('get_coords')
        get_coords = rospy.ServiceProxy('get_coords', robotclient.srv.GetCoord)
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
