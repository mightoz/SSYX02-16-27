#!/usr/bin/env python

#import roslib; roslib.load_manifest(PKG)
import rospy

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import Twist

import numpy as np
import math

from robotclient.srv  import *

class MainController():
    

    def __init__(self):
#        rospy.Subscriber("testmove", numpy_msg(Floats), self.moveTo)
#        rospy.spin()
        atGoal = False
        activeRobot = 0
        numberofRobots  = 1
        robot_1_coords = np.array([], dtype=np.float32) #Logged coordinates for robot 1
        robot_2_coords = np.array([], dtype=np.float32) #Logged coordinates for robot 2
    
 
        #Get starting coordinates
	activeRobot = 1
	print "Attempting to get cords for robot1"

        currentpos1 = self.getCoords()
        robot_1_coords.append(self.currentpos1[0], self.currentpos1[1])
	print "Got cords for robot1"
	activeRobot = 2
        #currentpos2 = self.getCoords()
	currentpos2 =  np.array([0,-1], dype=np.float32) 
        robot_2_coords.append(self.currentpos2[0], self.currentpos2[1])

        #Set positions for end nodes
        end_node = np.array([0, -2], dtype=np.float32)
        master_node = np.array([0, 1], dtype=np.float32)

        
        #Direction vector calculation
        v= np.array([], dtype=np.float32)
        v_x = master_node[0] - end_node[0]
        v_y = master_node[1] - end_node[1]
        v.add(v_x, v_y)



        #While distance from either first or robot to perfect line is further away than 10 centimeters, execute the move
        while ((np.absolute(np.cross(v, np.array[(master_node[0] - currentpos1[0]),(master_node[1] - currentpos1[1])]))/np.absolute(v) > 0.1) |
            (np.absolute(np.cross(v, np.array[(master_node[0] - currentpos2[0]), (master_node[1] - currentpos2[1])]))/np.absolute(v) > 0.1) ):
            #Test each robot and see if they're on the correct 'optimal' position, if not move and append the logged positions from moveTo to the coordinates for each robot
            for i in numberofRobots:
                nextPosition = np.array([], dtype=np.float32)
                nextPosition = self.correctPos(i)
                if self.getCoords(i) == nextPosition:
                    pass
                else:
                    activeRobot = i
                    res = self.moveTo(nextPosition)
                    if activeRobot==1:
                        currentpos1=self.getCoords(1)
                        robot_1_coords.append(res)
                    elif activeRobot==2 :
                        currentpos2=self.getCoords(2)
                        robot_2_coords.append(res)



    def correctPos(self, robot):
        #Calculates correct position for robot depending on active robot
        correctPosition = np.array([], dtype=np.float32)
        if (robot==1):
            nextCoordx = (self.end_node[0] + self.currentpos2[0])/2
            nextCoordy =  (self.end_node[1] + self.currentpos2[1])/2
            correctPosition.add(nextCoordx, nextCoordy)
        elif (robot==2):
            nextCoordx = (self.currentpos1[0] + self.master_node[0])/2
            nextCoordy = (self.currentpos1[1] + self.master_node[1])/2
            correctPosition.add(nextCoordx, nextCoordy)
        else:  
            pass
        return correctPosition


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
        srv = '/rotateRobot1'#+self.activeRobot
        rospy.wait_for_service(srv)
        mvRobot = rospy.ServiceProxy(srv, RotateRobot)
        try:
            x = mvRobot(angle)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def driveForward(self):
	print "attempting to move"
        srv = '/moveRobot1'#+self.activeRobot
        rospy.wait_for_service(srv)
        mvRobot = rospy.ServiceProxy(srv, MoveRobot)
        try:
            x = mvRobot(0.2)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))


    def getCoords(self):
	print "Trying to get Cords"
        srv = '/get_coord1'#+self.activeRobot
        rospy.wait_for_service(srv)
        try:
	    get_coords = rospy.ServiceProxy(srv, GetCoord)
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
