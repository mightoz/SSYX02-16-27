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
        self.activeRobot = 0
        numberofRobots  = 1
        robot_1_coords = np.array([], dtype=np.float32) #Logged coordinates for robot 1
        robot_2_coords = np.array([], dtype=np.float32) #Logged coordinates for robot 2
    
 
        #Get starting coordinates
	activeRobot = 1

	
	self.currentpos1 = np.array([], dtype=np.float32) # Temporary initialization HARDKODAT
	self.currentpos1 = self.getCoords(1) 	
	print "This is the position:",(self.currentpos1) 
        #robot_1_coords = np.append(robot_1_coords, ([self.currentpos1[0]], [self.currentpos1[1]]))

	#activeRobot = 2 	#TODO FIXA
        #currentpos2 = self.getCoords(2)
	self.currentpos2 =  np.array([0,-1], dtype=np.float32)  # Temporary initialization HARDKODAT
        robot_2_coords= np.append(robot_2_coords, ([self.currentpos2[0]], [self.currentpos2[1]]))
	

	#activeRobot = 1

        #Set positions for end nodes
        self.end_node = np.array([0,-2], dtype=np.float32)
        self.master_node = np.array([0,1], dtype=np.float32)
	
        
        #Direction vector calculation
        v_x = (self.master_node[0] - self.end_node[0])
        v_y = (self.master_node[1] - self.end_node[1])
        v= np.array([v_x, v_y], dtype=np.float32)


        #While distance from either first or robot to perfect line is further away than 10 centimeters, execute the move
        while ((np.absolute(np.cross(v, np.array([(self.master_node[0] - self.currentpos1[0]),(self.master_node[1] - self.currentpos1[1])])))/np.absolute(v[0] + 1j*v[1]) > 0.1) |
          (np.absolute(np.cross(v, np.array([(self.master_node[0] - self.currentpos2[0]), (self.master_node[1] - self.currentpos2[1])])))/np.absolute(v[0] + 1j*v[1]) > 0.1)):
            #Test each robot and see if they're on the correct 'optimal' position, if not move and append the logged positions from moveTo to the coordinates for each robot
            for i in (1, numberofRobots):
                nextPosition = np.array([], dtype=np.float32)
                nextPosition = self.correctPos(i)
                if (np.sum(np.abs(self.getCoords(i) - nextPosition)) <= 0.1):#TODO- Hur ska vi losa att vi maste skicka med vilken aktiv robot? # Temporary initialization HARDKODAT
                    pass
                else:
                    activeRobot = i
                    res = self.moveTo(nextPosition)
                    if activeRobot==1:
                        currentpos1=self.getCoords(1) #TODO TROR APPEND MASTE HA LIKA STORA :(((
                        robot_1_coords =  np.append(robot_1_coords, (res))
                    elif activeRobot==2 :
                        currentpos2=self.getCoords(2) #TODO
                        robot_2_cords = np.append(robot_2_coords, (res))



    def correctPos(self, robot):
        #Calculates correct position for robot depending on active robot
        correctPosition = np.array([], dtype=np.float32)
        if (robot==1):
            nextCoordx = (self.master_node[0] + self.currentpos2[0])/2
            nextCoordy =  (self.master_node[1] + self.currentpos2[1])/2
            correctPosition = np.append(correctPosition , ([nextCoordx], [nextCoordy]))
        elif (robot==2):
            nextCoordx = (self.currentpos1[0] + self.end_node[0])/2
            nextCoordy = (self.currentpos1[1] + self.end_node[1])/2
            correctPosition = np.append(correctPosition , ([nextCoordx], [nextCoordy]))
        else:  
            pass
        return correctPosition

    #Moves active robot to specified position
    def moveTo(self, coord):

        #ar = self.activeRobot
	
        self.recordedPositions = np.array([], dtype=(np.float32, np.float32)) #All recorded positions
        self.targetPos = coord
        #self.firstPos = np.array([2,-1], dtype=np.float32) 
	self.firstPos = self.getCoords(self.activeRobot)  # Temporary initialization HARDKODAT
        self.secondPos = self.firstPos  # Temporary initialization
	
	#Check if robot is already close enough to target position, else run first segment. 
        if (not (((np.absolute(self.targetPos[0] - self.secondPos[0])) <= 0.1) & ((np.absolute(self.targetPos[1] - self.secondPos[1])) <= 0.1))):
            self.driveForward()
            self.secondPos = self.getCoords(self.activeRobot) #np.array([], dtype=np.float32)#self.getCoords() # TEMPORARY HARDKODAT
            self.recordedPositions = np.append(self.recordedPositions , ([self.secondPos[0]], [self.secondPos[1]])) #Add position after movement to recorded positions
            if (not (((np.absolute(self.targetPos[0] - self.secondPos[0])) <= 0.1) & ((np.absolute(self.targetPos[1] - self.secondPos[1])) <= 0.1))):
                self.runNextSegment()
        else:
            return recordedPositions;


    #Reruns the next segment of total path until robot is close enough to target position. 
    def runNextSegment(self):
        angle = self.calculateAngle()
        self.rotate(angle)
        self.driveForward()
        self.firstPos = self.secondPos
        self.secondPos = self.getCoords(self.activeRobot)

	self.recordedPositions = np.append(self.recordedPositions , ([self.secondPos[0]], [self.secondPos[1]]))
        if (not (((np.absolute(self.targetPos[0] - self.secondPos[0])) <= 0.1) & ((np.absolute(self.targetPos[1] - self.secondPos[1])) <= 0.1))):
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
        srv = '/moveRobot1'#+self.activeRobot
        rospy.wait_for_service(srv)
        mvRobot = rospy.ServiceProxy(srv, MoveRobot)
        try:
            x = mvRobot(0.2)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))


    def getCoords(self, robotNbr):
	tmpPos = np.empty([], dtype=np.float32)
        srv = 'get_coord1'#+self.activeRobot #TODO robotnbr
        rospy.wait_for_service(srv)
	get_coords = rospy.ServiceProxy(srv, GetCoord)
        try:
	    f = Floats()
            f = get_coords()
	    tmpPos = f.data.data
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        return tmpPos


    #Calculates what angle the robot should turn for next segment
    def calculateAngle(self):
        fst = self.firstPos
        snd = self.secondPos
        target = self.targetPos
	print fst
	print snd
	print target

        direction = 0
        #Calculate if robot should turn right or left
        k_targ = (fst[1]-target[1])/(fst[0]-target[0])  # (yl-yt)/(xl-xt)
        k_move = (fst[1]-snd[1])/(fst[0]-snd[0])  # (yc-yl)/(xc-xl)
        if (fst[0] < 0 and fst[1] > 0) or (fst[0] > 0 and fst[1] < 0):
            if (k_move >= k_targ):
                direction = -1
            else:
                direction = 1
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

        turningDegree = direction*theta#(np.pi - math.asin(lengthB*(math.sin(theta)/lengthToTarget)))

        return turningDegree


if __name__ == '__main__':
    rospy.init_node('masternode')

    try:
        ne = MainController()
    except rospy.ROSInterruptException: pass
