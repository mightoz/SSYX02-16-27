#!/usr/bin/env python

#import roslib; roslib.load_manifest(PKG)
import rospy

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import Twist

import numpy as np
import math
import matplotlib.pyplot as plt

from robotclient.srv  import *

class MainController():
    

    def __init__(self):
#        rospy.Subscriber("testmove", numpy_msg(Floats), self.moveTo)
#        rospy.spin()
        atGoal = False
        self.activeRobot = 0
        numberofRobots  = 2
	self.lengthToTarget = 1
        robot_1_coords = np.array([], dtype=(np.float32,2)) #Logged coordinates for robot 1
        self.robot_1_xcoords = np.array([], dtype=np.float32) #Logged x-coordinates for robot 1
        self.robot_1_ycoords = np.array([], dtype=np.float32) #Logged y-coordinates for robot 1
        robot_2_coords = np.array([], dtype=(np.float32,2)) #Logged coordinates for robot 2
        self.robot_2_xcoords = np.array([], dtype=np.float32) #Logged x-coordinates for robot 1
        self.robot_2_ycoords = np.array([], dtype=np.float32) #Logged y-coordinates for robot 1
 
        #Get starting coordinates
	activeRobot = 1

	
	self.currentpos1 = np.array([], dtype=np.float32) # Temporary initialization HARDKODAT
	print "Trying to get coords for robot #1."
	self.currentpos1 = self.getCoords(1)
	print "currentpos1: ",(self.currentpos1) 
        robot_1_coords = np.append(robot_1_coords, [(self.currentpos1[0], self.currentpos1[1])])
        #robot_1_coords = np.append(robot_1_coords, [(self.currentpos1[0], self.currentpos1[1])])
	self.robot_1_xcoords = np.append(self.robot_1_xcoords, [self.currentpos1[0]])
	self.robot_1_ycoords = np.append(self.robot_1_ycoords, [self.currentpos1[1]])

	#print "All registered coords:",(robot_1_coords)
	#print "ALL X COORDS:", robot_1_xcoords
	#print "ALL Y COORDS:", robot_1_ycoords

	#activeRobot = 2 	#TODO FIXA
	#self.currentpos2 =  np.array([0,-1], dtype=np.float32)  # Temporary initialization HARDKODAT
	self.currentpos2 = np.array([], dtype=np.float32) # Temporary initialization HARDKODAT
	self.currentpos2 = self.getCoords(2)
	print "currentpos2: ", (self.currentpos2)
        robot_2_coords = np.append(robot_2_coords, ([self.currentpos2[0]], [self.currentpos2[1]]))
	

	#activeRobot = 1

        #Set positions for end nodes
        self.end_node = np.array([0,-2], dtype=np.float32)
        self.master_node = np.array([0,3], dtype=np.float32)
	
        
        #Direction vector calculation
        v_x = (self.master_node[0] - self.end_node[0])
        v_y = (self.master_node[1] - self.end_node[1])
        v= np.array([v_x, v_y], dtype=np.float32)
	
	

        #While distance from either first or robot to perfect line is further away than 10 centimeters, execute the move
        while ((np.absolute(np.cross(v, np.array([(self.master_node[0] - self.currentpos1[0]),(self.master_node[1] - self.currentpos1[1])])))/np.absolute(v[0] + 1j*v[1]) > 0.1) |
          (np.absolute(np.cross(v, np.array([(self.master_node[0] - self.currentpos2[0]), (self.master_node[1] - self.currentpos2[1])])))/np.absolute(v[0] + 1j*v[1]) > 0.1)):
            #Test each robot and see if they're on the correct 'optimal' position, if not move and append the logged positions from moveTo to the coordinates for each robot
	    print "Robot 1 not in place: ", (np.absolute(np.cross(v, np.array([(self.master_node[0] - self.currentpos1[0]),(self.master_node[1] - self.currentpos1[1])])))/np.absolute(v[0] + 1j*v[1]) > 0.1)
	    print "Robot 2 not in place: ", (np.absolute(np.cross(v, np.array([(self.master_node[0] - self.currentpos2[0]), (self.master_node[1] - self.currentpos2[1])])))/np.absolute(v[0] + 1j*v[1]) > 0.1)
            for i in (1, numberofRobots):
                nextPosition = np.array([], dtype=np.float32)
                nextPosition = self.correctPos(i)
		print "Calculated correct pos for robot #",i
		print "Correct pos: ", nextPosition
		print "Abovementionend robot close enough to correct pos: ", (np.sum(np.abs(self.getCoords(i) - nextPosition)) <= 0.1)
                if (np.sum(np.abs(self.getCoords(i) - nextPosition)) <= 0.1):#TODO- 
                    pass
                else:
                    self.activeRobot = i
                    robotPositions = self.moveTo(nextPosition)
                    if self.activeRobot==1:

                        self.currentpos1=self.getCoords(1) #TODO
                        robot_1_coords =  np.append(robot_1_coords, robotPositions)
			print robot_1_coords
                    elif self.activeRobot==2 :
                        self.currentpos2=self.getCoords(2) #TODO
                        robot_2_cords = np.append(robot_2_coords, robotPositions)

	    else:
		pass

	print "Recorded positions for robot 1: ", robot_1_coords
	print "X positions: ", self.robot_1_xcoords
	print "Y positions: ", self.robot_1_ycoords
	print "Recorded positions for robot 2: ", robot_2_coords
	print "X positions: ", self.robot_2_xcoords
	print "Y positions: ", self.robot_2_ycoords
	plt.plot(self.robot_1_xcoords, self.robot_1_ycoords, 'ro')
	plt.plot(self.robot_2_xcoords, self.robot_2_ycoords, 'bo')
	plt.plot(self.master_node[0], self.master_node[1], 'gx')
	plt.plot(self.end_node[0], self.end_node[1], 'gx')
	plt.plot(2,0, 'kx')
	plt.plot(1,-2, 'kx')
	plt.plot(-1,1, 'kx')
	plt.axis([-2, 3.5, -3, 3.5])
	plt.show()


    def correctPos(self, robot):
        #Calculates correct position for robot depending on active robot
        correctPosition = np.array([], dtype=np.float32)
        if (robot==1):
            nextCoordx = (self.master_node[0] + self.currentpos2[0])/2
            nextCoordy = (self.master_node[1] + self.currentpos2[1])/2
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
	
	print "Entered moveTo, activerobot: ", self.activeRobot
        #ar = self.activeRobot

        self.recordedPositions = np.array([], dtype=(np.float32, np.float32)) #All recorded positions
	self.recordedxPositions = np.array([], dtype=np.float32)
	self.recordedyPositions = np.array([], dtype=np.float32)
        self.targetPos = coord
	self.firstPos = self.getCoords(self.activeRobot)  # Temporary initialization HARDKODAT
        self.secondPos = self.firstPos  # Temporary initialization
	
	print "Targetpos: ", self.targetPos
	print "Secondpos: ", self.secondPos
	
        print "THIS IS THE IF STATEMENT IN MOVETO", (not (((np.absolute(self.targetPos[0] - self.secondPos[0])) <= 0.15) & ((np.absolute(self.targetPos[1] - self.secondPos[1])) <= 0.15)))
	#Check if robot is already close enough to target position, else run first segment. 
        if (not (((np.absolute(self.targetPos[0] - self.secondPos[0])) <= 0.15) & ((np.absolute(self.targetPos[1] - self.secondPos[1])) <= 0.15))):
            self.driveForward()
            self.secondPos = self.getCoords(self.activeRobot)
	    self.recordedxPositions = np.append(self.recordedxPositions , [self.secondPos[0]])
	    self.recordedyPositions = np.append(self.recordedyPositions , [self.secondPos[1]])
	    print "This is the position in x for moveTo:", self.recordedxPositions
	    print "This is the position in y for moveTo:", self.recordedyPositions
	    print "And return:",(self.recordedxPositions, self.recordedyPositions)	
            self.recordedPositions = np.append(self.recordedPositions , ([self.secondPos[0]], [self.secondPos[1]])) #Add position after movement to recorded positions
	    print "This is recorded pos:", self.recordedPositions
       	    if (not (((np.absolute(self.targetPos[0] - self.secondPos[0])) <= 0.15) & ((np.absolute(self.targetPos[1] - self.secondPos[1])) <= 0.15))):
                self.runNextSegment()
        else:
	    pass
	#Add recorded x and y positions to arrays of x and y positions for the active robot TODO make better
	#if (self.activeRobot ==1):
	#    print "coords should be appended."
	#    self.robot_1_xcoords = np.append(self.robot_1_xcoords, self.recordedxPositions)
	#    self.robot_1_ycoords = np.append(self.robot_1_ycoords, self.recordedyPositions)
	#elif(self.activeRobot ==2):
        #    self.robot_2_xcoords = np.append(self.robot_2_xcoords, self.recordedxPositions)
	#    self.robot_2_ycoords = np.append(self.robot_2_ycoords, self.recordedyPositions)
	#else:
	#    pass
	return self.recordedPositions


    #Reruns the next segment of total path until robot is close enough to target position. 
    def runNextSegment(self):
        angle = self.calculateAngle()
        self.rotate(angle)
        self.driveForward()
        self.firstPos = self.secondPos
        self.secondPos = self.getCoords(self.activeRobot)

	self.recordedPositions = np.append(self.recordedPositions , ([self.secondPos[0]], [self.secondPos[1]]))
	self.recordedxPositions = np.append(self.recordedxPositions , [self.secondPos[0]])
	self.recordedyPositions = np.append(self.recordedyPositions , [self.secondPos[1]])
	print "This is the position in x for nextSegment:", self.recordedxPositions
	print "This is the position in y for nextSegment:", self.recordedyPositions
	    
	
        if (not (((np.absolute(self.targetPos[0] - self.secondPos[0])) <= 0.15) & ((np.absolute(self.targetPos[1] - self.secondPos[1])) <= 0.15))):
            self.runNextSegment()
        else:
	    if (self.activeRobot == 1):
	    	self.robot_1_xcoords = np.append(self.robot_1_xcoords, self.recordedxPositions)
	    	self.robot_1_ycoords = np.append(self.robot_1_ycoords, self.recordedyPositions)
	    elif(self.activeRobot == 2):
		self.robot_2_xcoords = np.append(self.robot_2_xcoords, self.recordedxPositions)
	    	self.robot_2_ycoords = np.append(self.robot_2_ycoords, self.recordedyPositions)
	    else:
		pass
            return None

    def rotate(self,angle):
        srv = '/rotateRobot'+str(self.activeRobot)
        rospy.wait_for_service(srv)
        mvRobot = rospy.ServiceProxy(srv, RotateRobot)
        try:
            x = mvRobot(angle)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def driveForward(self):
        srv = '/moveRobot'+str(self.activeRobot)
	if(self.lengthToTarget <= 0.2):
	    length = self.lengthToTarget
	else:
	    length = 0.2
        rospy.wait_for_service(srv)
        mvRobot = rospy.ServiceProxy(srv, MoveRobot)
        try:
            x = mvRobot(length)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))


    def getCoords(self, robotNbr):
	tmpPos = np.empty([], dtype=np.float32)
        srv = 'get_coord'+str(robotNbr)
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
	print "First position: ", fst
	print "Second position: ", snd
	print "Current target: ", target

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
        """
        # Calculate degrees to turn
        theta = np.arccos(np.sum((target-fst)*(snd-fst))/(np.sqrt(np.sum((target-fst)**2))*np.sqrt(np.sum((snd-fst)**2))))
        return direction*theta
        """
        #Calculate degrees to turn
        dotProd = (snd[0]-fst[0])*(target[0]-fst[0])+(snd[1]-fst[1])*(target[1]-fst[1])

        lengthA = math.sqrt(math.pow((snd[0]-fst[0]),2)+math.pow((snd[1]-fst[1]),2))
        lengthB = math.sqrt(math.pow((target[0]-fst[0]),2)+math.pow((target[1]-fst[1]),2))

        self.lengthToTarget =  math.sqrt(math.pow((target[0]-snd[0]),2)+math.pow((target[1]-snd[1]),2))

        print rospy.get_name(), "LengthToTarget: %s"%str(self.lengthToTarget)

        theta = math.acos(dotProd/(lengthA*lengthB))

        print rospy.get_name(), "Turningdegree: %s"%str(theta)
	print rospy.get_name(), "Direction: %s"%str(direction)

        turningDegree = direction*theta#(np.pi - math.asin(lengthB*(math.sin(theta)/lengthToTarget)))

        return turningDegree


if __name__ == '__main__':
    rospy.init_node('masternode')

    try:
        ne = MainController()
    except rospy.ROSInterruptException: pass
