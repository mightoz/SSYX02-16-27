#!/usr/bin/env python

#import roslib; roslib.load_manifest(PKG)
import rospy

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import Twist

import numpy as np
import math


class MainNode():
    

    def main(self):
        rospy.init_node('masternode')
        atGoal = False
        activeRobot = 0
        numberofRobots  = 1
        robot_1_coords = np.array([], dtype=np.float32) #Logged coordinates for robot 1
        robot_2_coords = np.array([], dtype=np.float32) #Logged coordinates for robot 2
    
 
        #Get starting coordinates
        currentpos1 = getCoords(1)
        robot_1_coords.append(currentpos[0], currentpos[1])
        currentpos2 = getCoords(2)
        robot_1_coords.append(currentpos[0], currentpos[1])

        #Set positions for end nodes
        end_node = np.array([1, -2], dtype=np.float32)
        master_node = np.array([0, 1], dtype=np.float32)

        
        #Direction vector calculation
        v= np.array([], dtype=np.float32)
        v_x = master_node[0] - end_node[0]
        v_y = master_node[1] - end_node[1]
        v.add(v_x, v_y)



        #While distance from either first or robot to perfect line is further away than 10 centimeters, execute the move
        while ((np.absolute(np.cross(v, np.array[(master_node[0] - currentpos1[0]), (master_node[1] - currentpos1[1])]))/np.absolute(v) > 0.1) || 
            (np.absolute(np.cross(v, np.array[(master_node[0] - currentpos2[0]), (master_node[1] - currentpos2[1])]))/np.absolute(v) > 0.1) ):
            #Test each robot and see if they're on the correct 'optimal' position, if not move and append the logged positions from moveTo to the coordinates for each robot
            for i in numberofRobots:
                nextPosition = np.array([], dtype=np.float32)
                nextPosition = correctPosition(self, i)
                if self.getCoords(i) == nextPosition:
                    pass
                else:
                    activeRobot = i
                    res = moveTo(nextPosition)
                    if activeRobot=1:
                    currentpos1=self.getCoords(1)
                    robot_1_coords.append(res)
                    elif activeRobot=2:
                    currentpos2=self.getCoords(2)
                    robot_2_coords.append(res)



    def correctPos(self, robot):
        #Calculates correct position for robot depending on active robot
        correctPosition = np.array([], dtype=np.float32)
        if (robot==0):
            nextCoordx = (end_node[0] + self.currentpos2[0])/2 
            nextCoordy =  (end_node[1] + self.currentpos2[1])/2
            correctPosition.add(nextCoordx, nextCoordy)
        elif (robot==1):
            nextCoordx = (self.currentpos2[0] + master_node[0])/2
            nextCoordx = (self.currentpos2[1] + master_node[1])/2
            correctPosition.add(nextCoordx, nextCoordy)
        else:  
            pass
        return correctPosition
     



    def moveTo(coord):
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
