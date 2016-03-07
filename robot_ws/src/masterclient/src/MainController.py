#!/usr/bin/env python

#import roslib; roslib.load_manifest(PKG)
import rospy

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import Twist

import numpy as np
import math

robot1coords = np.array([], dtype = np.float32);
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

if __name__ == '__main__':
     main()
