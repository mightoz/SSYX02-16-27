#!/usr/bin/env python
PKG = 'robotclient'
import roslib; roslib.load_manifest(PKG)
from robotclient.srv import *
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np




def handle_rotateRobot(req):
    """
    Rotates robot a given angle
    :param req: Message type containing angle to rotate
    :return:Acknowledgement Msg
    """
    print rospy.get_name(), "Rotatating: %s"%str(req.deg)
    # Publisher to rosaria node
    pub = rospy.Publisher("RosAria/cmd_vel", Twist, queue_size=10)    

    # Sleep for 0.01s to ensure ready for rotate
    twist = Twist()
    pub.publish(twist)
    rospy.loginfo("Sleeping for 10ms to make sure it's stopped.")
    rospy.sleep(0.01);

    # Fixed rotating speed, sleeptime changes depending on deg
    z = 0.5
    twist.angular.z = z*np.sign(req.deg)  # Control command for rosaria
    rospy.loginfo("Rotatating robot.")
    pub.publish(twist)
    rospy.sleep(np.abs(req.deg/z));  # Altered sleeptime

    rospy.loginfo("Stopping.")
    twist = Twist()
    pub.publish(twist)
    rospy.sleep(0.01);

    print rospy.get_name(), "Finished Rotatating"
    ack = 1
    return RotateRobotResponse(ack)


    
def rotateRobot_server():
    """
    Initiates service
    :return:
    """
    rospy.init_node('rotateRobot_service')
    s=rospy.Service('rotateRobot', RotateRobot, handle_rotateRobot)
    print rospy.get_name(), "Ready to rotate"
    rospy.spin()


if __name__ == "__main__":
    rotateRobot_server()

