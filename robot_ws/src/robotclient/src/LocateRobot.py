#!/usr/bin/env python
PKG = 'numpy'
import numpy as np
import numpy.matlib as npm
import numpy.linalg as npl

def locateRobot(nodePos, robotNodeDist, maxOkRes, maxOkResDiff):
    """

    :param nodePos: matrix containing the position of the nodes. x-coordinates in the first row, y-coordinates
    in the second row.
    :param robotNodeDist: matrix containing the distance between each robot and each node. element i,j in this matrix
    represent the distance between node i and robot j.
    :param maxOkRes: Maximum average residual in position (sum of the square of the error in the calculated position
    and measured position to all noes) that the user is satisfied with.
    :param maxOkResDiff: Maximum average difference between last residual in position (sum of the square of the error
    in the calculated position and measured position to all noes) and current residual that the user is satisfied with.
    If the psoition does not change significantly with each iteration but the error is still too great.
    :return: Matrix containing x and y coorinates of the robots
    """

    nodePosX = np.array([nodePos[0, :]]).T  # creates a column vector of node's x-pos
    nodePosY = np.array([nodePos[1, :]]).T  # creates a column vector of node's y-pos
    # creates a starting point ([x0;y0]) for the steepest descent to work with for all positions
    robotPos = np.concatenate([npm.repmat(np.mean(nodePosX), 1, np.size(robotNodeDist[0, :])),
                               npm.repmat(np.mean(nodePosY), 1, np.size(robotNodeDist[0, :]))])

    residual = npm.repmat(100, 1, np.size(robotNodeDist[0, :]))  # residual from least square method
    lastResidual = npm.repmat(0, 1, np.size(robotNodeDist[0, :]))  # residual from least square method

    while np.abs(npl.norm(residual)-npl.norm(lastResidual)) > (maxOkResDiff * len(residual)):
        lastResidual = residual

        # update the calculated robot position from last iteration and make maxtix out of it
        # this is because we can solve the position from all of the nodes at the same time.
        # element i,j in this matrix represents robot j's (new) position after it have moved in a radial distance
        # to decrease the error in the calculated distance to the node and the the measured distance to the node.
        # these things will be done later in the code.
        robotX = npm.repmat(robotPos[0, :], np.size(nodePos[0, :]), 1)
        robotY = npm.repmat(robotPos[1, :], np.size(nodePos[0, :]), 1)

        # calculate new distance between nodes and robots. The distance is between each of the nodes to each of
        # the robots. element i,j in this matrix represend the distance between node i to robot j.
        robotNodeDistCalc = np.sqrt((robotX - nodePosX)**2 + (robotY - nodePosY)**2)

        # difference in distance between current robot position to node and measured  robot position to node.
        distError = robotNodeDist - robotNodeDistCalc
        # row vector of residuals for each position
        residual = npl.norm(distError, axis=0)

        if npl.norm(residual) > (maxOkRes * len(residual)):

            # move the robot half the difference in calculated distance to the node and the measured distance to
            # the node in the positive or negative radial direction depending on if the calculated distance is closer
            # or farther away than the measuresured distance.
            moveX = (robotX-nodePosX) / robotNodeDistCalc * distError.astype(float) / 2
            moveY = (robotY-nodePosY) / robotNodeDistCalc * distError.astype(float) / 2

            # sum the calculated steps for each robot with respect to each node.
            # This will produce a movement is the direction of the gradient.
            robotPos[0, :] = robotPos[0, :] + moveX.sum(axis=0)
            robotPos[1, :] = robotPos[1, :] + moveY.sum(axis=0)
        else:
            break
    return robotPos
