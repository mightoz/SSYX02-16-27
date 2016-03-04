#!/usr/bin/env python
PKG = 'numpy'

import numpy as np
import LocateRobot
import GetRangeMeasurment
import matplotlib.pyplot as plt


def RunLocRob(sock, reqIp, respID, nodePos, nbrOfSuccessReadings, maxOkRes, maxOkResDiff, doPrint):
    """

    :param sock: The socket used in communication between the computer and RCM connected via ethernet cable.
    :param reqIp: The IP of the RCM:s (wired) ip, eg 192.168.1.104
    :param respID: A row vector (N-by-1 matrix) containing the wireless ID of the nodes, for example 106, 109, 114 etc.
    The order of these have to correspond to the order of the nodes in the nodePos
    :param nodePos: A matrix containing the coordinates of the nodes. The first row contains the x-coordinates of
    the nodes. The second row contains the y-coordinates of the nodes.
    :param nbrOfSuccessReadings: The number of successful range measurement to each of the nodes. Three nodes and four
    success readings would produce 12 measurements in total.
    :param maxOkRes: Maximum average residual in position (sum of the square of the error in the calculated position
    and measured position to all noes) that the user is satisfied with.
    :param maxOkResDiff: Maximum average difference between last residual in position (sum of the square of the error
    in the calculated position and measured position to all noes) and current residual that the user is satisfied with.
    If the psoition does not change significantly with each iteration but the error is still too great.
    :param doPrint: 1 to output information about the measuring process, 0 to don't output any information. This
    parameter controls whether or not to plot circles around the nodes with the radius being the measured distances to
    the robots.
    :return: A matrix containing the positions of the robots. The first row contains the x-coordinates of the robots
    and the second row contains the y-coordinates of the robots.
    """

    # instantiate the row vector (N-by-1 matrix) that will hold the measured distance to the nodes. The order that these
    # distances will be in corresponds to the order of the respID parameter.
    distance = np.zeros((np.size(respID), 1), dtype=np.float)
    for i in range(0, nbrOfSuccessReadings):
        for j in range(0, np.size(respID)):
            dist = GetRangeMeasurment.measRange(sock, reqIp, respID[j, 0], doPrint)  # get the distance data.
            distance[j, 0] += np.float(dist)/nbrOfSuccessReadings  # add the average distance.

    # plots circles around the nodes with the radius being the measured distances to the robots.
    if doPrint:
        v = np.linspace(0, 2 * np.pi, 100)
        x = nodePos[0, 0] + distance[0, 0] * np.cos(v)
        y = nodePos[1, 0] + distance[0, 0] * np.sin(v)
        plt.plot(x, y)
        x = nodePos[0, 1] + distance[1, 0] * np.cos(v)
        y = nodePos[1, 1] + distance[1, 0] * np.sin(v)
        plt.plot(x, y)
        x = nodePos[0, 2] + distance[2, 0] * np.cos(v)
        y = nodePos[1, 2] + distance[2, 0] * np.sin(v)
        plt.plot(x, y)

    # calculate the position based on the distance measured above.
    pos = LocateRobot.locateRobot(nodePos, distance, maxOkRes, maxOkResDiff)
    return pos
