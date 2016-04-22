#!/usr/bin/env python
PKG = 'numpy'
import numpy as np
import LocateRobot
import GetRangeMeasurment
import matplotlib.pyplot as plt


def run_loc_rob(s, ip, anchors, nbr_of_success_readings, max_tol, do_print):
    """

    :param s: The socket used in communication between the computer and RCM connected via ethernet cable.
    :param ip: The IP of the RCM:s (wired) ip, eg 192.168.1.104
    :param anchors: A list oftThe instances of the Anchor class used
    :param nbr_of_success_readings: The number of successful range measurement to each of the nodes. Three nodes and four
    success readings would produce 12 measurements in total.
    :param max_tol: A list of maximum average residual in position as well as maximum average difference between
    last residual in position and current residual that the user is satisfied with.
    :param do_print: 1 to output information about the measuring process, 0 to don't output any information. This
    parameter controls whether or not to plot circles around the nodes with the radius being the measured distances to
    the robots.
    :return: A matrix containing the positions of the robots. The first row contains the x-coordinates of the robots
    and the second row contains the y-coordinates of the robots.
    """

    # instantiate the row vector (N-by-1 matrix) that will hold the measured distance to the nodes. The order that these
    # distances will be in corresponds to the order of the anchors parameter.
    distance = np.zeros((np.size(anchors), 1), dtype=np.float)
    for i in range(0, nbr_of_success_readings):
        for j in range(0, np.size(anchors)):
            dist = GetRangeMeasurment.meas_range(s, ip, anchors[j].get_ip(), do_print)  # get the distance data.
            distance[j, 0] += np.float(dist) / nbr_of_success_readings  # add the average distance.

    # plots circles around the nodes with the radius being the measured distances to the robots.
    if do_print:
        v = np.linspace(0, 2 * np.pi, 100)
        x = anchors[0].get_pos()[0] + distance[0, 0] * np.cos(v)
        y = anchors[0].get_pos()[1] + distance[0, 0] * np.sin(v)
        plt.plot(x, y)
        x = anchors[1].get_pos()[0] + distance[1, 0] * np.cos(v)
        y = anchors[1].get_pos()[1] + distance[1, 0] * np.sin(v)
        plt.plot(x, y)
        x = anchors[0].get_pos()[0] + distance[2, 0] * np.cos(v)
        y = anchors[0].get_pos()[1] + distance[2, 0] * np.sin(v)
        plt.plot(x, y)

    # calculate the position based on the distance measured above.
    anchor_pos = np.zeros((2, np.size(anchors)))
    for i in range(0, np.size(anchors)):
        anchor_pos[:, i] = anchors[i].get_pos()
    pos = LocateRobot.locate_robot(anchor_pos, distance, max_tol)
    return pos
