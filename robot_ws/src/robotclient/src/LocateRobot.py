#!/usr/bin/env python
PKG = 'numpy'
import numpy as np
import numpy.matlib as npm
import numpy.linalg as npl


def locate_robot(anchors_pos, robot_anchor_dist, max_tol):
    """

    :param anchors_pos: A matrix containing the coordinates of the anchors
    :param robot_anchor_dist: matrix containing the distance between each robot and each node. element
    i,j in this matrix represent the distance between node i and robot j.
    :param max_tol: A list of maximum average residual in position as well as maximum average difference
    between last residual in position and current residual that the user is satisfied with.
    :return: Matrix containing x and y coordinates of the robots
    """
    anchor_pos_x = np.array([anchors_pos[0, :]]).T  # creates a column vector of node's x-pos
    anchor_pos_y = np.array([anchors_pos[1, :]]).T  # creates a column vector of node's y-pos
    # creates a starting point ([x0;y0]) for the steepest descent to work with for all positions
    robot_pos = np.concatenate([npm.repmat(np.mean(anchor_pos_x), 1, np.size(robot_anchor_dist[0, :])),
                                npm.repmat(np.mean(anchor_pos_y), 1, np.size(robot_anchor_dist[0, :]))])

    residual = npm.repmat(100, 1, np.size(robot_anchor_dist[0, :]))  # residual from least square method
    last_residual = npm.repmat(0, 1, np.size(robot_anchor_dist[0, :]))  # residual from least square method

    while np.abs(npl.norm(residual)-npl.norm(last_residual)) > (max_tol[1] * len(residual)):
        last_residual = residual

        # update the calculated robot position from last iteration and make matrix out of it
        # this is because we can solve the position from all of the nodes at the same time.
        # element i,j in this matrix represents robot j's (new) position after it have moved in a radial distance
        # to decrease the error in the calculated distance to the node and the the measured distance to the node.
        # these things will be done later in the code.
        robot_x = npm.repmat(robot_pos[0, :], np.size(anchors_pos[0, :]), 1)
        robot_y = npm.repmat(robot_pos[1, :], np.size(anchors_pos[0, :]), 1)

        # calculate new distance between nodes and robots. The distance is between each of the nodes to each of
        # the robots. element i,j in this matrix represent the distance between node i to robot j.
        robot_anchor_dist_calc = np.sqrt((robot_x - anchor_pos_x)**2 + (robot_y - anchor_pos_y)**2)

        # difference in distance between current robot position to node and measured  robot position to node.
        dist_error = robot_anchor_dist - robot_anchor_dist_calc
        # row vector of residuals for each position
        residual = npl.norm(dist_error, axis=0)

        if npl.norm(residual) > (max_tol[0] * len(residual)):

            # move the robot half the difference in calculated distance to the node and the measured distance to
            # the node in the positive or negative radial direction depending on if the calculated distance is closer
            # or farther away than the measured distance.
            move_x = (robot_x-anchor_pos_x) / robot_anchor_dist_calc * dist_error.astype(float) / 2
            move_y = (robot_y-anchor_pos_y) / robot_anchor_dist_calc * dist_error.astype(float) / 2

            # sum the calculated steps for each robot with respect to each node.
            # This will produce a movement is the direction of the gradient.
            robot_pos[0, :] = robot_pos[0, :] + move_x.sum(axis=0)
            robot_pos[1, :] = robot_pos[1, :] + move_y.sum(axis=0)
        else:
            break
    return robot_pos
