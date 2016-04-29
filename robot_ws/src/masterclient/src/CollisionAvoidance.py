import numpy as np
import matplotlib.pyplot as plt
import Kalman
import Controls

"""
Send position, orientation and controls and predict BEFORE applying controls to see
if there might be any collision
TODO: apply gradient descent. if reltol is reached then there should not be any collision. reltol should
be quite small. if abstol is reached, there may be a collision. abstol should be quite large.

TODO: add a function that checks gradient descent for each pair of robots.
"""


def predict(pos, theta, x, z, time_step):
    """
    This method can be used to predict the robots position after dt seconds solely based on
    the controls. The control error increases when using this method as no correction is made.

    :param pos: position vector [[x], [y]] before control was applied
    :param theta: orientation before control was applied relative to grid (theta = 0 means the robot is facing
    in +x direction)
    :param x: forward/backward motion control
    :param z: anti-clockwise/clockwise motion control
    :param time_step: time to execute controls during
    :return: A prediction of the current position based on the control input and last position position
    """
    time_step = np.abs(time_step)
    if np.abs(z) > 1e-40:
        x_k_k1 = np.array([[pos[0]+(np.sin(theta+z*time_step)-np.sin(theta))*x/z],
                           [x*np.cos(theta)],
                           [pos[1]+(np.cos(theta)-np.cos(theta+z*time_step))*x/z],
                           [x*np.sin(theta)]])  # predicted state
    else:
        x_k_k1 = np.array([[pos[0]+x*np.cos(theta)*time_step],
                           [x*np.cos(theta)],
                           [pos[1]+x*np.sin(theta)*time_step],
                           [x*np.sin(theta)]])  # predicted state
    theta += z*time_step
    return x_k_k1, theta


def get_rot_dir(theta, q, rob_2_pos):
    """

    :param theta: angle
    :param q: point on the circle with radius abstol centered at rob_2_pos [q_x, q_y]
    :param rob_2_pos: robot 2 position [x, y]
    :return:
    """
    # calculate the angle between the x-axis and the line intersecting end_pos and start_pos
    if np.abs(q[0] - rob_2_pos[0]) > 1e-30:
        z1 = q - rob_2_pos
        z2 = np.array([1, 0])
        if q[1] >= rob_2_pos[1]:  # 0 <= phi <= pi
            phi = np.arccos(np.dot(z1, z2) / (np.linalg.norm(z1) * np.linalg.norm(z2)))
        else:  # pi < phi < 2*pi
            phi = 2 * np.pi - np.arccos(np.dot(z1, z2) / (np.linalg.norm(z1) * np.linalg.norm(z2)))
    else:
        phi = np.sign(q[1] - rob_2_pos[1]) * np.pi / 2

    # determine the rotation direction (+1 ccw, -1 cw)
    if 3*np.pi/2 <= phi < 2*np.pi:
        if np.mod(theta, 2*np.pi) < phi:
            angle = phi - np.pi/2
        else:
            angle = phi + np.pi/2
    elif 0 <= phi < np.pi/2:
        if np.mod(theta, 2*np.pi) >= phi:
            angle = phi + np.pi/2
        else:
            angle = phi - np.pi/2
    else:  # np.pi/2 <= phi < 3*np.pi/2
        if np.mod(theta, 2*np.pi) >= phi:
            angle = phi + np.pi/2
        else:
            angle = phi - np.pi/2
    return angle


def __tar_fun_lin_lin__(pos_1, theta_1, x_1, pos_2, theta_2, x_2, t_2, t_1):  # rob1 lin, rob2 lin
    result = ((pos_1-pos_2)[0] + x_1 * np.cos(theta_1) * t_1 - x_2 * np.cos(theta_2) * t_2) ** 2 + \
             ((pos_1-pos_2)[1] + x_1 * np.sin(theta_1) * t_1 - x_2 * np.sin(theta_2) * t_2) ** 2
    return result


def __tar_grad_lin_lin__(pos_1, theta_1, x_1, pos_2, theta_2, x_2, t_2, t_1):  # rob1 lin, rob2 lin
    grad_t_2 = -2 * x_2 * np.cos(theta_2) * ((pos_1 - pos_2)[0] + x_1 * np.cos(theta_1) * t_1 - x_2 *
                                             np.cos(theta_2) * t_2) - \
               2 * x_2 * np.sin(theta_2) * ((pos_1 - pos_2)[1] + x_1 * np.sin(theta_1) * t_1 - x_2 *
                                            np.sin(theta_2) * t_2)
    grad_t_1 = 2 * x_1 * np.cos(theta_1) * ((pos_1 - pos_2)[0] + x_1 * np.cos(theta_1) * t_1 - x_2 *
                                            np.cos(theta_2) * t_2) + \
               2 * x_1 * np.sin(theta_1) * ((pos_1 - pos_2)[1] + x_1 * np.sin(theta_1) * t_1 - x_2 *
                                            np.sin(theta_2) * t_2)
    return grad_t_2, grad_t_1


def __tar_fun_rot_rot__(pos_1, theta_1, x_1, z_1, pos_2, theta_2, x_2, z_2, t_2, t_1):  # rob1 rot, rob2 rot
    result = ((pos_1-pos_2)[0] + x_1 / z_1 * (np.sin(theta_1 + z_1 * t_1) - np.sin(theta_1)) -
              x_2 / z_2 * (np.sin(theta_2 + z_2 * t_2) - np.sin(theta_2))) ** 2 + \
             ((pos_1-pos_2)[1] + x_1 / z_1 * (np.cos(theta_1) - np.cos(theta_1 + z_1 * t_1)) -
              x_2 / z_2 * (np.cos(theta_2) - np.cos(theta_2 + z_2 * t_2))) ** 2
    return result


def __tar_grad_rot_rot__(pos_1, theta_1, x_1, z_1, pos_2, theta_2, x_2, z_2, t_2, t_1):  # rob1 rot, rob2 rot
    grad_t_2 = -2 * x_2 * np.cos(theta_2 + z_2 * t_2) * \
               ((pos_1-pos_2)[0] + x_1 / z_1 * (np.sin(theta_1 + z_1 * t_1) - np.sin(theta_1)) -
                x_2 / z_2 * (np.sin(theta_2 + z_2 * t_2) - np.sin(theta_2))) - \
               2 * x_2 * np.sin(theta_2 + z_2 * t_2) * \
               ((pos_1-pos_2)[1] + x_1 / z_1 * (np.cos(theta_1) - np.cos(theta_1 + z_1 * t_1)) -
                x_2 / z_2 * (np.cos(theta_2) - np.cos(theta_2 + z_2 * t_2)))
    grad_t_1 = 2 * x_1 * np.cos(theta_1 + z_1 * t_1) * \
               ((pos_1-pos_2)[0] + x_1 / z_1 * (np.sin(theta_1 + z_1 * t_1) - np.sin(theta_1)) -
                x_2 / z_2 * (np.sin(theta_2 + z_2 * t_2) - np.sin(theta_2))) + \
               2 * x_1 * np.sin(theta_1 + z_1 * t_1) * \
               ((pos_1-pos_2)[1] + x_1 / z_1 * (np.cos(theta_1) - np.cos(theta_1 + z_1 * t_1)) -
                x_2 / z_2 * (np.cos(theta_2) - np.cos(theta_2 + z_2 * t_2)))
    return grad_t_2, grad_t_1


def __tar_fun_lin_rot__(pos_1, theta_1, x_1, pos_2, theta_2, x_2, z_2, t_2, t_1):  # rob1 lin, rob2 rot
    result = ((pos_1 - pos_2)[0] + x_1 * np.cos(theta_1) * t_1 - x_2 / z_2 *
              (np.sin(theta_2 + z_2 * t_2) - np.sin(theta_2)))**2 + \
             ((pos_1 - pos_2)[1] + x_1 * np.sin(theta_1) * t_1 - x_2 / z_2 *
              (np.cos(theta_2) - np.cos(theta_2 + z_2 * t_2)))**2
    return result


def __tar_grad_lin_rot__(pos_1, theta_1, x_1, pos_2, theta_2, x_2, z_2, t_2, t_1):  # rob1 lin, rob2 rot
    grad_t_2 = -2 * x_2 * np.cos(theta_2 + z_2 * t_2) * \
               ((pos_1 - pos_2)[0] + x_1 * np.cos(theta_1) * t_1 -
                x_2 / z_2 * (np.sin(theta_2 + z_2 * t_2) - np.sin(theta_2))) - \
               2 * x_2 * np.sin(theta_2 + z_2 * t_2) * \
               ((pos_1 - pos_2)[1] + x_1 * np.sin(theta_1) * t_1 -
                x_2 / z_2 * (np.cos(theta_2) - np.cos(theta_2 + z_2 * t_2)))
    grad_t_1 = 2 * x_1 * np.cos(theta_1) * ((pos_1 - pos_2)[0] + x_1 * np.cos(theta_1) * t_1 -
                                            x_2 / z_2 * (np.sin(theta_2 + z_2 * t_2) - np.sin(theta_2))) + \
               2 * x_1 * np.sin(theta_1) * ((pos_1 - pos_2)[1] + x_1 * np.sin(theta_1) * t_1 -
                                            x_2 / z_2 * (np.cos(theta_2) - np.cos(theta_2 + z_2 * t_2)))
    return grad_t_2, grad_t_1


def __tar_fun_rot_lin__(pos_1, theta_1, x_1, z_1, pos_2, theta_2, x_2, t_2, t_1):  # rob1 rot, rob2 lin
    result = ((pos_2 - pos_1)[0] + x_2 * np.cos(theta_2) * t_2 - x_1 / z_1 *
              (np.sin(theta_1 + z_1 * t_1) - np.sin(theta_1))) ** 2 + \
             ((pos_2 - pos_1)[1] + x_2 * np.sin(theta_2) * t_2 - x_1 / z_1 *
              (np.cos(theta_1) - np.cos(theta_1 + z_1 * t_1))) ** 2
    return result


def __tar_grad_rot_lin__(pos_1, theta_1, x_1, z_1, pos_2, theta_2, x_2, t_2, t_1):  # rob1 rot, rob2 lin
    grad_t_2 = 2 * x_2 * np.cos(theta_2) * ((pos_2 - pos_1)[0] + x_2 * np.cos(theta_2) * t_2 -
                                            x_1 / z_1 * (np.sin(theta_1 + z_1 * t_1) - np.sin(theta_1))) + \
               2 * x_2 * np.sin(theta_2) * ((pos_2 - pos_1)[1] + x_2 * np.sin(theta_2) * t_2 -
                                            x_1 / z_1 * (np.cos(theta_1) - np.cos(theta_1 + z_1 * t_1)))
    grad_t_1 = -2 * x_1 * np.cos(theta_1 + z_1 * t_1) * \
               ((pos_2 - pos_1)[0] + x_2 * np.cos(theta_2) * t_2 -
                x_1 / z_1 * (np.sin(theta_1 + z_1 * t_1) - np.sin(theta_1))) - \
               2 * x_1 * np.sin(theta_1 + z_1 * t_1) * \
               ((pos_2 - pos_1)[1] + x_2 * np.sin(theta_2) * t_2 -
                x_1 / z_1 * (np.cos(theta_1) - np.cos(theta_1 + z_1 * t_1)))
    return grad_t_2, grad_t_1


class CollisionAvoidance(object):

    def __init__(self, abs_tol, rel_tol, k):
        self.abs_tol = abs_tol
        self.rel_tol = rel_tol
        self.k = k

    def gradient_descent(self, pos_1, theta_1, x_1, z_1, pos_2, theta_2, x_2, z_2, time_exec):
        t_2 = 0
        t_1 = 0
        last_residual = 2e2
        residual = 1e2
        if np.abs(z_1) > 1e-40:
            if np.abs(z_2) > 1e-40:  # rob1 rot rob2 rot
                while np.abs(last_residual-residual) > self.rel_tol:
                    last_residual = residual
                    dt_2, dt_1 = __tar_grad_rot_rot__(pos_1, theta_1, x_1, z_1, pos_2, theta_2, x_2, z_2, t_2, t_1)
                    t_2 -= self.k*dt_2
                    t_1 -= self.k*dt_1
                    if 0 > t_2 or t_2 > time_exec or 0 > t_1 or t_1 > time_exec:
                        return False
                    residual = __tar_fun_rot_rot__(pos_1, theta_1, x_1, z_1, pos_2, theta_2, x_2, z_2, t_2, t_1)
                    if np.abs(residual) < self.abs_tol:
                        return True
            else:  # rob1 rot rob2 lin
                while np.abs(last_residual-residual) > self.rel_tol:
                    last_residual = residual
                    dt_2, dt_1 = __tar_grad_rot_lin__(pos_1, theta_1, x_1, z_1, pos_2, theta_2, x_2, t_2, t_1)
                    t_2 -= self.k*dt_2
                    t_1 -= self.k*dt_1
                    if 0 > t_2 or t_2 > time_exec or 0 > t_1 or t_1 > time_exec:
                        return False
                    residual = __tar_fun_rot_lin__(pos_1, theta_1, x_1, z_1, pos_2, theta_2, x_2, t_2, t_1)
                    if np.abs(residual) < self.abs_tol:
                        return True
        else:
            if np.abs(z_2) > 1e-40:  # rob1 lin rob2 rot
                while np.abs(last_residual-residual) > self.rel_tol:
                    last_residual = residual
                    dt_2, dt_1 = __tar_grad_lin_rot__(pos_1, theta_1, x_1, pos_2, theta_2, x_2, z_2, t_2, t_1)
                    t_2 -= self.k*dt_2
                    t_1 -= self.k*dt_1
                    if 0 > t_2 or t_2 > time_exec or 0 > t_1 or t_1 > time_exec:
                        return False
                    residual = __tar_fun_lin_rot__(pos_1, theta_1, x_1, pos_2, theta_2, x_2, z_2, t_2, t_1)
                    if np.abs(residual) < self.abs_tol:
                        return True
            else:  # rob1 lin rob2 lin
                while np.abs(last_residual-residual) > self.rel_tol:
                    last_residual = residual
                    dt_2, dt_1 = __tar_grad_lin_lin__(pos_1, theta_1, x_1, pos_2, theta_2, x_2, t_2, t_1)
                    t_2 -= self.k*dt_2
                    t_1 -= self.k*dt_1
                    if 0 > t_2 or t_2 > time_exec or 0 > t_1 or t_1 > time_exec:
                        return False
                    residual = __tar_fun_lin_lin__(pos_1, theta_1, x_1, pos_2, theta_2, x_2, t_2, t_1)
                    if np.abs(residual) < self.abs_tol:
                        return True
        return False

    def calc_new_controls(self, pos_1, theta_1, x_1, z_1, pos_2, theta_2, x_2, z_2, time_exec):
        new_x_1 = x_1
        new_z_1 = z_1
        new_x_2 = x_2
        new_z_2 = z_2
        collide1 = self.gradient_descent(pos_1, theta_1, new_x_1, new_z_1, pos_2, theta_2, new_x_2, new_z_2, time_exec)
        if collide1:
            print 'collide1'
            new_x_2 = 0
            new_z_2 = 0
            collide2 = self.gradient_descent(pos_1, theta_1, new_x_1, new_z_1, pos_2, theta_2, new_x_2, new_z_2, time_exec)
            if collide2:
                print 'collide2'
                t_2 = 0
                t_1 = 0
                last_residual = 2e2
                residual = 1e2
                if np.abs(new_z_1) > 1e-40:
                    while np.abs(last_residual-residual) > self.rel_tol:
                        last_residual = residual
                        dt_2, dt_1 = __tar_grad_rot_lin__(pos_1, theta_1, new_x_1, new_z_1, pos_2, theta_2, new_x_2, t_2, t_1)
                        t_2 -= self.k*dt_2
                        t_1 -= self.k*dt_1
                        residual = __tar_fun_rot_lin__(pos_1, theta_1, new_x_1, new_z_1, pos_2, theta_2, new_x_2, t_2, t_1)
                else:
                    while np.abs(last_residual-residual) > self.rel_tol:
                        last_residual = residual
                        dt_2, dt_1 = __tar_grad_lin_lin__(pos_1, theta_1, new_x_1, pos_2, theta_2, new_x_2, t_2, t_1)
                        t_2 -= self.k*dt_2
                        t_1 -= self.k*dt_1
                        residual = __tar_fun_lin_lin__(pos_1, theta_1, new_x_1, pos_2, theta_2, new_x_2, t_2, t_1)
                x, v = predict(pos_1, theta_1, new_x_1, new_z_1, t_1)
                p = np.array([x[0, 0], x[2, 0]])
                q = pos_2 + (p - pos_2) / np.linalg.norm(p - pos_2) * 1.1 * self.abs_tol  # 110% just to be sure
                angle = get_rot_dir(theta_1, q, pos_2)
                new_z_1 = (angle - theta_1) / t_1
                collide3 = self.gradient_descent(pos_1, theta_1, new_x_1, new_z_1, pos_2, theta_2, new_x_2, new_z_2, time_exec)
                if collide3:
                    print 'collide3', pos_1, theta_1*180/np.pi, pos_2, theta_2*180/np.pi
                    assd, dssa = predict(pos_1, theta_1, new_x_1, new_z_1, t_1)
                    poss1 = np.array([assd[0, 0], assd[2, 0]])
                    plt.plot(poss1[0], poss1[1], 'co')

                    plt.plot(p[0], p[1], 'mo')
                    plt.plot(q[0], q[1], 'ko')
                    plt.plot(pos_1[0], pos_1[1], 'go')
                    plt.plot(pos_2[0], pos_2[1], 'bo')
                    time = np.linspace(0, time_exec, 101)
                    pos1 = np.zeros((2, 101))
                    pos2 = np.zeros((2, 101))
                    for i in range(0, len(time)):
                        asd, dsa = predict(pos_1, theta_1, new_x_1, new_z_1, time[i])
                        pos1[:, i] = np.array([asd[0, 0], asd[2, 0]])
                        asdd, dsad = predict(pos_1, theta_1, x_1, z_1, time[i])
                        pos2[:, i] = np.array([asdd[0, 0], asdd[2, 0]])
                    plt.plot(pos1[0, :], pos1[1, :], 'g')
                    plt.plot(pos2[0, :], pos2[1, :], 'b')
                    plt.axis('equal')
                    plt.show()
                    new_x_1 = 0
        return new_x_1, new_z_1, new_x_2, new_z_2
