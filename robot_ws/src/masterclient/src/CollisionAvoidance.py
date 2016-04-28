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
class CollisionAvoidance(object):

    def __init__(self, nodes):
        self.nodes = nodes
        pass

    def tar_fun_lin(self, pos_1, theta_1, x_1, pos_2, theta_2, x_2, s, t):
        result = ((pos_1-pos_2)[0] + x_1 * np.cos(theta_1) * t - x_2 * np.cos(theta_2) * s) ** 2 + \
                 ((pos_1-pos_2)[1] + x_1 * np.sin(theta_1) * t - x_2 * np.sin(theta_2) * s) ** 2
        return result

    def tar_grad_lin(self, pos_1, theta_1, x_1, pos_2, theta_2, x_2, s, t):
        x = -2 * x_2 * np.cos(theta_2) * ((pos_1 - pos_2)[0] + x_1 * np.cos(theta_1) * t - x_2 * np.cos(theta_2) * s) - \
            2 * x_2 * np.sin(theta_2) * ((pos_1 - pos_2)[1] + x_1 * np.sin(theta_1) * t - x_2 * np.sin(theta_2) * s)
        y = 2 * x_1 * np.cos(theta_1) * ((pos_1 - pos_2)[0] + x_1 * np.cos(theta_1) * t - x_2 * np.cos(theta_2) * s) + \
            2 * x_1 * np.sin(theta_1) * ((pos_1 - pos_2)[1] + x_1 * np.sin(theta_1) * t - x_2 * np.sin(theta_2) * s)
        return x, y

    def tar_fun_rot(self, pos_1, theta_1, x_1, z_1, pos_2, theta_2, x_2, z_2, s, t):
        result = ((pos_1-pos_2)[0] + x_1/z_1*(np.sin(theta_1+z_1*t)-np.sin(theta_1)) -
                  x_2/z_2*(np.sin(theta_2+z_2*s)-np.sin(theta_2)))**2 + \
                 ((pos_1-pos_2)[1] + x_1/z_1*(np.cos(theta_1)-np.cos(theta_1+z_1*t)) -
                  x_2/z_2*(np.cos(theta_2)-np.cos(theta_2+z_2*s)))**2
        return result

    def tar_grad_rot(self, pos_1, theta_1, x_1, z_1, pos_2, theta_2, x_2, z_2, s, t):
        x = -2 * x_2 * np.cos(theta_2 + z_2 * s) * \
            ((pos_1-pos_2)[0] + x_1/z_1*(np.sin(theta_1+z_1*t)-np.sin(theta_1)) -
             x_2/z_2*(np.sin(theta_2+z_2*s)-np.sin(theta_2))) - \
            2 * x_2 * np.sin(theta_2 + z_2 * s) * \
            ((pos_1-pos_2)[1] + x_1/z_1*(np.cos(theta_1)-np.cos(theta_1+z_1*t)) -
             x_2/z_2*(np.cos(theta_2)-np.cos(theta_2+z_2*s)))
        y = 2 * x_1 * np.cos(theta_1 + z_1 * t) * \
            ((pos_1-pos_2)[0] + x_1/z_1*(np.sin(theta_1+z_1*t)-np.sin(theta_1)) -
             x_2/z_2*(np.sin(theta_2+z_2*s)-np.sin(theta_2))) + \
            2 * x_1 * np.sin(theta_1 + z_1 * t) * \
            ((pos_1-pos_2)[1] + x_1/z_1*(np.cos(theta_1)-np.cos(theta_1+z_1*t)) -
             x_2/z_2*(np.cos(theta_2)-np.cos(theta_2+z_2*s)))
        return x, y


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


def tar_fun_lin(pos_1, theta_1, x_1, pos_2, theta_2, x_2, s, t):
    result = ((pos_1-pos_2)[0] + x_1 * np.cos(theta_1) * t - x_2 * np.cos(theta_2) * s) ** 2 + \
             ((pos_1-pos_2)[1] + x_1 * np.sin(theta_1) * t - x_2 * np.sin(theta_2) * s) ** 2
    return result


def tar_grad_lin(pos_1, theta_1, x_1, pos_2, theta_2, x_2, s, t):
    x = -2 * x_2 * np.cos(theta_2) * ((pos_1 - pos_2)[0] + x_1 * np.cos(theta_1) * t - x_2 * np.cos(theta_2) * s) - \
        2 * x_2 * np.sin(theta_2) * ((pos_1 - pos_2)[1] + x_1 * np.sin(theta_1) * t - x_2 * np.sin(theta_2) * s)
    y = 2 * x_1 * np.cos(theta_1) * ((pos_1 - pos_2)[0] + x_1 * np.cos(theta_1) * t - x_2 * np.cos(theta_2) * s) + \
        2 * x_1 * np.sin(theta_1) * ((pos_1 - pos_2)[1] + x_1 * np.sin(theta_1) * t - x_2 * np.sin(theta_2) * s)
    return x, y


def tar_fun_rot(pos_1, theta_1, x_1, z_1, pos_2, theta_2, x_2, z_2, s, t):
    result = ((pos_1-pos_2)[0] + x_1/z_1*(np.sin(theta_1+z_1*t)-np.sin(theta_1)) -
              x_2/z_2*(np.sin(theta_2+z_2*s)-np.sin(theta_2)))**2 + \
             ((pos_1-pos_2)[1] + x_1/z_1*(np.cos(theta_1)-np.cos(theta_1+z_1*t)) -
              x_2/z_2*(np.cos(theta_2)-np.cos(theta_2+z_2*s)))**2
    return result


def tar_grad_rot(pos_1, theta_1, x_1, z_1, pos_2, theta_2, x_2, z_2, s, t):
    x = -2 * x_2 * np.cos(theta_2 + z_2 * s) * \
        ((pos_1-pos_2)[0] + x_1/z_1*(np.sin(theta_1+z_1*t)-np.sin(theta_1)) -
         x_2/z_2*(np.sin(theta_2+z_2*s)-np.sin(theta_2))) - \
        2 * x_2 * np.sin(theta_2 + z_2 * s) * \
        ((pos_1-pos_2)[1] + x_1/z_1*(np.cos(theta_1)-np.cos(theta_1+z_1*t)) -
         x_2/z_2*(np.cos(theta_2)-np.cos(theta_2+z_2*s)))
    y = 2 * x_1 * np.cos(theta_1 + z_1 * t) * \
        ((pos_1-pos_2)[0] + x_1/z_1*(np.sin(theta_1+z_1*t)-np.sin(theta_1)) -
         x_2/z_2*(np.sin(theta_2+z_2*s)-np.sin(theta_2))) + \
        2 * x_1 * np.sin(theta_1 + z_1 * t) * \
        ((pos_1-pos_2)[1] + x_1/z_1*(np.cos(theta_1)-np.cos(theta_1+z_1*t)) -
         x_2/z_2*(np.cos(theta_2)-np.cos(theta_2+z_2*s)))
    return x, y

k = 0.1
T = 2
rob1 = [np.array([0.5, 0.5]), 3*np.pi/4, 1, 0.5]
rob2 = [np.array([-0.5, 0.5]), np.pi/4, 1, 1]
plt.figure()

a = np.linspace(0, 2)
b = np.linspace(0, 2)
A, B = np.meshgrid(a, b)
# val = tar_fun_lin(rob1[0], rob1[1], rob1[2], rob2[0], rob2[1], rob2[2], A, B)
val = tar_fun_rot(rob1[0], rob1[1], rob1[2], rob1[3], rob2[0], rob2[1], rob2[2], rob2[3], A, B)
f = 0
g = 0
plt.plot(f, g, 'ro')
for i in range(0, 100):
    # v, u = tar_grad_lin(rob1[0], rob1[1], rob1[2], rob2[0], rob2[1], rob2[2], f, g)
    v, u = tar_grad_rot(rob1[0], rob1[1], rob1[2], rob1[3], rob2[0], rob2[1], rob2[2], rob2[3], f, g)
    f -= k*v
    g -= k*u
    if 0 < f > T or 0 < g > T:
        break
    plt.plot(f, g, 'ro')
asd, dsa = predict(rob1[0], rob1[1], rob1[2], rob1[3], g)
asdd, dsaa = predict(rob2[0], rob2[1], rob2[2], rob2[3], f)
print asd[0, 0], asd[2, 0], '\n', asdd[0, 0], asdd[2, 0]
CS = plt.contour(A, B, val)
plt.clabel(CS, inline=1, fontsize=10)
plt.title('Simplest default with labels')
plt.show()
