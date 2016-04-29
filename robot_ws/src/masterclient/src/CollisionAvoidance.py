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
                        print 'res ', residual
                        asd, dsa = predict(pos_1, theta_1, x_1, z_1, t_1)
                        asdd, dsad = predict(pos_2, theta_2, x_2, z_2, t_2)
                        plt.plot(asd[0, 0], asd[2, 0], 'mo')
                        plt.plot(asdd[0, 0], asdd[2, 0], 'co')
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


k = 0.3
T = 3
c_a = CollisionAvoidance(0.5**2, 1e-2**2, k)
dsad = False
while not dsad:
    rob1 = [6*np.random.rand(2)-3, 2*np.pi*np.random.rand(), np.random.rand(), 2*np.random.rand()-1]
    rob2 = [6*np.random.rand(2)-3, 2*np.pi*np.random.rand(), np.random.rand(), 2*np.random.rand()-1]
    dsad = c_a.gradient_descent(rob1[0], rob1[1], rob1[2], rob1[3], rob2[0], rob2[1], rob2[2], rob2[3], T)
time = np.linspace(0, T, 101)
pos1 = np.zeros((2, 101))
pos2 = np.zeros((2, 101))
for i in range(0, len(time)):
    asd, dsa = predict(rob1[0], rob1[1], rob1[2], rob1[3], time[i])
    pos1[:, i] = np.array([asd[0, 0], asd[2, 0]])
    asdd, dsad = predict(rob2[0], rob2[1], rob2[2], rob2[3], time[i])
    pos2[:, i] = np.array([asdd[0, 0], asdd[2, 0]])
plt.plot(pos1[0, :], pos1[1, :], 'g')
plt.plot(pos2[0, :], pos2[1, :], 'b')
plt.figure()

a = np.linspace(-3, 3)
b = np.linspace(-3, 3)
A, B = np.meshgrid(a, b)
# val = __tar_fun_lin_lin__(rob1[0], rob1[1], rob1[2], rob2[0], rob2[1], rob2[2], A, B)
val = __tar_fun_rot_rot__(rob1[0], rob1[1], rob1[2], rob1[3], rob2[0], rob2[1], rob2[2], rob2[3], A, B)
# val = __tar_fun_lin_rot__(rob1[0], rob1[1], rob1[2], rob2[0], rob2[1], rob2[2], rob2[3], A, B)
# val = __tar_fun_rot_lin__(rob1[0], rob1[1], rob1[2], rob1[3], rob2[0], rob2[1], rob2[2], B, A)
f = 0
g = 0
plt.plot(f, g, 'ro')
for i in range(0, 300):
    # v, u = __tar_grad_lin_lin__(rob1[0], rob1[1], rob1[2], rob2[0], rob2[1], rob2[2], f, g)
    v, u = __tar_grad_rot_rot__(rob1[0], rob1[1], rob1[2], rob1[3], rob2[0], rob2[1], rob2[2], rob2[3], f, g)
    # v, u = __tar_grad_lin_rot__(rob1[0], rob1[1], rob1[2], rob2[0], rob2[1], rob2[2], rob2[3], f, g)
    # v, u = __tar_grad_rot_lin__(rob1[0], rob1[1], rob1[2], rob1[3], rob2[0], rob2[1], rob2[2], g, f)
    f -= k*v
    g -= k*u
    if 0 > f or f > T or 0 > g or g > T:
        print 'no collision'
        break
    plt.plot(f, g, 'ro')
asd, dsa = predict(rob1[0], rob1[1], rob1[2], rob1[3], g)
asdd, dsaa = predict(rob2[0], rob2[1], rob2[2], rob2[3], f)
print asd[0, 0], asd[2, 0], '\n', asdd[0, 0], asdd[2, 0]
CS = plt.contour(A, B, val, 100)
plt.clabel(CS, inline=1, fontsize=10)
plt.title('Simplest default with labels')
plt.show()
