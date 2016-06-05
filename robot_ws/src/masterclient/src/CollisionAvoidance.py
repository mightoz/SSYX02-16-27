import numpy as np
import time

"""
Send position, orientation and controls and predict BEFORE applying controls
to see if there might be any collision
TODO: apply gradient descent. if reltol is reached then there should not be
any collision. reltol should be quite small. if abstol is reached, there may
be a collision. abstol should be quite large.

TODO: add a function that checks gradient descent for each pair of robots.
"""


def __tar_fun_lin_lin__(pos_1, theta_1, x_1, pos_2, theta_2, x_2, t_2, t_1):
    """
    robot 1 has linear motion, robot 2 has linear motion
    
    :param pos_1: Position of robot 1 [x, y] (m)
    :param theta_1: Orientation of robot 1 (rad)
    :param x_1: velocity of robot 1 (m/s)
    :param pos_2: Position of robot 2 [x, y] (m)
    :param theta_2: Orientation of robot 2 (in radians)
    :param x_2: velocity of robot 2 (m/s)
    :param t_2: Time instant where robot 2 is (s)
    :param t_1: Time instant where robot 1 is (s)
    :return: Value of target function
    """
    result = ((pos_1-pos_2)[0]+x_1*np.cos(theta_1)*t_1 -
              x_2*np.cos(theta_2)*t_2)** 2 + \
             ((pos_1-pos_2)[1]+x_1*np.sin(theta_1)*t_1 -
              x_2*np.sin(theta_2)*t_2)** 2
    return result


def __tar_grad_lin_lin__(pos_1, theta_1, x_1, pos_2, theta_2, x_2, t_2, t_1):
    """
    robot 1 has linear motion, robot 2 has linear motion
    
    :param pos_1: Position of robot 1 [x, y] (m)
    :param theta_1: Orientation of robot 1 (rad)
    :param x_1: velocity of robot 1 (m/s)
    :param pos_2: Position of robot 2 [x, y] (m)
    :param theta_2: Orientation of robot 2 (in radians)
    :param x_2: velocity of robot 2 (m/s)
    :param t_2: Time instant where robot 2 is (s)
    :param t_1: Time instant where robot 1 is (s)
    :return: Gradient of target function (with the same name): t_2, t_1
    """
    grad_t_2 = -2*x_2*np.cos(theta_2)*((pos_1-pos_2)[0] +
                                       x_1*np.cos(theta_1)*t_1-x_2 *
                                       np.cos(theta_2)*t_2) - \
        2*x_2*np.sin(theta_2)*((pos_1-pos_2)[1] +
                               x_1 * np.sin(theta_1)*t_1-x_2 *
                               np.sin(theta_2)*t_2)
    grad_t_1 = 2*x_1*np.cos(theta_1)*((pos_1-pos_2)[0] +
                                      x_1*np.cos(theta_1)*t_1-x_2 *
                                      np.cos(theta_2)*t_2) + \
        2*x_1*np.sin(theta_1)*((pos_1-pos_2)[1] +
                               x_1 * np.sin(theta_1)*t_1-x_2 *
                               np.sin(theta_2)*t_2)
    return grad_t_2, grad_t_1


def __tar_fun_rot_rot__(pos_1, theta_1, x_1, z_1,
                        pos_2, theta_2, x_2, z_2, t_2, t_1):
    """
    robot 1 has rotatig motion, robot 2 has rotating motion
    
    :param pos_1: Position of robot 1 [x, y] (m)
    :param theta_1: Orientation of robot 1 (rad)
    :param x_1: velocity of robot 1 (m/s)
    :param z_1: rotation velocity of robot 1 (rad/s)
    :param pos_2: Position of robot 2 [x, y] (m)
    :param theta_2: Orientation of robot 2 (in radians)
    :param x_2: velocity of robot 2 (m/s)
    :param z_2: rotation velocity of robot 2 (rad/s)
    :param t_2: Time instant where robot 2 is (s)
    :param t_1: Time instant where robot 1 is (s)
    :return: Value of target function
    """
    result = ((pos_1-pos_2)[0]+x_1/z_1*(np.sin(theta_1+z_1*t_1) -
                                        np.sin(theta_1)) -
              x_2/z_2*(np.sin(theta_2+z_2*t_2)-np.sin(theta_2)))**2 +\
             ((pos_1-pos_2)[1]+x_1/z_1*(np.cos(theta_1) -
                                        np.cos(theta_1+z_1*t_1)) -
              x_2/z_2*(np.cos(theta_2)-np.cos(theta_2+z_2*t_2)))**2
    return result


def __tar_grad_rot_rot__(pos_1, theta_1, x_1, z_1,
                         pos_2, theta_2, x_2, z_2, t_2, t_1):
    """
    robot 1 has rotating motion, robot 2 has rotating motion
    
    :param pos_1: Position of robot 1 [x, y] (m)
    :param theta_1: Orientation of robot 1 (rad)
    :param x_1: velocity of robot 1 (m/s)
    :param z_1: rotation velocity of robot 1 (rad/s)
    :param pos_2: Position of robot 2 [x, y] (m)
    :param theta_2: Orientation of robot 2 (in radians)
    :param x_2: velocity of robot 2 (m/s)
    :param z_2: rotation velocity of robot 2 (rad/s)
    :param t_2: Time instant where robot 2 is (s)
    :param t_1: Time instant where robot 1 is (s)
    :return: Gradient of target function (with the same name): t_2, t_1
    """
    grad_t_2 = -2*x_2*np.cos(theta_2+z_2*t_2) *\
               ((pos_1-pos_2)[0]+x_1/z_1*(np.sin(theta_1+z_1*t_1) -
                                          np.sin(theta_1)) -
                x_2/z_2*(np.sin(theta_2+z_2*t_2)-np.sin(theta_2))) -\
               2*x_2*np.sin(theta_2+z_2*t_2) *\
               ((pos_1-pos_2)[1]+x_1/z_1*(np.cos(theta_1) -
                                          np.cos(theta_1 + z_1 * t_1)) -
                x_2 / z_2 * (np.cos(theta_2) - np.cos(theta_2 + z_2 * t_2)))
    grad_t_1 = 2 * x_1 * np.cos(theta_1 + z_1 * t_1) *\
               ((pos_1-pos_2)[0]+x_1/z_1*(np.sin(theta_1+z_1*t_1) -
                                          np.sin(theta_1)) -
                x_2/z_2*(np.sin(theta_2+z_2*t_2)-np.sin(theta_2))) + \
               2*x_1*np.sin(theta_1+z_1*t_1) *\
               ((pos_1-pos_2)[1]+x_1/z_1*(np.cos(theta_1) -
                                          np.cos(theta_1+z_1*t_1)) -
                x_2/z_2*(np.cos(theta_2)-np.cos(theta_2+z_2*t_2)))
    return grad_t_2, grad_t_1


def __tar_fun_lin_rot__(pos_1, theta_1, x_1,
                        pos_2, theta_2, x_2, z_2, t_2, t_1):
    """
    robot 1 has linear motion, robot 2 has rotating motion
    
    :param pos_1: Position of robot 1 [x, y] (m)
    :param theta_1: Orientation of robot 1 (rad)
    :param x_1: velocity of robot 1 (m/s)
    :param pos_2: Position of robot 2 [x, y] (m)
    :param theta_2: Orientation of robot 2 (in radians)
    :param x_2: velocity of robot 2 (m/s)
    :param z_2: rotation velocity of robot 2 (rad/s)
    :param t_2: Time instant where robot 2 is (s)
    :param t_1: Time instant where robot 1 is (s)
    :return: Value of target function
    """
    result = ((pos_1 - pos_2)[0] + x_1 * np.cos(theta_1) * t_1 - x_2 / z_2 *
              (np.sin(theta_2 + z_2 * t_2) - np.sin(theta_2)))**2 + \
             ((pos_1 - pos_2)[1] + x_1 * np.sin(theta_1) * t_1 - x_2 / z_2 *
              (np.cos(theta_2) - np.cos(theta_2 + z_2 * t_2)))**2
    return result


def __tar_grad_lin_rot__(pos_1, theta_1, x_1,
                         pos_2, theta_2, x_2, z_2, t_2, t_1):
    """
    robot 1 has linear motion, robot 2 has rotating motion
    
    :param pos_1: Position of robot 1 [x, y] (m)
    :param theta_1: Orientation of robot 1 (rad)
    :param x_1: velocity of robot 1 (m/s)
    :param pos_2: Position of robot 2 [x, y] (m)
    :param theta_2: Orientation of robot 2 (in radians)
    :param x_2: velocity of robot 2 (m/s)
    :param z_2: rotation velocity of robot 2 (rad/s)
    :param t_2: Time instant where robot 2 is (s)
    :param t_1: Time instant where robot 1 is (s)
    :return: Gradient of target function (with the same name): t_2, t_1
    """
    grad_t_2 = -2*x_2*np.cos(theta_2+z_2*t_2) *\
               ((pos_1-pos_2)[0]+x_1*np.cos(theta_1)*t_1 -
                x_2/z_2*(np.sin(theta_2+z_2*t_2)-np.sin(theta_2))) -\
               2*x_2*np.sin(theta_2+z_2*t_2) *\
               ((pos_1-pos_2)[1]+x_1*np.sin(theta_1)*t_1 -
                x_2/z_2*(np.cos(theta_2)-np.cos(theta_2+z_2*t_2)))
    grad_t_1 = 2*x_1*np.cos(theta_1)*((pos_1-pos_2)[0] +
                                      x_1*np.cos(theta_1)*t_1 -
                                      x_2/z_2*(np.sin(theta_2+z_2*t_2) -
                                               np.sin(theta_2))) +\
               2*x_1*np.sin(theta_1)*((pos_1-pos_2)[1] +
                                      x_1*np.sin(theta_1)*t_1 -
                                      x_2/z_2*(np.cos(theta_2) -
                                               np.cos(theta_2+z_2*t_2)))
    return grad_t_2, grad_t_1


def __tar_fun_rot_lin__(pos_1, theta_1, x_1, z_1,
                        pos_2, theta_2, x_2, t_2, t_1):
    """
    robot 1 has rotating motion, robot 2 has linear motion
    
    :param pos_1: Position of robot 1 [x, y] (m)
    :param theta_1: Orientation of robot 1 (rad)
    :param x_1: velocity of robot 1 (m/s)
    :param z_1: rotation velocity of robot 1 (rad/s)
    :param pos_2: Position of robot 2 [x, y] (m)
    :param theta_2: Orientation of robot 2 (in radians)
    :param x_2: velocity of robot 2 (m/s)
    :param t_2: Time instant where robot 2 is (s)
    :param t_1: Time instant where robot 1 is (s)
    :return: Value of target function
    """
    result = ((pos_2-pos_1)[0]+x_2*np.cos(theta_2)*t_2-x_1/z_1 *
              (np.sin(theta_1+z_1*t_1)-np.sin(theta_1)))**2 +\
             ((pos_2-pos_1)[1]+x_2*np.sin(theta_2)*t_2-x_1/z_1 *
              (np.cos(theta_1)-np.cos(theta_1+z_1*t_1)))**2
    return result


def __tar_grad_rot_lin__(pos_1, theta_1, x_1, z_1,
                         pos_2, theta_2, x_2, t_2, t_1):
    """
    robot 1 has rotating motion, robot 2 has linear motion
    
    :param pos_1: Position of robot 1 [x, y] (m)
    :param theta_1: Orientation of robot 1 (rad)
    :param x_1: velocity of robot 1 (m/s)
    :param z_1: rotation velocity of robot 1 (rad/s)
    :param pos_2: Position of robot 2 [x, y] (m)
    :param theta_2: Orientation of robot 2 (in radians)
    :param x_2: velocity of robot 2 (m/s)
    :param t_2: Time instant where robot 2 is (s)
    :param t_1: Time instant where robot 1 is (s)
    :return: Gradient of target function (with the same name): t_2, t_1
    """
    grad_t_2 = 2*x_2*np.cos(theta_2)*((pos_2-pos_1)[0] +
                                      x_2*np.cos(theta_2)*t_2 -
                                      x_1/z_1*(np.sin(theta_1+z_1*t_1) -
                                               np.sin(theta_1))) +\
               2*x_2*np.sin(theta_2)*((pos_2-pos_1)[1] +
                                      x_2*np.sin(theta_2)*t_2 -
                                      x_1/z_1*(np.cos(theta_1) -
                                               np.cos(theta_1+z_1*t_1)))
    grad_t_1 = -2*x_1*np.cos(theta_1+z_1*t_1) *\
               ((pos_2-pos_1)[0]+x_2*np.cos(theta_2)*t_2 -
                x_1/z_1*(np.sin(theta_1+z_1*t_1)-np.sin(theta_1))) -\
               2*x_1*np.sin(theta_1+z_1*t_1) *\
               ((pos_2-pos_1)[1]+x_2*np.sin(theta_2)*t_2 -
                x_1/z_1*(np.cos(theta_1)-np.cos(theta_1+z_1*t_1)))
    return grad_t_2, grad_t_1


class CollisionAvoidance(object):

    def __init__(self, abs_dist_tol, rel_dist_tol):
        self.abs_dist_tol = abs_dist_tol
        self.rel_dist_tol = rel_dist_tol
        self.k = 0.5
        self.calls = 0

    def gradient_descent(self, pos_1, theta_1, x_1, z_1,
                         pos_2, theta_2, x_2, z_2, time_exec, ret_on_col):
        """
        This function calculates wether or not a collision may occur for the
        given pair of robots within the given execution time and when it will
        occur for each robot.
        Known bugs: If the collision occurs outside the execution time (meaning
        it would have occured in the past or too far into the future), there is
        a posibility that the collision detection can trigger slightly too far
        into the future or past. For low velocities this should not be
        noticable.
        
        :param pos_1: Position of robot 1 [x, y] (m)
        :param theta_1: Orientation of robot 1 (rad)
        :param x_1: velocity of robot 1 (m/s)
        :param z_1: rotation velocity of robot 1 (rad/s)
        :param pos_2: Position of robot 2 [x, y] (m)
        :param theta_2: Orientation of robot 2 (in radians)
        :param x_2: velocity of robot 2 (m/s)
        :param z_2: rotation velocity of robot 2 (rad/s)
        :param time_exec: Duration of iteration (s)
        :param ret_on_col: Return on collision, True/False
        :return: wether or not if a collision occurs and the respective
        times for the robots when they occur: collide, t_1, t_2
        """
        t_2 = time_exec/2.0
        t_1 = time_exec/2.0
        collide = False
        last_residual = 2e2
        residual = 1e2
        if np.abs(z_1) > 1e-40:
            if np.abs(z_2) > 1e-40:  # rob1 rot rob2 rot
                bfr = time.time()
                while np.abs(last_residual-residual) > self.rel_dist_tol**2:
                    last_residual = residual
                    dt_2, dt_1 = __tar_grad_rot_rot__(pos_1, theta_1, x_1, z_1,
                                                      pos_2, theta_2, x_2, z_2,
                                                      t_2, t_1)
                    t_2 -= self.k*dt_2
                    t_1 -= self.k*dt_1
                    residual = __tar_fun_rot_rot__(pos_1, theta_1, x_1, z_1,
                                                   pos_2, theta_2, x_2, z_2,
                                                   t_2, t_1)
                    if np.abs(residual) < self.abs_dist_tol**2 and ret_on_col:
                        collide = True
                        break
                    if (0 > t_2 or t_2 > time_exec or
                        0 > t_1 or t_1 > time_exec):
                        break
                    if time.time() - bfr > 0.1:
                        print 'calculation timed out', t_1, t_2
                        break
            else:  # rob1 rot rob2 lin
                bfr = time.time()
                while np.abs(last_residual-residual) > self.rel_dist_tol**2:
                    last_residual = residual
                    dt_2, dt_1 = __tar_grad_rot_lin__(pos_1, theta_1, x_1, z_1,
                                                      pos_2, theta_2, x_2,
                                                      t_2, t_1)
                    t_2 -= self.k*dt_2
                    t_1 -= self.k*dt_1
                    residual = __tar_fun_rot_lin__(pos_1, theta_1, x_1, z_1,
                                                   pos_2, theta_2, x_2,
                                                   t_2, t_1)
                    if np.abs(residual) < self.abs_dist_tol**2 and ret_on_col:
                        collide = True
                        break
                    if (0 > t_2 or t_2 > time_exec or
                        0 > t_1 or t_1 > time_exec):
                        break
                    if time.time() - bfr > 0.1:
                        print 'calculation timed out', t_1, t_2
                        break
        else:
            if np.abs(z_2) > 1e-40:  # rob1 lin rob2 rot
                bfr = time.time()
                while np.abs(last_residual-residual) > self.rel_dist_tol**2:
                    last_residual = residual
                    dt_2, dt_1 = __tar_grad_lin_rot__(pos_1, theta_1, x_1,
                                                      pos_2, theta_2, x_2, z_2,
                                                      t_2, t_1)
                    t_2 -= self.k*dt_2
                    t_1 -= self.k*dt_1
                    residual = __tar_fun_lin_rot__(pos_1, theta_1, x_1,
                                                   pos_2, theta_2, x_2, z_2,
                                                   t_2, t_1)
                    if np.abs(residual) < self.abs_dist_tol**2 and ret_on_col:
                        collide = True
                        break
                    if (0 > t_2 or t_2 > time_exec or
                        0 > t_1 or t_1 > time_exec):
                        break
                    if time.time() - bfr > 0.1:
                        print 'calculation timed out', t_1, t_2
                        break
            else:  # rob1 lin rob2 lin
                bfr = time.time()
                while np.abs(last_residual-residual) > self.rel_dist_tol**2:
                    last_residual = residual
                    dt_2, dt_1 = __tar_grad_lin_lin__(pos_1, theta_1, x_1,
                                                      pos_2, theta_2, x_2,
                                                      t_2, t_1)
                    t_2 -= self.k*dt_2
                    t_1 -= self.k*dt_1
                    residual = __tar_fun_lin_lin__(pos_1, theta_1, x_1,
                                                   pos_2, theta_2, x_2,
                                                   t_2, t_1)
                    if np.abs(residual) < self.abs_dist_tol**2 and ret_on_col:
                        collide = True
                        break
                    if (0 > t_2 or t_2 > time_exec or
                        0 > t_1 or t_1 > time_exec):
                        break
                    if time.time() - bfr > 0.1:
                        print 'calculation timed out', t_1, t_2
                        break
        return collide, t_1, t_2

    def calc_new_controls(self, pos_1, theta_1, x_1, z_1,
                          pos_2, theta_2, x_2, z_2, time_exec):
        """
        Attempts to detect collisions and calculates new controls to
        avoidthis. This collision avoidance is simple and will only
        stop robots if a collision is detected.
        
        :param pos_1: Position of robot 1 [x, y] (m)
        :param theta_1: Orientation of robot 1 (rad)
        :param x_1: velocity of robot 1 (m/s)
        :param z_1: rotation velocity of robot 1 (rad/s)
        :param pos_2: Position of robot 2 [x, y] (m)
        :param theta_2: Orientation of robot 2 (in radians)
        :param x_2: velocity of robot 2 (m/s)
        :param z_2: rotation velocity of robot 2 (rad/s)
        :param time_exec: Duration of iteration (s)
        :return: new controls for the pair of robots that will avoid
        collision: new_x_1, new_z_1, new_x_2, new_z_2
        times for the robots when they occur: collide, t_1, t_2
        """
        self.calls += 1
        new_x_1 = x_1
        new_z_1 = z_1
        new_x_2 = x_2
        new_z_2 = z_2
        collide1, t1, t2 = self.gradient_descent(pos_1, theta_1, new_x_1,
                                                 new_z_1, pos_2, theta_2,
                                                 new_x_2, new_z_2,
                                                 time_exec, True)
        if collide1:
            print 'collide1: stopping robot2'
            new_x_2 = 0
            collide2, t1, t2 = self.gradient_descent(pos_1, theta_1, new_x_1,
                                                     new_z_1, pos_2, theta_2,
                                                     new_x_2, new_z_2,
                                                     time_exec, True)
            if collide2:
                print 'collide2: stopping robot1'
                new_x_1 = 0
                collide3, t1, t2 = self.gradient_descent(pos_1, theta_1,
                                                         new_x_1, new_z_1,
                                                         pos_2, theta_2,
                                                         new_x_2, new_z_2,
                                                         time_exec, True)
        return new_x_1, new_z_1, new_x_2, new_z_2
