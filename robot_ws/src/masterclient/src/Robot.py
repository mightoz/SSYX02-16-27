import numpy as np
import Kalman
import Controls


class Robot(object):

    def __init__(self, x, z, theta, pos):
        self.trans_vel = x
        self.ang_vel = z
        self.orientation = theta
        self.curr_pos = pos
        self.state = np. array([[pos[0]],
                                [0],
                                [pos[1]],
                                [0],
                                [theta],
                                [0]])
        self.__wheel_dist = 0.43
        self.kalman = Kalman.Kalman()
        self.controls = Controls.Controls()

    def set_kalman(self, sigma_meas, sigma_x, sigma_z):
        self.kalman.set_sigma_meas(sigma_meas)
        self.kalman.set_sigma_x(sigma_x)
        self.kalman.set_sigma_z(sigma_z)

    def set_controls(self, x_min, x_max, z_min, z_max, k, t_x, t_z, ok_dist):
        self.controls.set_x_min(x_min)
        self.controls.set_x_max(x_max)
        self.controls.set_z_min(z_min)
        self.controls.set_z_max(z_max)
        self.controls.set_k(k)
        self.controls.set_t_x(t_x)
        self.controls.set_t_z(t_z)
        self.controls.set_ok_dist(ok_dist)

    def set_x(self, val):
        self.trans_vel = val

    def set_z(self, val):
        self.ang_vel = val

    def set_state(self, state):
        self.state = state

    def get_x(self):
        return self.trans_vel

    def get_z(self):
        return self.ang_vel

    def get_state(self):
        return self.state

    def get_x_pos(self):
        return self.state[0, 0]

    def get_x_vel(self):
        return self.state[1, 0]

    def get_y_pos(self):
        return self.state[2, 0]

    def get_y_vel(self):
        return self.state[3, 0]

    def get_theta(self):
        return self.state[4, 0]

    def get_theta_vel(self):
        return self.state[5, 0]

    def get_pos(self):
        return np.array([self.get_x_pos(), self.get_y_pos()])

    def get_kalman(self):
        return self.kalman

    def get_controls(self):
        return self.controls
