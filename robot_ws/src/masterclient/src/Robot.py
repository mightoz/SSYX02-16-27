import numpy as np
import Kalman
import Controls


class Robot(object):

    def __init__(self, x, z, theta, pos):
        self.trans_vel = x
        self.ang_vel = z
        self.orientation = theta
        self.curr_pos = pos
        self.kalman = Kalman.Kalman(0.5, 0, 0, 0)
        self.controls = Controls.Controls(0, 0, 0, 0, 0, 2, 2, 0)

    def set_kalman(self, sigma_meas, sigma_x, sigma_z, dt):
        self.kalman.set_sigma_meas(sigma_meas)
        self.kalman.set_sigma_x(sigma_x)
        self.kalman.set_sigma_z(sigma_z)
        self.kalman.set_time_step(dt)

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

    def set_theta(self, val):
        self.orientation = val

    def set_pos(self, val):
        self.curr_pos = val

    def get_x(self):
        return self.trans_vel

    def get_z(self):
        return self.ang_vel

    def get_theta(self):
        return self.orientation

    def get_pos(self):
        return self.curr_pos

    def get_kalman(self):
        return self.kalman

    def get_controls(self):
        return self.controls
