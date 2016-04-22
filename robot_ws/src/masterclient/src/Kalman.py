import numpy as np


class Kalman(object):

    def __init__(self, sigma_meas, sigma_x, sigma_z, time_step):
        """

        :param sigma_meas: standard deviation of measurement noise. sqrt(2) times the gauss radius
        (where the function has decreased by a factor exp(-1)) of the control
        :param sigma_x: standard deviation of translational noise relative to 1 (m/s for example). sqrt(2) times
        the gauss radius (where the function has decreased by a factor exp(-1)) of the control
        :param sigma_z: standard deviation of rotational noise relative to 1 (rad/s for example). sqrt(2) times
        the gauss radius (where the function has decreased by a factor exp(-1)) of the control
        """
        self.p = 0.1*np.eye(4)
        self.Q = np.zeros((4, 4))
        self.std_meas = sigma_meas
        self.std_dev_x = sigma_x
        self.std_dev_z = sigma_z
        for i in range(0, 30):
            self.correct(np.array([0, 0]), 0, np.random.normal(0, self.std_meas, 2), 0, 1, time_step)

    def get_noise(self, theta, x, z, time_step):
        """

        :param theta: orientation before control was applied relative to grid (theta = 0 means the robot is facing
        in +x direction)
        :param x: forward/backward motion control
        :param z: anti-clockwise/clockwise motion control
        :param time_step: time to execute controls during
        :return: the control noise standard deviation matrix based on the control input
        """
        if z > 1e-40:
            x_w1 = x*(np.sin(theta+z*time_step)-np.sin(theta))/z
            x_w2 = z*(np.cos(theta+z*time_step)*time_step-(np.sin(theta+z*time_step)-np.sin(theta))/z)*x/z
            x_dot_w1 = x*np.cos(theta)
            y_w1 = x*(np.cos(theta)-np.cos(theta+z*time_step))/z
            y_w2 = z*(np.sin(theta+z*time_step)*time_step+(np.cos(theta+z*time_step)-np.cos(theta))/z)*x/z
            y_dot_w1 = x*np.sin(theta)
        else:
            x_w1 = x*np.cos(theta)*time_step
            x_w2 = 0
            x_dot_w1 = x*np.cos(theta)
            y_w1 = x*np.sin(theta)*time_step
            y_w2 = 0
            y_dot_w1 = x*np.sin(theta)
        L = np.array([[x_w1, x_w2, 0, 0],
                      [x_dot_w1, 0, 0, 0],
                      [y_w1, y_w2, 0, 0],
                      [y_dot_w1, 0, 0, 0]])

        Q = np.array([[self.std_dev_x**2, 0, 0, 0],
                      [0, self.std_dev_z**2, 0, 0],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0]])
        return np.dot(np.dot(L, Q), L.T)

    def predict(self, pos, theta, x, z, time_step):
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
        if z > 1e-40:
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
        self.Q += self.get_noise(theta, x, z, time_step)
        return x_k_k1, theta

    def correct(self, pos, theta, pos_meas, x, z, time_step):
        """

        :param pos: position vector [[x], [y]] before control was applied
        :param theta: orientation before control was applied relative to grid (theta = 0 means the robot is facing
        in +x direction)
        :param pos_meas: measured position
        :param x: forward/backward motion control
        :param z: anti-clockwise/clockwise motion control
        :param time_step: time to execute controls during
        :return: the (hopefully) best estimation of the state ([[x], [x_dot], [y], [y_dot]]) given the error in
        measurements and control
        """
        if z > 1e-40:
            x_k_k1 = np.array([[pos[0]+(np.sin(theta+z*time_step)-np.sin(theta))*x/z],
                               [x*np.cos(theta)],
                               [pos[1]+(np.cos(theta)-np.cos(theta+z*time_step))*x/z],
                               [x*np.sin(theta)]])  # predicted state
            F = np.array([[1, np.sin(z*time_step)/z, 0, (np.cos(z*time_step)-1)/z],
                          [0, 1, 0, 0],
                          [0, (1-np.cos(z*time_step))/z, 1, np.sin(z*time_step)/z],
                          [0, 0, 0, 1]])
        else:
            x_k_k1 = np.array([[pos[0]+x*np.cos(theta)*time_step],
                               [x*np.cos(theta)],
                               [pos[1]+x*np.sin(theta)*time_step],
                               [x*np.sin(theta)]])  # predicted state
            F = np.array([[1, time_step, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, time_step],
                          [0, 0, 0, 1]])
        self.Q += self.get_noise(theta, x, z)  # control noise standard deviation
        R = self.std_meas**2*np.array([[1, 0, 0, 0],
                                      [0, 1, 0, 0],
                                      [0, 0, 1, 0],
                                      [0, 0, 0, 1]])  # measurement noise standard deviation
        H = np.array([[1, 0, 0, 0],
                      [0, 0, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 0]])  # observation matrix
        z_k = np.array([[pos_meas[0]],
                        [0],
                        [pos_meas[1]],
                        [0]])  # measured state
        y_k = z_k - np.dot(H, x_k_k1)  # residual
        p_k_k1 = np.dot(np.dot(F, self.p), F.T) + self.Q  # predicted current covariance
        S = np.linalg.solve(np.dot(np.dot(H, p_k_k1), H.T) + R, np.eye(4))
        K = np.dot(np.dot(p_k_k1, H.T), S)  # Kalman gain
        x_k_k = x_k_k1 + np.dot(K, y_k)  # current state
        self.p = np.dot(np.eye(4) - np.dot(K, H), p_k_k1)  # current covariance
        theta += z*time_step
        self.Q = np.zeros((4, 4))  # reset control noise after correction
        return x_k_k, theta

    def set_sigma_meas(self, val):
        self.std_meas = val

    def set_sigma_x(self, val):
        self.std_dev_x = val

    def set_sigma_z(self, val):
        self.std_dev_z = val

    def get_sigma_meas(self):
        return self.std_meas

    def get_sigma_x(self):
        return self.std_dev_x

    def get_sigma_z(self):
        return self.std_dev_z

