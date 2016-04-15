import numpy as np


class Kalman(object):

    def __init__(self, sigma_meas, sigma_X, sigma_Z, theta):
        """

        :param sigma_meas: standard deviation of measurement noise. sqrt(2) times the gauss radius
        (where the function has decreased by a factor exp(-1)) of the control
        :param sigma_X: standard deviation of translational noise relative to 1 (m/s for example). sqrt(2) times
        the gauss radius (where the function has decreased by a factor exp(-1)) of the control
        :param sigma_Z: standard deviation of rotational noise relative to 1 (rad/s for example). sqrt(2) times
        the gauss radius (where the function has decreased by a factor exp(-1)) of the control
        :return: A first estimation of the covariance matrix, P_0_0
        """
        self.p = 0.1*np.eye(4)
        self.Q = np.zeros((4, 4))
        self.sigma_meas = sigma_meas
        self.sigma_X = sigma_X
        self.sigma_Z = sigma_Z
        self.theta = theta
        for i in range(0, 30):
            a, b = self.correct(np.array([[0], [0]]), 0, np.random.normal(0, self.sigma_meas, (2, 1)),
                                0, 1, 0.5, self.get_noise(0, 0, 1, 0.5))


    def get_noise(self, theta, X, Z, dt):
        """

        :param theta: orientation before control was applied relative to grid (theta = 0 means the robot is facing
        in +x direction)
        :param Z: anti-clockwise/clockwise motion control
        :param dt: time step between last and this control input
        :return: the control noise standard deviation matrix based on the control input
        """
        x_w1 = X*(np.sin(theta+Z*dt)-np.sin(theta))/Z
        x_w2 = Z*(np.cos(theta+Z*dt)*dt-(np.sin(theta+Z*dt)-np.sin(theta))/Z)*X/Z
        xdot_w1 = X*np.cos(theta)
        y_w1 = X*(np.cos(theta)-np.cos(theta+Z*dt))/Z
        y_w2 = Z*(np.sin(theta+Z*dt)*dt+(np.cos(theta+Z*dt)-np.cos(theta))/Z)*X/Z
        ydot_w1 = X*np.sin(theta)
        L = np.array([[x_w1, x_w2, 0, 0],
                      [xdot_w1, 0, 0, 0],
                      [y_w1, y_w2, 0, 0],
                      [ydot_w1, 0, 0, 0]])

        Q = np.array([[self.sigma_X**2, 0, 0, 0],
                      [0, self.sigma_Z**2, 0, 0],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0]])
        return np.dot(np.dot(L, Q), L.T)

    def predict(self, pos, theta, X, Z, dt):
        """
        This method can be used to predict the robots position after dt seconds solely based on
        the controls. The control error increases when using this method as no correction is made.

        :param pos: position vector [[x], [y]] before control was applied
        :param theta: orientation before control was applied relative to grid (theta = 0 means the robot is facing
        in +x direction)
        :param X: forward/backward motion control
        :param Z: anti-clockwise/clockwise motion control
        :param dt: time step between last and this control input
        :return: A prediction of the current position based on the control input and last position position
        """
        if Z > 1e-40:
            x_k_k1 = np.array([[pos[0]+(np.sin(theta+Z*dt)-np.sin(theta))*X/Z],
                               [X*np.cos(theta)],
                               [pos[1]+(np.cos(theta)-np.cos(theta+Z*dt))*X/Z],
                               [X*np.sin(theta)]])  # predicted state
        else:
            x_k_k1 = np.array([[pos[0]+X*np.cos(theta)*dt],
                               [X*np.cos(theta)],
                               [pos[1]+X*np.sin(theta)*dt],
                               [X*np.sin(theta)]])  # predicted state
        theta += Z*dt
        self.Q += self.get_noise(theta, X, Z, dt)
        return x_k_k1, theta

    def correct(self, pos, theta, pos_meas, X, Z, dt, Q):
        """

        :param pos: position vector [[x], [y]] before control was applied
        :param theta: orientation before control was applied relative to grid (theta = 0 means the robot is facing
        in +x direction)
        :param pos_meas: measured position
        :param X: forward/backward motion control
        :param sigma_X: sqrt(2) times the gauss radius (there the function has decreased by a factor exp(-1))
        of the control
        :param Z: anti-clockwise/clockwise motion control
        :param dt: time step between last and this control input
        :param Q: control noise standard deviation matrix
        :return: the (hopefully) best estimation of the state ([[x], [x_dot], [y], [y_dot]]) given the error in
        measurements and control
        """
        if Z > 1e-40:
            x_k_k1 = np.array([[pos[0]+(np.sin(theta+Z*dt)-np.sin(theta))*X/Z],
                               [X*np.cos(theta)],
                               [pos[1]+(np.cos(theta)-np.cos(theta+Z*dt))*X/Z],
                               [X*np.sin(theta)]])  # predicted state
        else:
            x_k_k1 = np.array([[pos[0]+X*np.cos(theta)*dt],
                               [X*np.cos(theta)],
                               [pos[1]+X*np.sin(theta)*dt],
                               [X*np.sin(theta)]])  # predicted state
        self.Q += self.get_noise(theta, X, Z, dt)  # control noise standard deviation
        R = self.sigma_meas**2*np.array([[1, 0, 0, 0],
                                         [0, 1, 0, 0],
                                         [0, 0, 1, 0],
                                         [0, 0, 0, 1]])  # measurement noise standard deviation
        F = np.array([[1, np.sin(Z*dt)/Z, 0, (np.cos(Z*dt)-1)/Z],
                      [0, 1, 0, 0],
                      [0, (1-np.cos(Z*dt))/Z, 1, np.sin(Z*dt)/Z],
                      [0, 0, 0, 1]])
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
        theta += Z*dt
        self.Q = np.zeros((4, 4))  # reset control noise after correction
        return x_k_k, theta
