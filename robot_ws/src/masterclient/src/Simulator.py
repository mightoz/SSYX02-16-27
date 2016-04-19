import numpy as np
import matplotlib.pyplot as plt
import Kalman
import Controls


def movenext(pos, theta, X, Z, dt):
    """
    Only here for simulation purposes.

    :param pos: current position vector [[x], [y]]
    :param theta: current orientation relative to grid (theta = 0 means the robot is facing in +x direction)
    :param X: forward/backward motion control
    :param Z: anti-clockwise/clockwise motion control
    :param dt: time step between this and the next control input
    :return: A prediction of the next position based on the control input and current position
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
    newtheta = theta + Z*dt
    return x_k_k1, newtheta

sigma_meas = 0.05
sigma_X = 0.05
sigma_Z = 0.025

iterations = 50
n_robots = 3
n_iter_no_corr = n_robots-1
"""
TODO: k (used in getcontrols) should be a function of n_iter_no_corr, X and Z.
n_iter_no_cor/20.0 works fine for now
"""
truepos = np.zeros((n_robots, 2, n_iter_no_corr*iterations))
truetheta = np.zeros((n_robots, n_iter_no_corr*iterations))
measpos = np.zeros((n_robots, 2, iterations)) + np.random.normal(0, sigma_meas, (n_robots, 2, iterations))
currpos = np.zeros((n_robots, 2, n_iter_no_corr*iterations))
currtheta = np.zeros((n_robots, n_iter_no_corr*iterations))

truepos[:, :, 0] = 10*np.random.rand(n_robots, 2)-5
truetheta[:, 0] = 2*np.pi*np.random.rand(n_robots)
measpos[:, :, 0] = truepos[:, :, 0]
currpos[:, :, 0] = truepos[:, :, 0]
currtheta[:, 0] = truetheta[:, 0]
"""
truepos[:, :, 0] = np.array([[-6, 7, 1],
                             [7, -2, -8]]).T
truetheta[:, 0] = 2*np.pi*np.random.rand(n_robots)
currpos[:, :, 0] = truepos[:, :, 0]
currtheta[:, 0] = truetheta[:, 0]
"""

kal = [Kalman.Kalman(sigma_meas, sigma_X, sigma_Z, currtheta[0, 0]),
       Kalman.Kalman(sigma_meas, sigma_X, sigma_Z, currtheta[1, 0]),
       Kalman.Kalman(sigma_meas, sigma_X, sigma_Z, currtheta[2, 0])]
ctrl = [Controls.Controls(0.05, 1, 0, 1),
        Controls.Controls(0.05, 1, 0, 1),
        Controls.Controls(0.05, 1, 0, 1)]

basepos = 10*np.random.rand(2)-5
endnodepos = 10*np.random.rand(2)-5
"""
basepos = np.array([8, 7])
endnodepos = np.array([-7, -8])
"""
X = np.zeros((n_robots, n_iter_no_corr*iterations))
Z = np.zeros((n_robots, n_iter_no_corr*iterations))

for j in range(0, iterations):
    controlnoiset = np.random.normal(0, sigma_X)
    controlnoiser = np.random.normal(0, sigma_Z)
    corr_idx = np.mod(j, n_robots)
    for m in range(0, n_robots):
        x1, v1 = movenext(truepos[m, :, n_iter_no_corr*j-1], truetheta[m, n_iter_no_corr*j-1],
                          X[m, n_iter_no_corr*j-1]*(1+controlnoiset),
                          Z[m, n_iter_no_corr*j-1]*(1+controlnoiser), 0.5)
        truetheta[m, n_iter_no_corr*j] += v1
        truepos[m, :, n_iter_no_corr*j] += np.array([x1[0, 0], x1[2, 0]])
        if m != corr_idx:
            x3, v3 = kal[m].predict(currpos[m, :, n_iter_no_corr*j-1], currtheta[m, n_iter_no_corr*j-1],
                                    X[m, n_iter_no_corr*j-1], Z[m, n_iter_no_corr*j-1], 0.5)
            currtheta[m, n_iter_no_corr*j] += v3
            currpos[m, :, n_iter_no_corr*j] += np.array([x3[0, 0], x3[2, 0]])
        else:
            x3, v3 = kal[m].correct(currpos[m, :, n_iter_no_corr*j-1],
                                           currtheta[corr_idx, n_iter_no_corr*j-1],  measpos[corr_idx, :, j],
                                           X[corr_idx, n_iter_no_corr*j-1],
                                           Z[corr_idx, n_iter_no_corr*j-1], 0.5)
            currtheta[corr_idx, n_iter_no_corr*j] = v3
            currpos[corr_idx, :, n_iter_no_corr*j] = np.array([x3[0, 0], x3[2, 0]])
    for m in range(0, n_robots):
        if m != 0 and m != n_robots-1:
            X[m, n_iter_no_corr*j], Z[m, n_iter_no_corr*j] = \
                ctrl[m].get_controls(currtheta[m, n_iter_no_corr*j], currpos[m, :, n_iter_no_corr*j],
                                     currpos[m-1, :, n_iter_no_corr*j], currpos[m+1, :, n_iter_no_corr*j],
                                     n_iter_no_corr/20.0, 2, 2)
        elif m == 0:
            X[m, n_iter_no_corr*j], Z[m, n_iter_no_corr*j] = \
                ctrl[m].get_controls(currtheta[m, n_iter_no_corr*j], currpos[m, :, n_iter_no_corr*j],
                                     basepos, currpos[m+1, :, n_iter_no_corr*j],
                                     n_iter_no_corr/20.0, 2, 2)
        elif m == n_robots-1:
            X[m, n_iter_no_corr*j], Z[m, n_iter_no_corr*j] = \
                ctrl[m].get_controls(currtheta[m, n_iter_no_corr*j], currpos[m, :, n_iter_no_corr*j],
                                     currpos[m-1, :, n_iter_no_corr*j], endnodepos,
                                     n_iter_no_corr/20.0, 2, 2)



"""
    if j > 0:
        controlnoiset = np.random.normal(0, sigma_X)
        controlnoiser = np.random.normal(0, sigma_Z)
        for m in range(0, n_robots):
            x1, v1 = movenext(truepos[m, :, n_iter_no_corr*j-1], truetheta[m, n_iter_no_corr*j-1], X[m, n_iter_no_corr*j-1]*(1+controlnoiset), Z[m, n_iter_no_corr*j-1]*(1+controlnoiser), 0.5)
            truetheta[m, n_iter_no_corr*j] += v1
            truepos[m, :, n_iter_no_corr*j] += np.array([x1[0, 0], x1[2, 0]])
            measpos[m, :, j] += truepos[m, :, n_iter_no_corr*j]
            x3, v3 = kal[m].correct(currpos[m, :, n_iter_no_corr*j-1], currtheta[m, n_iter_no_corr*j-1],  measpos[m, :, j],
                                    X[m, n_iter_no_corr*j-1], Z[m, n_iter_no_corr*j-1], 0.5)
            currtheta[m, n_iter_no_corr*j] = v3
            currpos[m, :, n_iter_no_corr*j] = np.array([x3[0, 0], x3[2, 0]])
        for m in range(0, n_robots):
            if m != 0 and m != n_robots-1:
                X[m, n_iter_no_corr*j], Z[m, n_iter_no_corr*j] = ctrl[m].get_controls(currtheta[m, n_iter_no_corr*j], currpos[m, :, n_iter_no_corr*j],
                                                                                      currpos[m-1, :, n_iter_no_corr*j], currpos[m+1, :, n_iter_no_corr*j],
                                                                                      n_iter_no_corr/20.0, 2, 2)
            elif m == 0:
                X[m, n_iter_no_corr*j], Z[m, n_iter_no_corr*j] = ctrl[m].get_controls(currtheta[m, n_iter_no_corr*j], currpos[m, :, n_iter_no_corr*j],
                                                                                      basepos, currpos[m+1, :, n_iter_no_corr*j],
                                                                                      n_iter_no_corr/20.0, 2, 2)
            elif m == n_robots-1:
                X[m, n_iter_no_corr*j], Z[m, n_iter_no_corr*j] = ctrl[m].get_controls(currtheta[m, n_iter_no_corr*j], currpos[m, :, n_iter_no_corr*j],
                                                                                      currpos[m-1, :, n_iter_no_corr*j], endnodepos,
                                                                                      n_iter_no_corr/20.0, 2, 2)

    for i in range(1, n_iter_no_corr):
        controlnoiset = np.random.normal(0, sigma_X)
        controlnoiser = np.random.normal(0, sigma_Z)
        for m in range(0, n_robots):
            x1, v1 = movenext(truepos[m, :, n_iter_no_corr*j+i-1], truetheta[m, n_iter_no_corr*j+i-1],
                              X[m, n_iter_no_corr*j+i-1]*(1+controlnoiset), Z[m, n_iter_no_corr*j+i-1]*(1+controlnoiser), 0.5)
            truetheta[m, n_iter_no_corr*j+i] += v1
            truepos[m, :, n_iter_no_corr*j+i] += np.array([x1[0, 0], x1[2, 0]])
            x3, v3 = kal[m].predict(currpos[m, :, n_iter_no_corr*j+i-1], currtheta[m, n_iter_no_corr*j+i-1],
                                    X[m, n_iter_no_corr*j+i-1], Z[m, n_iter_no_corr*j+i-1], 0.5)
            currtheta[m, n_iter_no_corr*j+i] += v3
            currpos[m, :, n_iter_no_corr*j+i] += np.array([x3[0, 0], x3[2, 0]])
        for m in range(0, n_robots):
            if m != 0 and m != n_robots-1:
                X[m, n_iter_no_corr*j+i], Z[m, n_iter_no_corr*j+i] = ctrl[m].get_controls(currtheta[m, n_iter_no_corr*j+i], currpos[m, :, n_iter_no_corr*j+i],
                                                                                          currpos[m-1, :, n_iter_no_corr*j+i], currpos[m+1, :, n_iter_no_corr*j+i],
                                                                                          n_iter_no_corr/20.0, 2, 2)
            elif m == 0:
                X[m, n_iter_no_corr*j+i], Z[m, n_iter_no_corr*j+i] = ctrl[m].get_controls(currtheta[m, n_iter_no_corr*j+i], currpos[m, :, n_iter_no_corr*j+i],
                                                                                          basepos, currpos[m+1, :, n_iter_no_corr*j+i],
                                                                                          n_iter_no_corr/20.0, 2, 2)
            elif m == n_robots-1:
                X[m, n_iter_no_corr*j+i], Z[m, n_iter_no_corr*j+i] = ctrl[m].get_controls(currtheta[m, n_iter_no_corr*j+i], currpos[m, :, n_iter_no_corr*j+i],
                                                                                          currpos[m-1, :, n_iter_no_corr*j+i], endnodepos,
                                                                                          n_iter_no_corr/20.0, 2, 2)
"""
"""
print np.sum(np.linalg.norm(truepos-corrpos, axis=0))
print np.sum(np.linalg.norm(truepos-calcpos, axis=0))
"""
plt.plot(basepos[0], basepos[1], 'co')
plt.plot(endnodepos[0], endnodepos[1], 'mo')
for i in range(0, n_robots):
    plt.plot(truepos[i, 0, :], truepos[i, 1, :], 'r')
    plt.plot(currpos[i, 0, :], currpos[i, 1, :], 'b')
    plt.plot(measpos[i, 0, :], measpos[i, 1, :], 'o')
plt.show()
