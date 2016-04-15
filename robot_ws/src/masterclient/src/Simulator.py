import numpy as np
import matplotlib.pyplot as plt
import kalman


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
    x_k1_k1 = np.array([[pos[0]],
                        [X*np.cos(theta)],
                        [pos[1]],
                        [X*np.sin(theta)]])
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

sigma_meas = 0.5
sigma_X = 0.05
sigma_Z = 0.025
X = 2*np.random.rand(100)
Z = 2*np.random.rand(len(X))-1
#X = np.array([1, 2, 0.4, 1.2, 0.7, 1, 2, 0.4, 1.2, 0.7])
#Z = np.array([2e-3, -0.5, -0.9, 1, 1.4, 2e-3, -0.5, -0.9, 1, 1.4])
n=2
truepos = np.zeros((2, n*len(X)))
truetheta = np.zeros(n*len(X))
measpos = np.zeros((2, len(X))) + np.random.normal(0, sigma_meas, (2, len(X)))
meastheta = np.zeros(len(X)) + np.random.normal(0, sigma_meas/2.0, len(X))
calcpos = np.zeros((2, n*len(X)))
calctheta = np.zeros(n*len(X))
corrpos = np.zeros((2, n*len(X)))
corrtheta = np.zeros(n*len(X))

truepos[:, 0] += np.array([1, 0])
truetheta[0] += np.pi
measpos[:, 0] += np.array([1, 0])
meastheta[0] += np.pi
calcpos[:, 0] += np.array([1, 0])
calctheta[0] += np.pi
corrpos[:, 0] += np.array([1, 0])
corrtheta[0] += np.pi

Q = np.zeros((4, 4))
p = kalman.estimateP(sigma_meas, sigma_X, sigma_Z)
for j in range(0, len(X)):
    if j > 0:
        controlnoiset = np.random.normal(0, sigma_X)
        controlnoiser = np.random.normal(0, sigma_Z)
        x1, v1 = movenext(truepos[:, n*j-1], truetheta[n*j-1], X[j]+controlnoiset, Z[j]+controlnoiser, 0.5)
        # x1, v1 = movenext(truepos[:, n*j-1], truetheta[n*j-1], X[j]*(1+controlnoiset), Z[j]*(1+controlnoiser), 0.5)
        truetheta[n*j] += v1
        truepos[:, n*j] += np.array([x1[0, 0], x1[2, 0]])
        measpos[:, j] += truepos[:, n*j]
        x2, v2, Q1 = kalman.predict(calcpos[:, n*j-1], calctheta[n*j-1],
                                    X[j], sigma_X,
                                    Z[j], sigma_Z, 0.5, Q)
        calctheta[n*j] += v2
        calcpos[:, n*j] += np.array([x2[0, 0], x2[2, 0]])
        x3, p, v3, Q = kalman.correct(corrpos[:, n*j-1], corrtheta[n*j-1],  measpos[:, j], sigma_meas, p,
                                      X[j], sigma_X,
                                      Z[j], sigma_Z, 0.5, Q)
        corrtheta[n*j] += v3
        corrpos[:, n*j] += np.array([x3[0, 0], x3[2, 0]])

    for i in range(1, n):
        controlnoiset = np.random.normal(0, sigma_X)
        controlnoiser = np.random.normal(0, sigma_Z)
        x1, v1 = movenext(truepos[:, n*j+i-1], truetheta[n*j+i-1], X[j]+controlnoiset, Z[j]+controlnoiser, 0.5)
        #x1, v1 = movenext(truepos[:, n*j+i-1], truetheta[n*j+i-1], X[j]*(1+controlnoiset), Z[j]*(1+controlnoiser), 0.5)
        truetheta[n*j+i] += v1
        truepos[:, n*j+i] += np.array([x1[0, 0], x1[2, 0]])
        x2, v2, Q1 = kalman.predict(calcpos[:, n*j+i-1], calctheta[n*j+i-1],
                                    X[j], sigma_X,
                                    Z[j], sigma_Z, 0.5, Q)
        calctheta[n*j+i] += v2
        calcpos[:, n*j+i] += np.array([x2[0, 0], x2[2, 0]])
        x3, v3, Q = kalman.predict(corrpos[:, n*j+i-1], corrtheta[n*j+i-1],
                                      X[j], sigma_X,
                                      Z[j], sigma_Z, 0.5, Q)
        corrtheta[n*j+i] += v3
        corrpos[:, n*j+i] += np.array([x3[0, 0], x3[2, 0]])


print np.sum(np.linalg.norm(truepos-corrpos, axis=0))
print np.sum(np.linalg.norm(truepos-calcpos, axis=0))
plt.plot(truepos[0, :], truepos[1, :], 'r')
plt.plot(calcpos[0, :], calcpos[1, :], 'g')
plt.plot(corrpos[0, :], corrpos[1, :], 'b')
plt.plot(measpos[0, :], measpos[1, :], 'o')
plt.show()
