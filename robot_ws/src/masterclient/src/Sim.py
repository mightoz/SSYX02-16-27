import numpy as np
import matplotlib.pyplot as plt
import Robot

sigmaMeas = 0.05
n_iter = 70
n_rob = 3
n_iter_no_corr = n_rob-1

Xmax = 1
Xmin = 0.05
Zmax = 1
Zmin = 0
sigmaX = 0.05
sigmaZ = 0.025
X = 0
Z = 0
robot = []
trueRobot = []
kalman = []
controls = []
for i in range(0, n_rob):
    robot += [Robot.Robot(X, Z, 2*np.pi*np.random.rand(), 10*np.random.rand(2)-5)]
    robot[i].setKalman(sigmaMeas, sigmaX, sigmaZ, dt)
    trueRobot += [Robot.Robot(X, Z, robot[i].getTheta(), robot[i].getPos())]
BASE = Robot.Robot(0, 0, 0, 10*np.random.rand(2)-5)
ENDNODE = Robot.Robot(0, 0, 0, 10*np.random.rand(2)-5)
plt.plot(BASE.getPos()[0], BASE.getPos()[1], 'bo')
plt.plot(ENDNODE.getPos()[0], ENDNODE.getPos()[1], 'go')

for j in range(0, n_iter):
    corr_idx = np.mod(j, n_rob)  # Decide which robot should correct its position
    for i in range(0, n_rob):  # Calculate/Estimate new state
        x1, v1 = trueRobot[i].getKalman().predict(trueRobot[i].getPos(), trueRobot[i].getTheta(),
                                                  trueRobot[i].getX(), trueRobot[i].getZ(),
                                                  trueRobot[i].getKalman().getTimeStep())
        trueRobot[i].setTheta(v1)
        trueRobot[i].setPos(np.array([x1[0, 0], x1[2, 0]]))
        if i != corr_idx:
            x2, v2 = robot[i].getKalman().predict(robot[i].getPos(), robot[i].getTheta(),
                                                  robot[i].getX(), robot[i].getZ(),
                                                  robot[i].getKalman().getTimeStep())
            robot[i].setTheta(v2)
            robot[i].setPos(np.array([x2[0, 0], x2[2, 0]]))
        else:
            # We should have a method call that measures the robot's position here
            measpos = trueRobot[i].getPos()+np.random.normal(0, trueRobot[i].getKalman().getStdMeas(), 2)
            x2, v2 = robot[i].getKalman().correct(robot[i].getPos(), robot[i].getTheta(),
                                                  measpos, robot[i].getX(), robot[i].getZ(),
                                                  robot[i].getKalman().getTimeStep())
            robot[i].setTheta(v2)
            robot[i].setPos(np.array([x2[0, 0], x2[2, 0]]))
            plt.plot(measpos[0], measpos[1], 'ro')
        plt.plot(robot[i].getPos()[0], robot[i].getPos()[1], 'ko')
    for i in range(0, n_rob):  # Calculate new controls at time k
        controlnoiset = np.random.normal(0, robot[i].getSigmaX())
        controlnoiser = np.random.normal(0, robot[i].getSigmaZ())
        if i != 0 and i != n_rob-1:
            x3, v3 = robot[i].getControls().get_controls(robot[i].getTheta(), robot[i].getPos(),
                                                         robot[i-1].getPos(), robot[i+1].getPos(),
                                                         robot[i].getControls().getK(),
                                                         robot[i].getControls().getTX(),
                                                         robot[i].getControls().getTZ())
            robot[i].setX(x3)
            robot[i].setZ(v3)
            trueRobot[i].setX(x3*(1+controlnoiset))
            trueRobot[i].setZ(v3*(1+controlnoiser))
        elif i == 0:
            x3, v3 = robot[i].getControls().get_controls(robot[i].getTheta(), robot[i].getPos(),
                                                         BASE.getPos(), robot[i+1].getPos(),
                                                         robot[i].getControls().getK(),
                                                         robot[i].getControls().getTX(),
                                                         robot[i].getControls().getTZ())
            robot[i].setX(x3)
            robot[i].setZ(v3)
            trueRobot[i].setX(x3*(1+controlnoiset))
            trueRobot[i].setZ(v3*(1+controlnoiser))
        elif i == n_rob-1:
            x3, v3 = robot[i].getControls().get_controls(robot[i].getTheta(), robot[i].getPos(),
                                                         robot[i-1].getPos(), ENDNODE.getPos(),
                                                         robot[i].getControls().getK(),
                                                         robot[i].getControls().getTX(),
                                                         robot[i].getControls().getTZ())
            robot[i].setX(x3)
            robot[i].setZ(v3)
            trueRobot[i].setX(x3*(1+controlnoiset))
            trueRobot[i].setZ(v3*(1+controlnoiser))

plt.plot(robot[0].getPos()[0], robot[0].getPos()[1], 'yo')
plt.plot(robot[1].getPos()[0], robot[1].getPos()[1], 'yo')
plt.plot(robot[2].getPos()[0], robot[2].getPos()[1], 'yo')
plt.show()
