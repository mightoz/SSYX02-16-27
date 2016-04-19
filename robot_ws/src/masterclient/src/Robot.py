import numpy as np
import Kalman
import Controls


class Robot(object):

    def __init__(self, X, Z, theta, pos):
        self.transVel = X
        self.angVel = Z
        self.orientation = theta
        self.currPos = pos
        self.kalman = Kalman.Kalman(0.5, 0, 0, 0)
        self.controls = Controls.Controls(0, 0, 0, 0, 0, 2, 2)

    def setKalman(self, sigmaMeas, sigmaX, sigmaZ, dt):
        self.kalman.setStdMeas(sigmaMeas)
        self.kalman.setStdX(sigmaX)
        self.kalman.setStdZ(sigmaZ)
        self.kalman.setTimeStep(dt)

    def setControls(self, xMin, xMax, zMin, zMax, k, tX, tZ):
        self.controls.setXMin(xMin)
        self.controls.setXMax(xMax)
        self.controls.setZMin(zMin)
        self.controls.setZMax(zMax)
        self.controls.setK(k)
        self.controls.setTX(tX)
        self.controls.setTZ(tZ)

    def setX(self, val):
        self.transVel = val

    def setZ(self, val):
        self.angVel = val

    def setTheta(self, val):
        self.orientation = val
        self.kalman.setOrientation(val)

    def setPos(self, val):
        self.currPos = val

    def getX(self):
        return self.transVel

    def getZ(self):
        return self.angVel

    def getTheta(self):
        return self.orientation

    def getPos(self):
        return self.currPos

    def getKalman(self):
        return self.kalman

    def getControls(self):
        return self.controls
