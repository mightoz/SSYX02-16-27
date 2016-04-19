import numpy as np


def find_next_pos(currpos, neighbour1pos, neighbour2pos, k):
    """

    :param currpos: your position
    :param neighbour1pos: your first neighbour
    :param neighbour2pos: your left neighbour
    :param k: some constant (0<k<1) to get target = pos + k*grad(f)
    :return: the next target position
    """
    dist1 = neighbour1pos - currpos
    dist2 = neighbour2pos - currpos
    return currpos + k * (dist1 + dist2)


def get_rot_dir(theta, currpos, tarpos):
    """

    :param theta: angle
    :param currpos: [xstart,ystart]
    :param tarpos: [xend,yend]
    :return:
    """
    # calculate the angle between the x-axis and the line intersecting endpos and startpos
    phi = 0
    if np.abs(currpos[0] - tarpos[0]) > 1e-30:
        z1 = currpos - tarpos
        z2 = np.array([1, 0])
        if currpos[1] >= tarpos[1]:  # 0 <= phi <= pi
            phi = np.arccos(np.dot(z1, z2) / (np.linalg.norm(z1) * np.linalg.norm(z2)))
        else:  # pi < phi < 2*pi
            phi = 2 * np.pi - np.arccos(np.dot(z1, z2) / (np.linalg.norm(z1) * np.linalg.norm(z2)))
    else:
        phi = np.sign(currpos[1] - tarpos[1]) * np.pi / 2

    # calculate the effective angle needed to determine how to change Z
    angle = np.mod(theta - np.pi, 2 * np.pi)
    # determine the rotation direction (+1 ccw, -1 cw)
    if 0 <= phi <= np.pi:
        if phi <= angle < (phi + np.pi):
            return -1
        else:
            return 1
    else:
        if np.mod(phi + np.pi, 2 * np.pi) <= angle < phi:
            return 1
        else:
            return -1


class Controls(object):
    def __init__(self, x_min, x_max, z_min, z_max):
        self.X_min = x_min
        self.X_max = x_max
        self.Z_min = z_min
        self.Z_max = z_max

    def get_trans_magn_1(self, currpos, tarpos, T, phi):
        """

        :param currpos: [xstart, ystart]
        :param tarpos: [xend, yend]
        :param T: time to reach target (if it heading straight towards the endpos)
        :param phi: angle to rotate to face target pos
        :return: The velocity for the next iteration
        """
        dist = np.linalg.norm(currpos - tarpos)
        if dist / T * (np.pi - phi) / np.pi > self.X_max:
            return self.X_max
        elif dist / T * (np.pi - phi) / np.pi < self.X_min:
            return self.X_min
        else:
            return dist / T * (np.pi - phi) / np.pi

    def get_trans_magn_2(self, current, target, t, phi):

        lengthtotarget = np.linalg.norm(target-current)

        # X FUNKTION AV Z OCH LENGTH

        if lengthtotarget > 0.5:
            x = 1
        elif lengthtotarget > 0.25:
            x = 0.5 * lengthtotarget / t
        elif lengthtotarget > 0.05:
            x = 0.25 * lengthtotarget / t
        else:
            x = 0

        return x

    def get_rot_magn_1(self, theta, currpos, tarpos, t):
        """

        :param theta: angle
        :param currpos: [xstart, ystart]
        :param tarpos: [xend, yend]
        :param t: time to face target
        :return: The rotation for the next iteration
        """
        y = tarpos - currpos
        A = np.array([[np.cos(theta), np.sin(theta)],
                      [np.sin(theta), -np.cos(theta)]])
        x = np.linalg.solve(A, y)

        d = np.array([x[0] * np.cos(theta), x[0] * np.sin(theta)])

        if x[0] >= 0:
            phi = np.arccos(np.dot(y, d) / (np.linalg.norm(y) * np.linalg.norm(d)))
        else:
            phi = np.pi - np.arccos(np.dot(y, d) / (np.linalg.norm(y) * np.linalg.norm(d)))

        if phi / t > self.Z_max:
            return self.Z_max
        elif phi / t < self.Z_min:
            return self.Z_min
        else:
            return phi / t

    def get_rot_magn_2(self, theta, current, target, t):

        direction = get_rot_dir(theta, current, target)

        a = np.array([[np.cos(theta), np.sin(theta)],[np.sin(theta), -np.cos(theta)]])
        b = target-current
        (alfa,beta) = np.linalg.solve(a,b)
        (alfa,beta) = (np.abs(alfa),np.abs(beta))
        lengthtotarget = np.linalg.norm(target-current)

        phi = np.arctan(beta / alfa)

        x = np.arctan((target[1]-current[1])/(target[0]-current[0]))


        if np.cos(theta-x) < 0:
            print ('phi', phi)
            phi = np.pi-phi

        if phi > (45*(np.pi)/180):
            za = 1
        elif phi > (20*(np.pi)/180):
            za = 0.5
        elif phi > (5*(np.pi)/180):
            za = 0.25
        else:
            print "z blev noll"
            za = 0
            print ('lengthtotarget:', str(lengthtotarget))
        print za

        if lengthtotarget < 0.05:
            z = 0
        else:
            z = phi / t * za * direction
        return z



    def get_controls(self, theta, currpos, neighbour1pos, neighbour2pos, k, T_X, T_Z):
        """

        :param theta: orientation relative to the x-axis in the coordinate grid. positive ccw
        :param currpos: current position
        :param neighbour1pos: position of neighbour 1
        :param neighbour2pos: position of neighbour 2. if only 1 neighbour exist, set it to 0
        :param k: tarpos = currpos + k*grad(f)
        :param T_X: Time to reach target pos if heading straight towards it
        :param T_Z: Time to face target pos
        :return: the next controls; X, Z
        """
        """
        if neighbour2pos == 0:
            tarpos = self.findnextpos(currpos, neighbour1pos, currpos, k)
        else:
            tarpos = self.findnextpos(currpos, neighbour1pos, neighbour2pos, k)
        """
        tarpos = find_next_pos(currpos, neighbour1pos, neighbour2pos, k)
        nextZ = get_rot_dir(theta, currpos, tarpos) * self.get_rot_magn_2(theta, currpos, tarpos, T_Z)
        nextX = self.get_trans_magn_2(currpos, tarpos, T_X, np.abs(nextZ * T_Z))
        return nextX, nextZ
