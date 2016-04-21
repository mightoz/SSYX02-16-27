import numpy as np


def get_rot_dir(theta, curr_pos, tar_pos):
    """

    :param theta: angle
    :param curr_pos: [x_start,y_start]
    :param tar_pos: [x_end,y_end]
    :return:
    """
    # calculate the angle between the x-axis and the line intersecting end_pos and start_pos
    if np.abs(curr_pos[0] - tar_pos[0]) > 1e-30:
        z1 = curr_pos - tar_pos
        z2 = np.array([1, 0])
        if curr_pos[1] >= tar_pos[1]:  # 0 <= phi <= pi
            phi = np.arccos(np.dot(z1, z2) / (np.linalg.norm(z1) * np.linalg.norm(z2)))
        else:  # pi < phi < 2*pi
            phi = 2 * np.pi - np.arccos(np.dot(z1, z2) / (np.linalg.norm(z1) * np.linalg.norm(z2)))
    else:
        phi = np.sign(curr_pos[1] - tar_pos[1]) * np.pi / 2

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
    def __init__(self, x_min, x_max, z_min, z_max, k, t_x, t_z, ok_dist):
        self.x_min = x_min
        self.x_max = x_max
        self.z_min = z_min
        self.z_max = z_max
        self.k = k
        self.t_x = t_x
        self.t_z = t_z
        self.ok_dist = ok_dist

    def find_next_pos(self, curr_pos, neighbour_1_pos, neighbour_2_pos):
        """

        :param curr_pos: your position
        :param neighbour_1_pos: your first neighbour
        :param neighbour_2_pos: your left neighbour
        :return: the next target position
        """
        dist1 = neighbour_1_pos - curr_pos
        dist2 = neighbour_2_pos - curr_pos
        return curr_pos + self.k * (dist1 + dist2)

    def get_trans_magn_1(self, curr_pos, tar_pos, phi):
        """

        :param curr_pos: [x_start, y_start]
        :param tar_pos: [x_end, y_end]
        :param phi: angle to rotate to face target pos
        :return: The velocity for the next iteration
        """
        dist = np.linalg.norm(curr_pos - tar_pos)
        if dist < self.ok_dist:
            return 0
        else:
            if dist / self.t_x * (np.pi - phi) / np.pi > self.x_max:
                return self.x_max
            elif dist / self.t_x * (np.pi - phi) / np.pi < self.x_min:
                return self.x_min
            else:
                return dist / self.t_x * (np.pi - phi) / np.pi

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

    def get_rot_magn_1(self, theta, curr_pos, tar_pos):
        """

        :param theta: angle
        :param curr_pos: [x_start, y_start]
        :param tar_pos: [x_end, y_end]
        :return: The rotation for the next iteration
        """
        y = tar_pos - curr_pos
        A = np.array([[np.cos(theta), np.sin(theta)],
                      [np.sin(theta), -np.cos(theta)]])
        x = np.linalg.solve(A, y)

        d = np.array([x[0] * np.cos(theta), x[0] * np.sin(theta)])
        if np.linalg.norm(y) > 1e-40:
            if x[0] >= 0:
                phi = np.arccos(np.dot(y, d) / (np.linalg.norm(y) * np.linalg.norm(d)))
            else:
                phi = np.pi - np.arccos(np.dot(y, d) / (np.linalg.norm(y) * np.linalg.norm(d)))
        else:
            phi = 0

        if phi / self.t_z > self.z_max:
            return self.z_max
        elif phi / self.t_z < self.z_min:
            return self.z_min
        else:
            return phi / self.t_z

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

    def calc_controls(self, theta, curr_pos, neighbour_1_pos, neighbour_2_pos, wheel_dist):
        """

        :param theta: orientation relative to the x-axis in the coordinate grid. positive ccw
        :param curr_pos: current position
        :param neighbour_1_pos: position of neighbour 1
        :param neighbour_2_pos: position of neighbour 2. if only 1 neighbour exist, set it to 0
        :param wheel_dist: distance between the front wheels
        :return: the next controls; X, Z
        """

        tar_pos = self.find_next_pos(curr_pos, neighbour_1_pos, neighbour_2_pos)
	print "Hej", tar_pos
        next_z = get_rot_dir(theta, curr_pos, tar_pos) * self.get_rot_magn_1(theta, curr_pos, tar_pos)
        next_x = self.get_trans_magn_1(curr_pos, tar_pos, np.abs(next_z * self.t_z))
        return next_x-wheel_dist*next_z/2, next_z

    def set_x_max(self, val):
        self.x_max = val

    def set_x_min(self, val):
        self.x_min = val

    def set_z_max(self, val):
        self.z_max = val

    def set_z_min(self, val):
        self.z_min = val

    def set_k(self, val):
        self.k = val

    def set_t_x(self, val):
        self.t_x = val

    def set_t_z(self, val):
        self.t_z = val

    def set_ok_dist(self, val):
        self.ok_dist = val

    def get_x_max(self):
        return self.x_max

    def get_x_min(self):
        return self.x_min

    def get_z_max(self):
        return self.z_max

    def get_z_min(self):
        return self. z_min

    def get_k(self):
        return self.k

    def get_t_x(self):
        return self.t_x

    def get_t_z(self):
        return self.t_z

    def get_ok_dist(self):
        return self.ok_dist
