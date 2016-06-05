import numpy as np


"""
This class is the anchors with known locations
used for the reference grid.
"""


class Anchor(object):

    def __init__(self):
        self.pos = None
        self.ip = None

    def set_pos(self, x, y):
        self.pos = np.array([x, y])

    def set_ip(self, val):
        self.ip = val

    def get_pos(self):
        return self.pos

    def get_ip(self):
        return self.ip
