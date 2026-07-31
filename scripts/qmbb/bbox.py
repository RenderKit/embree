import numpy as np
import math

class BBox():
    def __init__(self, bbmin=None, bbmax=None):
        if bbmin is None:
            self.bbmin = np.array([math.inf, math.inf, math.inf, math.inf])
        else:
            self.bbmin = bbmin
        if bbmax is None:
            self.bbmax = np.array([-math.inf, -math.inf, -math.inf, -math.inf])
        else:
            self.bbmax = bbmax

    def extend(self, other):
        if isinstance(other, BBox):
            self.bbmin = np.minimum(self.bbmin, other.bbmin)
            self.bbmax = np.maximum(self.bbmax, other.bbmax)
        else:
            self.bbmin = np.minimum(self.bbmin, other)
            self.bbmax = np.maximum(self.bbmax, other)

    def extendAll(self, *others):
        for other in others:
            self.extend(other)
            
    def contains(self, p, eps=0.0):
        for i in range(3):
            if p[i] < self.bbmin[i]-eps or p[i] > self.bbmax[i]+eps:
                return False
        return True

    def getCorners(self):
        corners = np.array([[self.bbmin[0], self.bbmin[1], self.bbmin[2], 1],
                            [self.bbmin[0], self.bbmin[1], self.bbmax[2], 1],
                            [self.bbmin[0], self.bbmax[1], self.bbmin[2], 1],
                            [self.bbmin[0], self.bbmax[1], self.bbmax[2], 1],
                            [self.bbmax[0], self.bbmin[1], self.bbmin[2], 1],
                            [self.bbmax[0], self.bbmin[1], self.bbmax[2], 1],
                            [self.bbmax[0], self.bbmax[1], self.bbmin[2], 1],
                            [self.bbmax[0], self.bbmax[1], self.bbmax[2], 1]])
        return corners

    def __str__(self):
        return "BBox\n    %s\n    %s" % (self.bbmin, self.bbmax)

def getFacesForBBoxCorners(c):
    faces = [[c[0][:3], c[1][:3], c[3][:3], c[2][:3]],
             [c[4][:3], c[5][:3], c[7][:3], c[6][:3]],
             [c[0][:3], c[2][:3], c[6][:3], c[4][:3]],
             [c[1][:3], c[3][:3], c[7][:3], c[5][:3]],
             [c[0][:3], c[1][:3], c[5][:3], c[4][:3]],
             [c[2][:3], c[3][:3], c[7][:3], c[6][:3]]]
    return faces
