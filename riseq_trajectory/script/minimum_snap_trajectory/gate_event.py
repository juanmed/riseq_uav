#!/usr/bin/env python

import math
import numpy as np


class GateEvent():
    # Constructor for class gate
    # @param: location: 4x3 array of corner locations
    # @param: inflation: Artificial inflation of gate to test for fly through
    def __init__(self, location, inflation):
        self.location = np.asarray(location)
        self.planeEquation = self.getPlaneOfGate()

        minLoc = np.amin(self.location, axis=0)
        maxLoc = np.amax(self.location, axis=0)
        self.xmin = minLoc[0] - inflation
        self.xmax = maxLoc[0] + inflation
        self.ymin = minLoc[1] - inflation
        self.ymax = maxLoc[1] + inflation
        self.zmin = minLoc[2] - inflation
        self.zmax = maxLoc[2] + inflation

    ## @brief Function to get the plane of the gate bounding box
    # @param self The object pointer
    def getPlaneOfGate(self):
        p1 = self.location[0, :]
        p2 = self.location[1, :]
        p3 = self.location[2, :]

        v1 = p3 - p1
        v2 = p2 - p1

        cp = np.cross(v1, v2)
        a, b, c = cp
        d = np.dot(cp, p3)
        return np.asarray([a, b, c, -d])

    ## @brief Function to get the distance of a point from plane
    # @param self The object pointer
    # @param point The query point to calcluate distance from
    def getDistanceFromPlane(self, point):
        d = math.fabs((self.planeEquation[0] * point[0] + self.planeEquation[1] * point[1] + self.planeEquation[2] *
                       point[2] + self.planeEquation[3]))
        e = math.sqrt(self.planeEquation[0] ** 2 + self.planeEquation[1] ** 2 + self.planeEquation[2] ** 2)
        return (d / e)

    ## @brief Function to check if the drone is flying through a gate
    # @param self The object pointer
    # @param point The translation of the drone
    # @param tol The point to plane distance that is considered acceptable
    def isEvent(self, point, tol):
        # Check if we are inside the inflated gate
        if (point[0] < self.xmax) and (point[0] > self.xmin):
            if (point[1] < self.ymax) and (point[1] > self.ymin):
                if (point[2] < self.zmax) and (point[2] > self.zmin):
                    # Compute the distance from the gate
                    d = self.getDistanceFromPlane(point)
                    if (d < tol):
                        return True
        return False
