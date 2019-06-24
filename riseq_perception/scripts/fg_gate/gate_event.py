#!/usr/bin/env python
import rospy
import math
import numpy as np


class GateEvent():
    def __init__(self, location, inflation, tolerance):
        self.location = np.asarray(location)
        self.planeEquation = self.getPlaneOfGate()
        self.tolerance = tolerance

        minLoc = np.amin(self.location, axis=0)
        maxLoc = np.amax(self.location, axis=0)
        self.xmin = minLoc[0] - inflation
        self.xmax = maxLoc[0] + inflation
        self.ymin = minLoc[1] - inflation
        self.ymax = maxLoc[1] + inflation
        self.zmin = minLoc[2] - inflation
        self.zmax = maxLoc[2] + inflation

    def getPlaneOfGate(self):
        """
        Function to get the plane of the gate bounding box
        """
        p1 = self.location[0, :]
        p2 = self.location[1, :]
        p3 = self.location[2, :]

        v1 = p3 - p1
        v2 = p2 - p1

        cp = np.cross(v1, v2)
        a, b, c = cp
        d = np.dot(cp, p3)
        return np.asarray([a, b, c, -d])

    def getDistanceFromPlane(self, point):
        """
        Function to get the distance of a point from plane
        plane : ax + by + cz + d = 0
        point : [ x1, y1, z1 ]
        distance : |ax1 + by1 + cz1 + d| / (x1^2 + y1^2 + z1^2)^0.5
        """
        d = math.fabs((self.planeEquation[0] * point[0] + self.planeEquation[1] * point[1] + self.planeEquation[2] *
                       point[2] + self.planeEquation[3]))
        e = math.sqrt(self.planeEquation[0] ** 2 + self.planeEquation[1] ** 2 + self.planeEquation[2] ** 2)
        distance = d/e
        return distance

    def isEvent(self, point):
        """
        Function to check if the drone is flying through a gate
        """
        # Check if we are inside the inflated gate
        if (point[0] < self.xmax) and (point[0] > self.xmin):
            if (point[1] < self.ymax) and (point[1] > self.ymin):
                if (point[2] < self.zmax) and (point[2] > self.zmin):
                    # Compute the distance from the gate
                    d = self.getDistanceFromPlane(point)
                    if d < self.tolerance:
                        return True
        return False


def check_gate(gate_events, current_position, gate_pass):
    """
    Function to check whether drone pass gate or not
    It needs current position of drone.
    """
    gate_name = rospy.get_param("/uav/gate_names")

    if gate_events[gate_pass].isEvent(current_position):
        gate_pass = gate_pass + 1
    # Check every gate not only next gate
    # for the case when drone skip gate and fly through another gate
    for i, gate in enumerate(gate_events[gate_pass:]):
        if gate.isEvent(current_position):
            # i == 0 means that drone goes well
            if i > 0:
                rospy.loginfo("Skipped %d events", i)
                # Record gate which drone skip
                for j in range(i):
                    rospy.loginfo("%s False", gate_name[gate_pass + j])
            rospy.loginfo("Reached %s at", gate_name[gate_pass + i])
            gate_pass += (i + 1)
        gate_count = len(gate_name)
        if gate_pass >= gate_count:
            rospy.loginfo("Completed the challenge")
            rospy.signal_shutdown("Challenge complete")
    return gate_pass


'''
        # Should count gate number because need to know next gate
        # Inflation means the scale of virtual cube including way point
        # For example, it will make cube space which is larger as much as gate times inflation.
        self.inflation = 2
        # Tolerance is like threshold to decide whether drone pass or not
        # If drone is close to gate within tolerance, it is determined as drone pass gate.
        self.tolerance = 1
        # Make array Class for counting gate which is traversed
        self.gate_events = []
        for i in range(self.gate_count):
            self.gate_events.append(ga.GateEvent(self.gate_location[i], self.inflation, self.tolerance))
        # count pass
'''