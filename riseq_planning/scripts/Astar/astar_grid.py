#!/usr/bin/env python
import rospy
import numpy as np
from heapq import *
from riseq_planning.waypoint_publisher import WayPointPublisher


class AStarGrid(WayPointPublisher):
    """
    Class to solve A* algorithm based on grid map and publish way point
    """
    def __init__(self, nmap, start, goal):
        self.nmap = nmap
        self.start = start
        self.goal = goal

        self.path = self.astar()
        self.path.reverse()

        self.waypoint = self.extract_waypoint()
        self.waypoint.insert(0, self.start)
        self.waypoint.append(self.goal)

        new_waypoint = self.reconstruct_waypoint()

        super(AStarGrid, self).__init__(new_waypoint)

    def heuristic(self, a, b):
        """
        helper function to get distance between A to B
        It can be used as heuristic cost estimate
        """
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def reconstruct_path(self, current, came_from):
        """
        function to construct path based on A* algorithm in grid map
        """
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path

    def astar(self):
        close_set = set()
        open_set = []

        # dictionary
        came_from = {}
        gscore = {self.start: 0}
        fscore = {self.start: self.heuristic(self.start, self.goal)}

        heappush(open_set, (fscore[self.start], self.start))

        # while open_set is not empty
        while open_set:

            # the node in openSet having the lowest fscore value
            # remove current in open_set
            current = heappop(open_set)[1]
            if current == self.goal:
                return self.reconstruct_path(current, came_from)

            close_set.add(current)
            neighbors = self.construct_neighbor(current)

            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                if neighbor in close_set:
                    # ignore the neighbor which is already evaluated
                    continue

                tentative_g_score = gscore[current] + self.heuristic(current, neighbor)

                if neighbor not in [i[1] for i in open_set] or tentative_g_score < gscore[neighbor]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, self.goal)
                    heappush(open_set, (fscore[neighbor], neighbor))

        return False

    def construct_neighbor(self, current):
        """
        function to construct neighbor around current point.
        Ignore wall and occupied space.
        Even also it ignores diagonal space where drone can not fly through.
        """
        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        is_neighbor = np.ones((3, 3), dtype=bool)
        is_neighbor[1, 1] = False

        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            if 0 <= neighbor[0] < self.nmap.shape[0]:
                if 0 <= neighbor[1] < self.nmap.shape[1]:
                    if self.nmap[neighbor[0]][neighbor[1]] == 1:
                        is_neighbor[i+1][j+1] = False
                else:
                    is_neighbor[i + 1][j + 1] = False
                    # array bound y walls
            else:
                is_neighbor[i + 1][j + 1] = False
                # array bound x walls

        # ignore diagonal space
        if bool(is_neighbor[0, 1]) is False:
            is_neighbor[0, 0] = False
            is_neighbor[0, 2] = False
        if bool(is_neighbor[1, 0]) is False:
            is_neighbor[0, 0] = False
            is_neighbor[2, 0] = False
        if bool(is_neighbor[1, 2]) is False:
            is_neighbor[0, 2] = False
            is_neighbor[2, 2] = False
        if bool(is_neighbor[2, 1]) is False:
            is_neighbor[2, 0] = False
            is_neighbor[2, 2] = False

        new_neighbors = []
        for i in range(0, 3):
            for j in range(0, 3):
                if bool(is_neighbor[i][j]) is True:
                    new_neighbors.append((i-1, j-1))

        return new_neighbors

    def extract_waypoint(self):
        """
        function to extract way points in path
        this way points will be used to generate trajectory.
        """
        waypoint = []
        for point in self.path:

            # omit point which adjacent to obstacle
            is_adjacement = False
            for i in (-1, 1):
                if 0 <= point[0] + i < self.nmap.shape[0]:
                    if self.nmap[point[0]+i][point[1]] == 1:
                        is_adjacement = True
                        break
                if 0 <= point[1] + i < self.nmap.shape[1]:
                    if self.nmap[point[0]][point[1]+i] == 1:
                        is_adjacement = True
                        break
            if is_adjacement is True:
                continue

            # include point which is located at diagonal direction from obstacle
            is_waypoint = False
            for i in (-1, 1):
                for j in (-1, 1):
                    if 0 <= point[0]+i < self.nmap.shape[0]:
                        if 0 <= point[1]+j < self.nmap.shape[1]:
                            if self.nmap[point[0]+i][point[1]+j] == 1:
                                waypoint.append(point)
                                is_waypoint = True
                                break
                if is_waypoint is True:
                     break

        return waypoint

    def reconstruct_waypoint(self):
        """
        function to reconstruct way point
        For now, A* algorithm solves path in 2D grid map and ignore heading of drone.
        Because point's form is (x, y), it have to be changed to (x, y, z, psi) form.
        """
        m = len(self.waypoint)
        new_waypoint = np.zeros((m, 4))

        for i in range(0, m):
            new_waypoint[i][0] = self.waypoint[i][0]  # x
            new_waypoint[i][1] = self.waypoint[i][1]  # y
            new_waypoint[i][2] = 0  # z
            new_waypoint[i][3] = 0  # psi

        return new_waypoint


'''
nmap = np.array([
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
'''

nmap = np.array([
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
'''
map = np.array([
[0, 0, 0, 0],
[1, 1, 0, 1],
[0, 0, 0, 0]])
'''


if __name__ == '__main__':
    rospy.init_node('riseq_waypoint_publisher', anonymous=True)

    # Wait some time before running. This is to adapt to some simulators
    # which require some 'settling time'

    try:
        wait_time = int(rospy.get_param('riseq/planning_wait'))
    except:
        print('riseq/planning_wait_time parameter is unavailable')
        print('Setting a wait time of 0 seconds.')
        wait_time = 1

    # wait time for simulator to get ready...
    while rospy.Time.now().to_sec() < wait_time:
        if (int(rospy.Time.now().to_sec()) % 1) == 0:
            rospy.loginfo(
                "Starting Waypoint Publisher in {:.2f} seconds".format(wait_time - rospy.Time.now().to_sec()))

    rospy.sleep(0.1)
    # IMPORTANT WAIT TIME!
    # If this is not here, the "start_time" in the trajectory generator is
    # initialized to zero (because the node has not started fully) and the
    # time for the trajectory will be degenerated

    way_point = AStarGrid(nmap, (0, 3), (6, 13))
    try:
        rospy.loginfo("UAV Waypoint Publisher Created")
        way_point.pub_point()
    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass



