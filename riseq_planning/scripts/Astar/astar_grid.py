#!/usr/bin/env python
import rospy
import numpy as np
from heapq import *
from visualization_msgs.msg import MarkerArray
from riseq_planning.waypoint_publisher import WayPointPublisher


#TODO : consider drone scale and configurration

start = (0.05, 0.05)
goal = (7.05, -2.05)


def heuristic(a, b):
    """
    helper function to get distance between A to B
    It can be used as heuristic cost estimate
    """
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)


def reconstruct_path(current, came_from):
    """
    function to construct path based on A* algorithm in grid map
    """
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return path


def construct_neighbor(current, occupied):
    """
    function to construct neighbor around current point.
    Ignore wall and occupied space.
    Even also it ignores diagonal space where drone can not fly through.
    """
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    is_neighbor = np.ones((3, 3), dtype=bool)
    is_neighbor[1, 1] = False

    for i, j in neighbors:
        neighbor = (current[0] + float(i)/10, current[1] + float(j)/10)
        if neighbor in occupied:
            is_neighbor[i + 1][j + 1] = False

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
                new_neighbors.append((float(i)/10 - 0.1, float(j)/10 - 0.1))

    return new_neighbors


class AStarMap(WayPointPublisher):
    def __init__(self, start, goal):
        self.waypoint = []
        self.path = []
        self.start = start
        self.goal = goal
        rospy.Subscriber("occupied_cells_vis_array", MarkerArray, self.astar)
        rospy.sleep(1)

        while not self.path:
            # wait until path is constructed completely.
            pass

        super(AStarMap, self).__init__()

    def astar(self, msg):
        occupied = set()
        for point in msg.markers[16].points:
            if 1.45 <= point.z <= 1.55:
                occupied.add((round(point.x, 2), round(point.y, 2)))

        close_set = set()
        open_set = []

        # dictionary
        came_from = {}
        gscore = {start: 0}
        fscore = {start: heuristic(self.start, self.goal)}

        heappush(open_set, (fscore[start], self.start))

        # while open_set is not empty
        while open_set:
            # the node in openSet having the lowest fscore value
            # remove current in open_set
            current = heappop(open_set)[1]
            if current == goal:
                path = reconstruct_path(current, came_from)
                self.path = path
                self.path.reverse()
                self.waypoint = self.path
                self.waypoint = self.reconstruct_waypoint()

            close_set.add(current)
            neighbors = construct_neighbor(current, occupied)
            for i, j in neighbors:
                neighbor = round(current[0] + i, 2), round(current[1] + j, 2)
                if neighbor in close_set:
                    # ignore the neighbor which is already evaluated
                    continue

                tentative_g_score = gscore[current] + heuristic(current, neighbor)

                if neighbor not in [i[1] for i in open_set] or tentative_g_score < gscore[neighbor]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + heuristic(neighbor, self.goal)
                    heappush(open_set, (fscore[neighbor], neighbor))
        print "Fail"

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
            new_waypoint[i][2] = 1.5  # z
            new_waypoint[i][3] = 0  # psi

        return new_waypoint

if __name__ == '__main__':
    rospy.init_node('riseq_waypoint_publisher', anonymous=True)

    # Wait some time before running. This is to adapt to some simulators
    # which require some 'settling time'

    try:
        wait_time = int(rospy.get_param('riseq/planning_wait'))
    except:
        print('riseq/planning_wait_time parameter is unavailable')
        print('Setting a wait time of 2 seconds.')
        wait_time = 2

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

    way_point = AStarMap(start, goal)
    try:
        rospy.loginfo("UAV Waypoint Publisher Created")
        way_point.pub_point()
    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass


