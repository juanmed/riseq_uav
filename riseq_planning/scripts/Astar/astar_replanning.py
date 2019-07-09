#!/usr/bin/env python
import rospy
import numpy as np
from heapq import *
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

"""
Code Explanation : Construct path forward or side from current position
                    If drone arrives, construct new path again and again.
"""
# TODO: modify Heuristic and G-score to make optimal path.
# TODO: This code calculate float not integer. I want to change this to integer calculation
# TODO: in octomap topic, only marker[16] has useful date for my algorithm. Think about it.


def get_gscore(a, b):
    """
    helper function to get gscore between A to B
    """
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return np.sqrt(2) * min(dx, dy) + abs(dx - dy)


def get_heuristic(a, b):
    """
    helper function to get heuristic between A to B
    """
    # Euclidean distance
    # return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return 1.4 * min(dx, dy) + abs(dx - dy) + 0.1 * (abs(dx - dy))
    #D = 1
    #D2 = 1
    # Chebyshev distance
    #return 1 * (dx + dy) + (D2 - 2 * D) * min(dx, dy)


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
    neighbors = [(1, 1), (1, -1), (-1, 1), (-1, -1), (0, 1), (0, -1), (1, 0), (-1, 0)]
    is_neighbor = np.ones((3, 3), dtype=bool)
    is_neighbor[1, 1] = False

    for i, j in neighbors:
        neighbor = (round(current[0] + float(i)/10, 2), round(current[1] + float(j)/10, 2))
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


def reconstruct_waypoint(path):
    """
    function to reconstruct way point
    For now, A* algorithm solves path in 2D grid map and ignore heading of drone.
    Because point's form is (x, y), it have to be changed to (x, y, z, psi) form.
    """
    m = len(path)
    waypoint = np.zeros((m, 4))

    if path:
        for i in range(0, m):
            waypoint[i][0] = path[i][0]  # x
            waypoint[i][1] = path[i][1]  # y
            waypoint[i][2] = 1  # z
            waypoint[i][3] = 0  # psi

    return waypoint


class AStarMap:
    def __init__(self):
        # only for x, y
        self.current_position = (0.05, 0.05)

        # last goal when path is updated. It is relative distance from current position
        self.last_goal = (0.0, 0.0)

        # inflate occupied space much as drone's scale
        self.scale = 0.5

        # check a* algorithm can solve problem
        self.path = []

        # set for occupied space
        self.occupied = set()

        self.point_pub = rospy.Publisher('riseq/planning/uav_waypoint', Path, queue_size=10)
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.pose_cb)
        rospy.Subscriber("occupied_cells_vis_array", MarkerArray, self.octomap_cb)
        print "Publisher and Subscriber completed"

        self.rate = rospy.Rate(2)
        while not self.current_position or not self.occupied:
            print "no subscribe message"
            self.rate.sleep()

    def octomap_cb(self, msg):
        self.occupied = set()
        for point in msg.markers[16].points:
            if 0.95 <= point.z <= 1.05:
                for i in range(0, int(self.scale * 10) + 1):
                    for j in range(0, int(self.scale * 10) + 1):
                        x = point.x + float(i)/10
                        y = point.y + float(j)/10
                        self.occupied.add((round(x, 2), round(y, 2)))
                        x = point.x - float(i)/10
                        y = point.y - float(j)/10
                        self.occupied.add((round(x, 2), round(y, 2)))

    def path_planning(self):
        #goal_array = [(1.00, 0.00), (0.50, -0.50), (0.50, 0.50), (0.00, -1.00), (0.00, 1.00)]
        goal_array = [(20.00, -1.00)]

        # if length of path is 2, need to update path
        if len(self.path) > 2 and self.astar(self.last_goal) is True:
            print "keep going"
            return True
        elif len(self.path) <= 2 or self.astar(self.last_goal) is False:
            for goal in goal_array:
                new_goal = (round(goal[0] + self.current_position[0], 2), round(goal[1] + self.current_position[1], 2))
                if self.astar(new_goal) is True:
                    self.last_goal = new_goal
                    rospy.loginfo("Find path %0.2f %0.2f" % (goal[0], goal[1]))
                    return True

            print "can't find path"
            return False

    def astar(self, goal):
        current_position = self.current_position
        start_time = rospy.get_time()

        close_set = set()
        open_set = []

        # dictionary
        came_from = {}
        gscore = {current_position: 0}
        fscore = {current_position: get_heuristic(current_position, goal)}

        heappush(open_set, (fscore[current_position], current_position))

        # while open_set is not empty
        while open_set:
            if rospy.get_time() - start_time > 2:
                return False

            # the node in openSet having the lowest fscore value
            # remove current in open_set
            current = heappop(open_set)[1]
            if current == goal:
                path = reconstruct_path(current, came_from)
                self.path = path
                self.path.reverse()
                self.waypoint = reconstruct_waypoint(self.path)
                return True

            close_set.add(current)
            neighbors = construct_neighbor(current, self.occupied)
            for i, j in neighbors:
                neighbor = round(current[0] + i, 2), round(current[1] + j, 2)
                if neighbor in close_set:
                    # ignore the neighbor which is already evaluated
                    continue

                tentative_g_score = gscore[current] + get_gscore(current, neighbor)

                if neighbor not in [i[1] for i in open_set] or tentative_g_score < gscore[neighbor]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + get_heuristic(neighbor, goal)
                    heappush(open_set, (fscore[neighbor], neighbor))

    def pub_point(self):
        while not rospy.is_shutdown():
            if self.path_planning() is True:
                # if path is constructed
                point = Path()
                point.header.stamp = rospy.Time.now()
                point.header.frame_id = "map"
                for i in range(0, len(self.waypoint)):
                    pose = PoseStamped()
                    pose.header.frame_id = str(i)
                    pose.pose.position.x = self.waypoint[i][0]
                    pose.pose.position.y = self.waypoint[i][1]
                    pose.pose.position.z = self.waypoint[i][2]
                    q = quaternion_from_euler(0, 0, self.waypoint[i][3])
                    pose.pose.orientation.x = q[0]
                    pose.pose.orientation.y = q[1]
                    pose.pose.orientation.z = q[2]
                    pose.pose.orientation.w = q[3]
                    point.poses.append(pose)

                self.point_pub.publish(point)
                self.rate.sleep()

    def pose_cb(self, msg):
        # This is trick for octree map because octree map has 10cm interval and 5cm at origin
        self.current_position = (round(round(msg.pose.position.x, 1) + 0.05, 2), round(round(msg.pose.position.y, 1) + 0.05, 2))


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
    '''
    # wait time for simulator to get ready...
    while rospy.Time.now().to_sec() < wait_time:
        if (int(rospy.Time.now().to_sec()) % 1) == 0:
            rospy.loginfo(
                "Starting Waypoint Publisher in {:.2f} seconds".format(wait_time - rospy.Time.now().to_sec()))
    '''

    way_point = AStarMap()
    try:
        rospy.loginfo("UAV Waypoint Publisher Created")
        way_point.pub_point()
    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass
