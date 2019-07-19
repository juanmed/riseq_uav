#!/usr/bin/env python
import rospy
import numpy as np
from heapq import *
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler


"""
Code Explanation : Construct path from current position in local frame or path in global frame.
                    If drone arrives, construct new path again and again.
"""
# TODO: modify Heuristic and G-score to make optimal path.
#  --> Almost Done, keep considering and developing
# TODO: This code calculate float not integer. I want to change this to integer calculation
#  --> Cannot do integer calculation but now it doesn't matter.
# TODO: in octomap topic, only marker[16] has useful date for my algorithm. Think about it.
#  --> Maybe the smallest marker is only the useful thing for us.


def get_gscore(a, b):
    """
     Helper function to get gscore between A to B
    """
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return np.sqrt(2) * min(dx, dy) + abs(dx - dy)


def get_heuristic(a, b):
    """
     Helper function to get heuristic between A to B

    Euclidean distance np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
    D = 1
    D2 = 1
    Chebyshev distance 1 * (dx + dy) + (D2 - 2 * D) * min(dx, dy)
    """
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return np.sqrt(2) * min(dx, dy) + abs(dx - dy) + 0.1 * (abs(dx - dy))


def reconstruct_path(current, came_from):
    """
     Function to construct path based on A* algorithm in grid map
    """
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return path


def construct_neighbor(current, occupied, resolution, round_scale):
    """
     Function to construct neighbor around current point.
    Ignore occupied space.
    Even also it ignores diagonal space where drone can not fly through.
    """
    neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 0), (0, 1), (1, -1), (1, 0), (1, 1)]
    is_neighbor = np.ones((3, 3), dtype=bool)
    is_neighbor[1, 1] = False

    for i, j in neighbors:
        neighbor = (round(current[0] + i * resolution, round_scale), round(current[1] + j * resolution, round_scale))
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
                new_neighbors.append((i * resolution + resolution, j * resolution - resolution))

    return new_neighbors


def reconstruct_waypoint(path):
    """
     Function to reconstruct way point
    For now, A* algorithm solves path in 2D grid map and ignore heading of drone.
    Because point's form is (x, y), it have to be changed to (x, y, z, psi) form.
    """
    m = len(path)
    waypoint = np.zeros((m, 4))

    if path:
        for i in range(0, m):
            waypoint[i][0] = path[i][0]  # x
            waypoint[i][1] = path[i][1]  # y
            waypoint[i][2] = 1.5  # z
            waypoint[i][3] = 0  # psi

    return waypoint


class AStarMap:
    """
     Class to solve path by using octomap and A* algorithm
    It needs drone pose and octomap topic at first.
    """
    def __init__(self):
        # only for x, y. default is origin
        self.current_position = ()

        # this goal point is random search in default. this can be changed by ros parameter.
        self.goal_point = [(1.00, 0.00), (0.50, -0.50), (0.50, 0.50), (0.00, -1.00), (0.00, 1.00)]

        # last goal when path is updated. It is relative distance from current position
        self.last_goal = (0.05, 0.05)

        # set height at applying A* algorithm
        self.height = 1.5

        # inflate occupied space much as drone's scale
        self.scale = 0.5

        # check a* algorithm can solve problem
        self.waypoint = []

        # set for occupied space
        self.occupied = set()

        # receive parameter. octomap's resolution.
        # resolution should be odd not even like 1, 0.1, 0.01, 0.3
        self.octomap_resolution = rospy.get_param("riseq_estimation_octomap_talker/resolution", 0.1)
        length = len(str(self.octomap_resolution)) - 1
        if int(round((self.octomap_resolution * 10 ** (length - 1)) % 2)) == 0:
            rospy.signal_shutdown("Octomap resolution should be odd not even")
        self.round_scale = len(str(round(self.octomap_resolution / 2, length))) - 2

        # create publisher and subscriber
        self.point_pub = rospy.Publisher('riseq/planning/uav_waypoint', Path, queue_size=10)
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.pose_cb)
        rospy.Subscriber("occupied_cells_vis_array", MarkerArray, self.octomap_cb)

        self.rate = rospy.Rate(2)
        while not self.current_position or not self.occupied:
            print "no subscribe message"
            self.rate.sleep()

    def path_planning(self):
        """
         Once path is constructed, keep following the path,
        but if length of path is 2 or can not construct path because of new obstacle, need to update path
        """
        # If goal_point is None, do not make path
        goal_point = rospy.get_param("riseq/planning/goal_point", None)
        # Define frame of goal point, "local" or "global"
        goal_frame = rospy.get_param("riseq/planning/goal_frame", "local")

        if goal_point is None:
            goal_point = [(1.00, 0.00), (0.50, -0.50), (0.50, 0.50), (0.00, -1.00), (0.00, 1.00)]
        else:
            goal_point = [goal_point]

        if self.goal_point != goal_point:
            self.waypoint = []

        # Once path is constructed, keep following the path
        if len(self.waypoint) > 2 and self.astar(self.last_goal) is True:
            rospy.loginfo("keep following path")
            return True
        # if length of path is 2 or can not construct path because of new obstacle, need to update path
        elif len(self.waypoint) <= 2 or self.astar(self.last_goal) is False:
            for goal in goal_point:
                if goal_frame == "local":
                    new_goal = (round(goal[0] + self.current_position[0], 2), round(goal[1] + self.current_position[1], 2))
                elif goal_frame == "global":
                    new_goal = goal
                if self.astar(new_goal) is True:
                    self.last_goal = new_goal
                    rospy.loginfo("Find path %0.2f %0.2f in %s frame" % (goal[0], goal[1], goal_frame))
                    self.goal_point = goal_point
                    return True

            print "can't find path"
            return False

    def astar(self, goal):
        """
         Usual A* algorithm for finding shortest path
        """
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
                path.reverse()
                self.waypoint = reconstruct_waypoint(path)
                return True

            close_set.add(current)
            neighbors = construct_neighbor(current, self.occupied, self.octomap_resolution, self.round_scale)
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
        """
         Publish way point if path is constructed
        """
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
        """
         Callback function to receive pose topic of drone.
        This is trick for octree map because octree map has 10cm interval and 5cm at origin(this value can be changed)
        """
        if msg.pose.position.x <= round(msg.pose.position.x, self.round_scale - 1):
            x = round(msg.pose.position.x, self.round_scale - 1) - self.octomap_resolution/2
            x = round(x, self.round_scale)
        else:
            x = round(msg.pose.position.x, self.round_scale - 1) + self.octomap_resolution/2
            x = round(x, self.round_scale)

        if msg.pose.position.y <= round(msg.pose.position.y, self.round_scale - 1):
            y = round(msg.pose.position.y, self.round_scale - 1) - self.octomap_resolution/2
            y = round(y, self.round_scale)
        else:
            y = round(msg.pose.position.y, self.round_scale - 1) + self.octomap_resolution/2
            y = round(y, self.round_scale)

        self.current_position = (x, y)

    def octomap_cb(self, msg):
        """
         Receive 17th markers in array
        17th markers have the smallest cube.
        17th markers's scale is what we set already.
        """
        self.occupied = set()
        for point in msg.markers[16].points:
            if round(self.height - self.octomap_resolution/2, self.round_scale) <= point.z <= round(self.height + self.octomap_resolution/2, self.round_scale):
                for i in range(0, int(round(self.scale/self.octomap_resolution) + 1)):
                    for j in range(0, int(round(self.scale/self.octomap_resolution) + 1)):
                        x = point.x + i * self.octomap_resolution
                        y = point.y + j * self.octomap_resolution
                        self.occupied.add((round(x, self.round_scale), round(y, self.round_scale)))
                        x = point.x - i * self.octomap_resolution
                        y = point.y - j * self.octomap_resolution
                        self.occupied.add((round(x, self.round_scale), round(y, self.round_scale)))


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
