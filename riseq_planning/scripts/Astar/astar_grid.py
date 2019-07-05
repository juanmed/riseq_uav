#!/usr/bin/env python
import rospy
import numpy as np
from heapq import *
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler


start = (0.05, 0.05)
goal = (1.55, -3.05)


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
    neighbors = [(1, 1), (1, -1), (-1, 1), (-1, -1), (0, 1), (0, -1), (1, 0), (-1, 0)]
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


class AStarMap:
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal

        # inflate occupied space as drone's scale
        self.scale = 0.2

        # check a* algorithm can solve problem
        self.is_path = False
        self.path = []

        rospy.Subscriber("occupied_cells_vis_array", MarkerArray, self.astar)

        while not self.is_path:
            rospy.sleep(0.1)

        self.point_pub = rospy.Publisher('riseq/planning/uav_waypoint', Path, queue_size=10)

    def astar(self, msg):
        start_time = rospy.get_time()
        occupied = set()
        for point in msg.markers[16].points:
            if 1.95 <= point.z <= 2.05:
                for i in range(0, int(self.scale * 10) + 1):
                    for j in range(0, int(self.scale * 10) + 1):
                        x = point.x + float(i)/10
                        y = point.y + float(j)/10
                        occupied.add((round(x, 2), round(y, 2)))
                        x = point.x - float(i)/10
                        y = point.y - float(j)/10
                        occupied.add((round(x, 2), round(y, 2)))

        close_set = set()
        open_set = []

        # dictionary
        came_from = {}
        gscore = {start: 0}
        fscore = {start: heuristic(self.start, self.goal)}

        heappush(open_set, (fscore[start], self.start))

        # while open_set is not empty
        while open_set:
            if rospy.get_time() - start_time > 2:
                break

            # the node in openSet having the lowest fscore value
            # remove current in open_set
            current = heappop(open_set)[1]
            if current == goal:
                path = reconstruct_path(current, came_from)
                self.path = path
                self.path.reverse()
                self.waypoint = self.reconstruct_waypoint()
                self.is_path = True
                break

            close_set.add(current)
            neighbors = construct_neighbor(current, occupied)
            if (-0.1, -0.1) in neighbors:
                if (-0.1, 0.0) not in neighbors:
                    print "!!!"
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

    def reconstruct_waypoint(self):
        """
        function to reconstruct way point
        For now, A* algorithm solves path in 2D grid map and ignore heading of drone.
        Because point's form is (x, y), it have to be changed to (x, y, z, psi) form.
        """
        m = len(self.path)
        new_waypoint = np.zeros((m, 4))

        if self.path:
            for i in range(0, m):
                new_waypoint[i][0] = self.path[i][0]  # x
                new_waypoint[i][1] = self.path[i][1]  # y
                new_waypoint[i][2] = 0.3  # z
                new_waypoint[i][3] = 0  # psi

        return new_waypoint

    def pub_point(self):
        hz = 10
        rate = rospy.Rate(hz)

        while not rospy.is_shutdown():
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
            rate.sleep()


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


