#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from heapq import *
import matplotlib.pyplot as plt
from tf.transformations import quaternion_from_euler


def heuristic(a, b):
    """
    helper function to get distance between A to B
    It can be used as heuristic cost estimate
    """
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)


def reconstruct_path(current, came_from):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return path


def astar(nmap, start, goal):
    close_set = set()
    open_set = []

    # dictionary
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}

    heappush(open_set, (fscore[start], start))

    # while open_set is not empty
    while open_set:

        # the node in openSet having the lowest fscore value
        # remove current in open_set
        current = heappop(open_set)[1]
        if current == goal:
            return reconstruct_path(current, came_from)

        close_set.add(current)
        neighbors = construct_neighbor(current, nmap)

        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            if neighbor in close_set:
                # ignore the neighbor which is already evaluated
                continue

            tentative_g_score = gscore[current] + heuristic(current, neighbor)

            if neighbor not in [i[1] for i in open_set]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heappush(open_set, (fscore[neighbor], neighbor))
            elif tentative_g_score > gscore[neighbor]:
                continue

    return False

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


def construct_neighbor(current, nmap):
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
        if 0 <= neighbor[0] < nmap.shape[0]:
            if 0 <= neighbor[1] < nmap.shape[1]:
                if nmap[neighbor[0]][neighbor[1]] == 1:
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

def extract_viapoint(path, nmap):
    via_point = []
    for point in path:

        is_adjacement = False
        # omit point which adjacent to obstacle
        for i in (-1, 1):
            if 0 <= point[0] + i < nmap.shape[0]:
                if nmap[point[0]+i][point[1]] == 1:
                    is_adjacement = True
                    break
            if 0 <= point[1] + i < nmap.shape[1]:
                if nmap[point[0]][point[1]+i] == 1:
                    is_adjacement = True
                    break
        if is_adjacement is True:
            continue

        is_viapoint = False
        for i in (-1, 1):
            for j in (-1, 1):
                if 0 <= point[0]+i < nmap.shape[0]:
                    if 0 <= point[1]+j < nmap.shape[1]:
                        if nmap[point[0]+i][point[1]+j] == 1:
                            via_point.append(point)
                            is_viapoint = True
                            break
            if is_viapoint is True:
                 break

    return via_point



if __name__ == '__main__':
    start_point = (0, 3)
    final_point = (6, 13)
    path = astar(nmap, (0, 3), (6, 13))
    print path
    path.reverse()
    print path
    via_point = extract_viapoint(path, nmap)
    print via_point
    via_point.insert(0, start_point)
    via_point.append(final_point)
    print via_point
    x = []
    y = []
    for i, j in via_point:
        x.append(i)
        y.append(j)
    print x
    print y
    plt.plot(x, y)
    plt.show()

    z = []
    psi = []
    for i in range(0, len(via_point)):
        z.append(0)
        psi.append(0)

    rospy.init_node('riseq_path_planning', anonymous=True)

    via_pub = rospy.Publisher('riseq/uav_waypoint', Path, queue_size=10)

    point = Path()
    for i in range(0, len(via_point)):
        pose = PoseStamped()
        pose.pose.position.x = x[i]
        pose.pose.position.y = y[i]
        pose.pose.position.z = 0
        q = quaternion_from_euler(0, 0, 0)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.x = q[1]
        pose.pose.orientation.x = q[2]
        pose.pose.orientation.x = q[3]
        point.poses.append(pose)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = ""

        via_pub.publish(point)
        rospy.loginfo(point)
        rate.sleep()



