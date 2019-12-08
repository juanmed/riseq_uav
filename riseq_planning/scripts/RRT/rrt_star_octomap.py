#!/usr/bin/env python

import rospy
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

import random
import math
import copy
import numpy as np

from pycallgraph import PyCallGraph
from pycallgraph import output
from pycallgraph.output import GraphvizOutput

class Node:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None


class RRT:
    """
    Class for RRT Planning
    """

    def __init__(self, goalSampleRate=20):
        self.current_position = ()
        self.occupied = set()

        self.drone_size = 0.5
        self.height = 1.5
        randArea = [-25, 25]  # ros parameter
        expandDis = 1

        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = 100

        # receive parameter. octomap's resolution.
        self.octomap_resolution = rospy.get_param("riseq_estimation_octomap_talker/resolution", 0.1)

        # create publisher and subscriber
        self.point_pub = rospy.Publisher('riseq/planning/uav_waypoint', Path, queue_size=10)
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.pose_cb)
        rospy.Subscriber("occupied_cells_vis_array", MarkerArray, self.octomap_cb)

        self.rate = rospy.Rate(2)
        while not self.current_position or not self.occupied:
            print "no subscribe message"
            self.rate.sleep()

    def pub_point(self):
        """
         Publish way point if path is constructed
        """
        while not rospy.is_shutdown():
            if self.rrt_star():
                rospy.loginfo("find path")
                # if path is constructed
                point = Path()
                point.header.stamp = rospy.Time.now()
                point.header.frame_id = "world"
                for i in range(0, len(self.waypoint)):
                    pose = PoseStamped()
                    pose.header.frame_id = str(i)
                    pose.pose.position.x = self.waypoint[i][0]
                    pose.pose.position.y = self.waypoint[i][1]
                    pose.pose.position.z = self.height
                    q = quaternion_from_euler(0, 0, 0)
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
        """
        x = msg.pose.position.x
        y = msg.pose.position.y

        self.current_position = (x, y)

    def octomap_cb(self, msg):
        """
         Receive 17th markers in array
        17th markers have the smallest cube.
        17th markers's scale is what we set already.
        """
        self.occupied = set()

        for point in msg.markers[16].points:
            if self.height - self.octomap_resolution/2 <= point.z <= self.height + self.octomap_resolution/2:
                self.occupied.add((point.x, point.y))

    def rrt_star(self, search_until_maxiter=True):
        """
        rrt path planning
        search_until_maxiter: search until max iteration for path improving or not
        """
        self.start = Node(self.current_position[0], self.current_position[1])
        end = rospy.get_param("riseq/planning/goal_point", [10, 0])
        self.end = Node(end[0], end[1])

        self.nodeList = [self.start]
        occupied = self.occupied
        for i in range(self.maxIter):
            rnd = self.get_random_point()
            nind = self.GetNearestListIndex(self.nodeList, rnd)

            new_node = self.steer(rnd, nind)

            if self.__CollisionCheck(new_node, occupied):
                nearinds = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, nearinds, occupied)
                self.nodeList.append(new_node)
                self.rewire(new_node, nearinds, occupied)
            print i

            # generate course
            lastIndex = self.get_best_last_index()
            if lastIndex:
                path = self.gen_final_course(lastIndex)
                path.reverse()
                self.waypoint = path
                return True
        '''    
        # generate course
        lastIndex = self.get_best_last_index()
        if lastIndex:
            path = self.gen_final_course(lastIndex)
            path.reverse()
            self.waypoint = path
            return True
        '''

        # reached max iteration
        rospy.loginfo("can't find path")
        return False

    def choose_parent(self, new_node, nearinds, occupied):
        if not nearinds:
            return new_node

        dlist = []
        for i in nearinds:
            dx = new_node.x - self.nodeList[i].x
            dy = new_node.y - self.nodeList[i].y
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)
            if self.check_collision_extend(self.nodeList[i], theta, d, occupied):
                dlist.append(self.nodeList[i].cost + d)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return new_node

        new_node.cost = mincost
        new_node.parent = minind

        return new_node

    def steer(self, rnd, nind):

        # expand tree
        nearest_node = self.nodeList[nind]
        theta = math.atan2(rnd[1] - nearest_node.y, rnd[0] - nearest_node.x)
        new_node = Node(rnd[0], rnd[1])
        currentDistance = math.sqrt(
            (rnd[1] - nearest_node.y) ** 2 + (rnd[0] - nearest_node.x) ** 2)
        # Find a point within expandDis of nind, and closest to rnd
        if currentDistance <= self.expandDis:
            pass
        else:
            new_node.x = nearest_node.x + self.expandDis * math.cos(theta)
            new_node.y = nearest_node.y + self.expandDis * math.sin(theta)
        new_node.cost = float("inf")
        new_node.parent = None
        return new_node

    def get_random_point(self):

        if random.randint(0, 100) > self.goalSampleRate:
            rnd = [random.uniform(self.minrand, self.maxrand),
                   random.uniform(self.minrand, self.maxrand)]
        else:  # goal point sampling
            rnd = [self.end.x, self.end.y]

        return rnd

    def get_best_last_index(self):

        disglist = [self.calc_dist_to_goal(
            node.x, node.y) for node in self.nodeList]
        goalinds = [disglist.index(i) for i in disglist if i <= self.expandDis]

        if not goalinds:
            return None

        mincost = min([self.nodeList[i].cost for i in goalinds])
        for i in goalinds:
            if self.nodeList[i].cost == mincost:
                return i

        return None

    def gen_final_course(self, goalind):
        path = [[self.end.x, self.end.y]]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            path.append([node.x, node.y])
            goalind = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def find_near_nodes(self, new_node):
        nnode = len(self.nodeList)
        r = 50.0 * math.sqrt(math.log(nnode) / nnode)
        dlist = [(node.x - new_node.x) ** 2 +
                 (node.y - new_node.y) ** 2 for node in self.nodeList]
        nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
        return nearinds

    def rewire(self, new_node, nearinds, occupied):
        nnode = len(self.nodeList)
        for i in nearinds:
            nearNode = self.nodeList[i]

            dx = new_node.x - nearNode.x
            dy = new_node.y - nearNode.y
            d = math.sqrt(dx ** 2 + dy ** 2)

            scost = new_node.cost + d

            if nearNode.cost > scost:
                theta = math.atan2(dy, dx)
                if self.check_collision_extend(nearNode, theta, d, occupied):
                    nearNode.parent = nnode - 1
                    nearNode.cost = scost

    def check_collision_extend(self, nearNode, theta, d, occupied):

        tmpNode = copy.deepcopy(nearNode)

        for i in range(int(d / self.expandDis)):
            tmpNode.x += self.expandDis * math.cos(theta)
            tmpNode.y += self.expandDis * math.sin(theta)
            if not self.__CollisionCheck(tmpNode, occupied):
                return False

        return True

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])
                 ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))

        return minind

    def __CollisionCheck(self, node, occupied):
        """
         Function to check whether obstacle blocks drone's path
        Return 'False' if drone can not pass.
        """

        for ox, oy in occupied:
            dx = ox - node.x
            dy = oy - node.y
            d = dx * dx + dy * dy
            if d <= self.drone_size ** 2:
                return False  # collision

        return True  # safe


if __name__ == '__main__':
    rospy.init_node('riseq_waypoint_publisher', anonymous=True)

    rrt = RRT()

    try:
        rospy.loginfo("UAV Waypoint Publisher Created")
        ### GraphvizOutput (Recommended) : This code shows profile Graphically
        #graphviz = output.GraphvizOutput(output_file='profile.png')
        #with PyCallGraph(output=graphviz):
        rrt.pub_point()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass
