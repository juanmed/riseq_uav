#!/usr/bin/env python
import rospy
from heapq import *
import numpy as np
import pyvisgraph as vg
from pyvisgraph.graph import Graph, Edge
from pyvisgraph.visible_vertices import visible_vertices
from riseq_planning.waypoint_publisher import WayPointPublisher


class AStarGraph(WayPointPublisher):
    """
    Class to solve A* algorithm based on graph and publish way point
    """
    def __init__(self, polys, origin, destination):
        n = len(polys)
        m = len(polys[0])
        self.polys = [[0] * m for i in range(n)]
        for i in range(n):
            for j in range(m):
                self.polys[i][j] = vg.Point(polys[i][j][0], polys[i][j][1])

        self.origin = vg.Point(origin[0], origin[1])
        self.destination = vg.Point(destination[0], destination[1])
        self.g = vg.VisGraph()
        self.g.build(self.polys)
        self.path = self.construct_path()

        self.waypoint = []
        for i in range(len(self.path)):
            self.waypoint.append([self.path[i].x, self.path[i].y, 0, 0])

        super(AStarGraph, self).__init__(self.waypoint)

    def construct_path(self):
        origin_exists = self.origin in self.g.visgraph
        dest_exists = self.destination in self.g.visgraph
        if origin_exists and dest_exists:
            return self.shortest_path(self.g.visgraph, self.origin, self.destination)
        orgn = None if origin_exists else self.origin
        dest = None if dest_exists else self.destination
        add_to_visg = Graph([])
        if not origin_exists:
            for v in visible_vertices(self.origin, self.g.graph, destination=dest):
                add_to_visg.add_edge(Edge(self.origin, v))
        if not dest_exists:
            for v in visible_vertices(self.destination, self.g.graph, origin=orgn):
                add_to_visg.add_edge(Edge(self.destination, v))
        return self.shortest_path(self.g.visgraph, self.origin, self.destination, add_to_visg)

    def heuristic(self, a, b):
        """
        helper function to get distance between A to B
        It can be used as heuristic cost estimate
        """
        return np.sqrt((b.x - a.x) ** 2 + (b.y - a.y) ** 2)

    def astar(self, graph, origin, destination, add_to_visgraph):
        """
        function to solve a* algorithm based on graph
        """
        close_set = set()
        open_set = []

        # dictionary
        came_from = {}
        gscore = {origin: 0}
        fscore = {origin: self.heuristic(origin, destination)}

        # priority queue, push key and value
        heappush(open_set, (fscore[origin], origin))

        count = 0
        # while open_set is not empty
        while open_set:

            # the node in openSet having the lowest fscore
            # remove current in open_set
            current = heappop(open_set)[1]
            count = count + 1
            if current == destination:
                return came_from

            close_set.add(current)

            edges = graph[current]
            if add_to_visgraph is not None and len(add_to_visgraph[current]) > 0:
                edges = add_to_visgraph[current] | graph[current]
            for e in edges:
                neighbor = e.get_adjacent(current)
                if neighbor in close_set:
                    # ignore the neighbor which is already evaluated
                    continue

                tentative_g_score = gscore[current] + self.heuristic(current, neighbor)

                if neighbor not in [i[1] for i in open_set] or tentative_g_score < gscore[neighbor]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, destination)
                    heappush(open_set, (fscore[neighbor], neighbor))

    def shortest_path(self, graph, origin, destination, add_to_visgraph=None):
        came_from = self.astar(graph, origin, destination, add_to_visgraph)
        path = []
        while 1:
            path.append(destination)
            if destination == origin: break
            destination = came_from[destination]
        path.reverse()
        return path


polys = [[(0.0, 1.0), (3.0, 1.0), (1.5, 4.0)],
         [(4.0, 4.0), (7.0, 4.0), (5.5, 8.0)]]
origin = (1.5, 0.0)
destination = (4.0, 6.0)

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

    way_point = AStarGraph(polys, (1.5, 0.0), (4.0, 6.0))
    try:
        rospy.loginfo("UAV Waypoint Publisher Created")
        way_point.pub_point()
    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass

