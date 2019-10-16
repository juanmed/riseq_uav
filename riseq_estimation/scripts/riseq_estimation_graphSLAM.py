#!/usr/bin/env python
"""
author:  Eugene Auh
version: 0.1.0
brief: Graph-based SLAM for IROS2019 ADR Competition. From the location of the gate, compensates drifts of VIO.
      Algorithm and code are based on PythonRobotics Pose Optimization SLAM 2D by Atsushi Sakai.
                                    (https://hithub.com/AtsushiSakai/PythonRobotics#pose-optimization-slam-2d)

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy of this
software and associated documentation files (the ""Software""), to deal in the 
Software without restriction, including without limitation the rights to use, copy, 
modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, 
and to permit persons to whom the Software is furnished to do so, subject to the 
following conditions:
The above copyright notice and this permission notice shall be included in all copies 
or substantial portions of the Software.
THE SOFTWARE IS PROVIDED *AS IS*, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF 
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE 
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

import sys
import time
import numpy as np
from scipy import sparse
from scipy.sparse import linalg
import matplotlib.pyplot as plt
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from eugene_kinematics import q2r


class TripletList:
    def __init__(self):
        self.row = []
        self.col = []
        self.data = []

    def push_back(self, irow, icol, idata):
        self.row.append(irow)
        self.col.append(icol)
        self.data.append(idata)


class Pose2D:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta


class Constraint2D:
    def __init__(self, id1, id2, t, info_mat):
        self.id1 = id1
        self.id2 = id2
        self.t = t
        self.info_mat = info_mat


class Optimizer2D:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('riseq_estimation_graph_SLAM')
        self.frequency = 1.0
        self.r = rospy.Rate(self.frequency)

        # Set Graph-SLAM parameters
        self.max_iter = 20
        self.min_iter = 3
        self.verbose = True
        self.animation = True
        self.p_lambda = 1e-6
        self.init_w = 1e10
        self.stop_thre = 1e-3
        self.dim = 3  # state dimension

        # Initialize nodes and edge with gate's pose and drone's initial pose, drift
        self.nodes, self.consts = [], []
        self.gate1_pose = Pose2D(5.0, 0.0, 0.0)
        self.gate1_id = 0
        self.nodes.append(self.gate1_pose)
        self.nodes.append(Pose2D(0.0, 0.0, 0.0))
        self.consts.append(Constraint2D(1, 0, self.gate1_pose, np.array([[self.init_w, 0.0, 0.0],
                                                                         [0.0, self.init_w, 0.0],
                                                                         [0.0, 0.0, self.init_w]])))
        self.x_drift = 0.0
        self.y_drift = 0.0

        # Initialize drone pose and time, id
        self.last_gate_time = rospy.Time.now()
        self.last_pose_time = rospy.Time.now()
        self.last_pose = PoseStamped()
        self.cur_id = 0
        self.cur_pose = PoseStamped()

        # Publisher, Subscriber
        self.pose_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
        rospy.Subscriber('/zed/zed_node/pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/riseq/perception/uav_mono_waypoint', Path, self.gate_cb)

    def loop(self):
        # Optimize nodes
        final_nodes = self.optimize_path(self.nodes, self.consts, self.max_iter, self.min_iter)
        self.x_drift = self.cur_pose.pose.position.x - final_nodes[len(final_nodes)-1].x
        self.y_drift = self.cur_pose.pose.position.y - final_nodes[len(final_nodes)-1].y

        # Plot result
        # plt.cla()
        # plot_nodes(self.nodes, color="-b", label="before")
        # plot_nodes(final_nodes, label="after")
        # plt.axis("equal")
        # plt.grid(True)
        # plt.legend()

        self.r.sleep()

    def pose_cb(self, msg):
        # Add pose-pose node and contrant, Update the last pose to calculate odometry information
        if (rospy.Time.now().to_sec() - self.last_pose_time.to_sec()) >= (1.0/self.frequency):
            self.nodes.append(Pose2D(msg.pose.position.x, msg.pose.position.y, 0.0))
            id1 = self.cur_id
            id2 = self.cur_id + 1
            t = Pose2D(msg.pose.position.x - self.last_pose.pose.position.x, msg.pose.position.y - self.last_pose.pose.position.y, 0.0)
            info_mat = np.array([[1.0, 0.0, 0.0],
                                 [0.0, 1.0, 0.0],
                                 [0.0, 0.0, self.init_w]])
            self.consts.append(Constraint2D(id1, id2, t, info_mat))
            self.cur_id += 1
            self.last_pose_time = rospy.Time.now()
            self.last_pose.pose.position.x = msg.pose.position.x
            self.last_pose.pose.position.y = msg.pose.position.y
            self.last_pose.pose.position.z = msg.pose.position.z

        # Publish compensated visual odometry position
        self.cur_pose.pose.position.x = msg.pose.position.x - self.x_drift
        self.cur_pose.pose.position.y = msg.pose.position.y - self.y_drift
        self.cur_pose.pose.position.z = msg.pose.position.z
        self.cur_pose.pose.orientation.x = msg.pose.orientation.x
        self.cur_pose.pose.orientation.y = msg.pose.orientation.y
        self.cur_pose.pose.orientation.z = msg.pose.orientation.z
        self.cur_pose.pose.orientation.w = msg.pose.orientation.w
        self.cur_pose.header.stamp = rospy.Time.now()
        self.pose_pub.publish(self.cur_pose)

    def gate_cb(self, msg):
        # Add pose-pose node and contraint, Update the last pose to calculate odometry information
        if rospy.Duration(self.last_gate_time, rospy.Time.now()) >= (1.0/self.frequency):
            self.nodes.append(Pose2D(self.cur_pose.pose.position.x, self.cur_pose.pose.position.y, 0.0))
            id1 = self.cur_id
            id2 = self.cur_id + 1
            t = Pose2D(self.cur_pose.pose.position.x - self.last_pose.pose.position.x, self.cur_pose.pose.position.y - self.last_pose.pose.position.y, 0.0)
            info_mat = np.array([[1.0, 0.0, 0.0],
                                 [0.0, 1.0, 0.0],
                                 [0.0, 0.0, self.init_w]])
            self.consts.append(Constraint2D(id1, id2, t, info_mat))
            self.cur_id += 1
            self.last_pose_time = rospy.Time.now()
            self.last_pose.pose.position.x = self.cur_pose.pose.position.x
            self.last_pose.pose.position.y = self.cur_pose.pose.position.y
            self.last_pose.pose.position.z = self.cur_pose.pose.position.z

            # Add pose-landmark node and constraint
            R = q2r(self.cur_pose.pose.orientation.w, self.cur_pose.pose.orientation.x, self.cur_pose.pose.orientation.y, self.cur_pose.pose.orientation.z)
            p = np.dot(R, np.array([[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z]]))
            self.nodes.append(Pose2D(p[0][0], p[1][0], 0.0))
            id1 = self.cur_id
            id2 = self.gate1_id
            t = Pose2D(msg.poses[0].position.x, msg.poses[0].position.y, 0.0)
            info_mat = np.array([[1.0, 0.0, 0.0],
                                 [0.0, 1.0, 0.0],
                                 [0.0, 0.0, self.init_w]])
            self.consts.append(Constraint2D(id1, id2, t, info_mat))
            self.last_gate_time = rospy.Time.now()

    def optimize_path(self, nodes, consts, max_iter, min_iter):
        graph_nodes = nodes[:]
        prev_cost = sys.float_info.max

        for i in range(max_iter):
            start = time.time()
            cost, graph_nodes = self.optimize_path_one_step(graph_nodes, consts)
            elapsed = time.time() - start
            if self.verbose:
                print("step ", i, " cost: ", cost, " time:", elapsed, "s")

            # check convergence
            if (i > min_iter) and (prev_cost - cost < self.stop_thre):
                if self.verbose:
                    print("converged:", prev_cost - cost, " < ", self.stop_thre)
                    break
            prev_cost = cost

            if self.animation and (i == 0 or i == max_iter-1):
                plt.cla()
                plot_nodes(nodes, color="-b")
                plot_nodes(graph_nodes)
                plt.axis("equal")

        return graph_nodes

    def optimize_path_one_step(self, graph_nodes, constraints):
        indlist = [i for i in range(self.dim)]
        numnodes = len(graph_nodes)
        bf = np.zeros(numnodes * self.dim)
        tripletList = TripletList()

        for con in constraints:
            ida = con.id1
            idb = con.id2
            assert 0 <= ida and ida < numnodes, "ida is invalid"
            assert 0 <= idb and idb < numnodes, "idb is invalid"
            r, Ja, Jb = self.calc_error(
                graph_nodes[ida], graph_nodes[idb], con.t)

            trJaInfo = np.dot(Ja.transpose(), con.info_mat)
            trJaInfoJa = np.dot(trJaInfo, Ja)
            trJbInfo = np.dot(Jb.transpose(), con.info_mat)
            trJbInfoJb = np.dot(trJbInfo, Jb)
            trJaInfoJb = np.dot(trJaInfo, Jb)

            # Update information matrix
            for k in indlist:
                for m in indlist:
                    tripletList.push_back(ida * self.dim + k, ida * self.dim + m, trJaInfoJa[k, m])
                    tripletList.push_back(idb * self.dim + k, idb * self.dim + m, trJbInfoJb[k, m])
                    tripletList.push_back(ida * self.dim + k, idb * self.dim + m, trJaInfoJb[k, m])
                    tripletList.push_back(idb * self.dim + k, ida * self.dim + m, trJaInfoJb[m, k])

            bf[ida * self.dim: ida * self.dim + 3] += np.dot(trJaInfo, r)
            bf[idb * self.dim: idb * self.dim + 3] += np.dot(trJbInfo, r)

        # Fix first node
        for k in indlist:
            tripletList.push_back(k, k, self.init_w)

        for i in range(self.dim * numnodes):
            tripletList.push_back(i, i, self.p_lambda)

        mat = sparse.coo_matrix((tripletList.data, (tripletList.row, tripletList.col)), shape=(numnodes * self.dim, numnodes * self.dim))
        x = linalg.spsolve(mat.tocsr(), -bf)

        out_nodes = []
        for i in range(len(graph_nodes)):
            u_i = i * self.dim
            pos = Pose2D(graph_nodes[i].x + x[u_i],
                         graph_nodes[i].y + x[u_i + 1],
                         graph_nodes[i].theta + x[u_i + 2])
            out_nodes.append(pos)

        cost = self.calc_global_cost(out_nodes, constraints)

        return cost, out_nodes

    def calc_global_cost(self, nodes, constraints):
        cost = 0.0
        for c in constraints:
            diff = self.error_func(nodes[c.id1], nodes[c.id2], c.t)
            cost += np.linalg.multi_dot([diff.transpose(), c.info_mat, diff])
        return cost

    def error_func(self, pa, pb, t):
        ba = self.calc_constraint_pose(pb, pa)
        error = np.array([ba.x - t.x,
                          ba.y - t.y,
                          self.pi2pi(ba.theta - t.theta)])
        return error

    def calc_constraint_pose(self, l, r):
        diff = np.array([l.x - r.x, l.y - r.y, l.theta - r.theta])
        v = np.dot(self.rot_mat_2d(-r.theta), diff)
        v[2] = self.pi2pi(l.theta - r.theta)
        return Pose2D(v[0], v[1], v[2])

    def rot_mat_2d(self, theta):
        return np.array([[np.cos(theta), -np.sin(theta), 0.0],
                         [np.sin(theta), np.cos(theta), 0.0],
                         [0.0, 0.0, 1.0]])

    def calc_error(self, pa, pb, t):
        e0 = self.error_func(pa, pb, t)
        dx = pb.x - pa.x
        dy = pb.y - pa.y
        dxdt = -np.sin(pa.theta) * dx + np.cos(pa.theta) * dy
        dydt = -np.cos(pa.theta) * dx - np.sin(pa.theta) * dy
        Ja = np.array([[-np.cos(pa.theta), -np.sin(pa.theta), dxdt],
                       [np.sin(pa.theta), -np.cos(pa.theta), dydt],
                       [0.0, 0.0, -1.0]])
        Jb = np.array([[np.cos(pa.theta), np.sin(pa.theta), 0.0],
                       [-np.sin(pa.theta), np.cos(pa.theta), 0.0],
                       [0.0, 0.0, 1.0]])
        return e0, Ja, Jb

    def pi2pi(self, rad):
        val = np.fmod(rad, 2.0 * np.pi)
        if val > np.pi:
            val -= 2.0 * np.pi
        elif val < -np.pi:
            val += 2.0 * np.pi
        return val


def plot_nodes(nodes, color ="-r", label = ""):
    x, y = [], []
    for n in nodes:
        x.append(n.x)
        y.append(n.y)
    plt.plot(x, y, color, label=label)


if __name__ == "__main__":
    try:
        optimizer = Optimizer2D()
        while not rospy.is_shutdown():
            optimizer.loop()

    except rospy.ROSInterruptException:
        pass
