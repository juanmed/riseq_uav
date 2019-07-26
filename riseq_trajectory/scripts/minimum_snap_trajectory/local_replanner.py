#!/usr/bin/env python

#############################
# [1] Helen Oleynikova, Michael Burri ... ETH zurich
# Continuous-Time Trajectory Optimization for Online UAV Replanning
#
# [2] Vladyslav Usenko ... Unversity of Munich
# Real-Time Trajectory Replanning for MAVs using Uniform B-splines and a 3D Circular Buffer
#
#############################

import rospy
from nav_msgs.msg import Path
from riseq_trajectory.msg import riseq_uav_trajectory
import riseq_common.differential_flatness as df
import tf
import unconstarined_QP as unqp
import numpy as np


# TODO : Formulating cost function for local trajectory
# TODO : Optimization method (eg. gradient method, Newton method, linear programming, quadratic programming...)
# TODO : time optimal

class LocalTrajectory:
    """
     Class for local re-planner

     As start point, we choose the point on the current trajectory tR seconds in the future, where tR is
    the update rate of the replanner.[1]
     The goal point is chosen as a point on the global trajectory that is h meters ahead of the start point, where h is
    planning horizon. If unoccupied, we accept that the point as the goal, otherwise we attempt to find the nearest
    unoccupied neighbor in the ESDF.[1]
     We initialize our replanning algorithm with six fixed control points at the beginning of the global trajectory[2]
    """

    def __init__(self, global_solution, global_m, global_time):
        # Our flat output is 4 [ x y z psi ], but psi is not considered sometimes.
        # It is recommended to adjust 9 order polynomial to construct square matrix for unconstrained QP.
        # unconstrained QP is more stable and faster than typical QP.
        self.order = 9
        self.n = 4

        #
        self.global_solution = global_solution
        self.global_m = global_m
        self.global_time = global_time

        self.solution = None
        self.waypoint = None
        self.time = None
        self.m = 0

        # Create subscriber and publisher
        rospy.Subscriber("riseq/planning/uav_local_waypoint", Path, self.path_cb)
        self.traj_pub = rospy.Publisher('riseq/trajectory/uav_local_trajectory', riseq_uav_trajectory, queue_size=10)

        # Wait until callback function receive way point and compute trajectory solution
        while self.waypoint is None or self.solution is None:
            rospy.sleep(0.01)

        # As start point, we choose the point on the current trajectory tR seconds in the future, where tR is
        # the update rate of the re-planner.[1]
        self.update_rate = 0.5

        # The goal point is chosen as a point on the global trajectory that is h meters ahead of the start point,
        # where h is planning horizon
        self.ahead_distance = 4

        # TODO: ros parameter control point
        # TODO: time segment
        # I set control point as 6, which means that drone will pass 6 way points in re-planning path
        # For now, each segments between control points have 1 seconds
        self.control_point = 6

        # initialize time
        # It can be substituted with rospy.get_rostime().secs
        self.start_time = rospy.get_time()
        self.last_time = self.start_time

        # start in trajectory from initial position to final position
        # start with Index = 0
        # It is index for where drone is now.
        self.index = 0

    def extract_start_state(self):
        """
         Function to extract future state of trajectory in tR seconds
        This is important to guarantee continuity of re-planning path
        """
        time = rospy.get_time()
        ref_time = time - self.last_time
        future_time = ref_time + self.update_rate

        return df.compute_output3D(self.solution, self.order, self.time[self.index], future_time)

    def extract_goal_state(self):
        """
         Function to extract goal state of trajectory
        The goal point of Re-planning path would be better to be near from global trajectory
        """
        time = rospy.get_time()
        ref_time = time - self.last_time
        future_time = ref_time + self.update_rate

        # get state of future time in global trajectory
        return df.compute_output3D(self.global_solution, self.order, self.time[self.future_index], future_time)

    def replanning_path(self):
        """
         Function to re-planning path
        It needs start point and goal point of path.
        Also needs way points which drone will traverse.
        In this procedure, solution is computed by optimization
        """
        start_state = self.extract_start_state()
        goal_state = self.extract_goal_state()



    def path_cb(self, msg):
        """
         Callback function that receive path message generate trajectory
        """
        # m is segment
        self.m = len(msg.poses) - 1
        self.time = np.ones(self.m)

        self.waypoint = np.zeros((self.m + 1, 4))
        for i in range(0, self.m + 1):
            self.waypoint[i][0] = msg.poses[i].pose.position.x
            self.waypoint[i][1] = msg.poses[i].pose.position.y
            self.waypoint[i][2] = msg.poses[i].pose.position.z
            phi, theta, psi = tf.transformations.euler_from_quaternion(
                [msg.poses[i].pose.orientation.x, msg.poses[i].pose.orientation.y,
                 msg.poses[i].pose.orientation.z,
                 msg.poses[i].pose.orientation.w], axes='sxyz')
            self.waypoint[i][3] = psi

        solution, val = unqp.unconstrained_qp(self.waypoint, self.time)
        self.solution = solution

        self.start_time = rospy.get_time()
        self.last_time = self.start_time
        self.index = 0

    def compute_reference_traj(self):
        # Compute trajectory at time = now
        time = rospy.get_time()

        # stay hover at the last waypoint position
        if self.index == self.m:
            hovering_point = self.waypoint[-1]
            ref_trajectory = hv.hovering_traj(hovering_point)
            # TODO: local planner hovering -> need to generate new global trajectory

        # In other word, reference time should be time which is subtracted by last time
        else:
            ref_time = time - self.last_time
            solution = self.solution[
                       self.n * (self.order + 1) * self.index: self.n * (self.order + 1) * (self.index + 1)]
            ref_trajectory = df.get_trajectory(solution, self.order, self.time[self.index], ref_time)
            # If the time is gone, segment should be changed
            # Index increases.
            if (time - self.last_time) > self.time[self.index]:
                self.index = self.index + 1  # trajectory to next gate
                self.last_time = time
        return ref_trajectory

    def pub_traj(self):
        hz = rospy.get_param('riseq/trajectory_update_rate', 100)

        # publish at Hz
        rate = rospy.Rate(hz)

        while not rospy.is_shutdown():
            ref_traj = self.compute_reference_traj()

            # create and fill message
            traj = riseq_uav_trajectory()
            traj.header.stamp = rospy.Time.now()
            traj.header.frame_id = ""

            x, y, z = ref_traj[0]
            vx, vy, vz = ref_traj[1]
            ax, ay, az = ref_traj[10]
            jx, jy, jz = ref_traj[11]
            sx, sy, sz = ref_traj[12]

            phi, theta, psi = ref_traj[2]
            p, q, r = ref_traj[3]
            yaw = ref_traj[13]
            yawdot = ref_traj[14]
            yawddot = ref_traj[15]

            uax, uay, uaz = ref_traj[4]
            ubx, uby, ubz = ref_traj[5]
            ucx, ucy, ucz = ref_traj[6]
            Rbw = np.array(ref_traj[9]).flatten().tolist()

            # Input (T, M) publisher to be used in estimation
            u_1 = ref_traj[7]
            u_xx, u_xy, u_xz = ref_traj[8]

            traj.pose.position.x = x
            traj.pose.position.y = y
            traj.pose.position.z = z

            # Following http://docs.ros.org/jade/api/tf/html/python/transformations.html
            # the axes parameter is such that
            # r = apply rotation on the "new" frame
            # zyx = this means first a rotation of 'psi' radians around the z axis is performed,
            #       then of 'theta' radians about the new y axis ( because 'r' was specified)
            #       then of 'phi' radians about the new x axis ( becuase 'r' was specified)
            quat = tf.transformations.quaternion_from_euler(psi, theta, phi, axes='rzyx')
            traj.pose.orientation.x = quat[0]
            traj.pose.orientation.y = quat[1]
            traj.pose.orientation.z = quat[2]
            traj.pose.orientation.w = quat[3]

            traj.twist.linear.x = vx
            traj.twist.linear.y = vy
            traj.twist.linear.z = vz

            traj.twist.angular.x = p
            traj.twist.angular.y = q
            traj.twist.angular.z = r

            traj.ua.x = uax
            traj.ua.y = uay
            traj.ua.z = uaz

            traj.ub.x = ubx
            traj.ub.y = uby
            traj.ub.z = ubz

            traj.uc.x = ucx
            traj.uc.y = ucy
            traj.uc.z = ucz

            traj.rot = Rbw

            traj.acc.x = ax
            traj.acc.y = ay
            traj.acc.z = az

            traj.jerk.x = jx
            traj.jerk.y = jy
            traj.jerk.z = jz

            traj.snap.x = sx
            traj.snap.y = sy
            traj.snap.z = sz

            traj.yaw = yaw
            traj.yawdot = yawdot
            traj.yawddot = yawddot

            # publish message
            self.traj_pub.publish(traj)
            #rospy.loginfo(traj)
            rate.sleep()