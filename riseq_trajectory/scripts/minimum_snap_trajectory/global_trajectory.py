#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from riseq_trajectory.msg import riseq_uav_trajectory
import tf

import riseq_trajectory.draw_trajectory as dt
import riseq_trajectory.hovering as hv
import riseq_common.differential_flatness as df

import numpy as np
import unconstrained_QP as unqp
#import local_replanner as lr


# TODO : Time Optimal


class GlobalTrajectory:
    """
     Publish minimum snap trajectory in global frame.
    It use unconstrained QP so that avoid singular matrix.
    To construct global trajectory, it needs every way point from search based path planning algorithm like A*.
    Once global trajectory is constructed, almost never changed unless local planner cannot find any path.
    Instead, local path planner will re-construct local path.

     2 stage global planner:
    1. Straight-line path using A* or RRT* algorithm
    2. Plan dynamically feasible polynomial trajectory

    Assume that drone start and arrive at rest in global trajectory.
    """
    def __init__(self):
        # Our flat output is 4 [ x y z psi ], but psi is not considered sometimes.
        # It is recommended to adjust 9 order polynomial to construct square matrix for unconstrained QP.
        # unconstrained QP is more stable and faster than typical QP.
        self.order = 9
        self.n = 4

        self.solution = None  # coefficients of polynomial
        self.waypoint = None  # way point which drone needs to traverse
        self.m = 0  # polynomial segment
        self.time = None  # segment time allocation
        self.val = 0  # value of cost function which is minimized by optimization

        # Ros parameter
        rospy.set_param("riseq/global_trajectory_update", True)

        # Create subscriber and publisher
        rospy.Subscriber("riseq/planning/uav_global_waypoint", Path, self.path_cb)
        self.traj_pub = rospy.Publisher('riseq/trajectory/uav_global_trajectory', riseq_uav_trajectory, queue_size=10)

        # Wait until callback function receive way point and compute trajectory solution
        while self.waypoint is None or self.solution is None:
            rospy.sleep(0.01)

        # initialize time
        # It can be substituted with rospy.get_rostime().secs
        self.start_time = rospy.get_time()
        self.last_time = self.start_time

        # start in trajectory from initial position to final position
        # start with Index = 0
        # It is index for where drone is now.
        self.index = 0

    def path_cb(self, msg):
        """
         Callback function that receive path message generate trajectory
        """
        is_update = rospy.get_param("riseq/global_trajectory_update", True)
        if is_update is True:
            # m is segment
            self.m = len(msg.poses) - 1
            self.time = np.ones(self.m)

            self.waypoint = np.zeros((self.m + 1, 4))
            for i in range(0, self.m + 1):
                self.waypoint[i][0] = msg.poses[i].pose.position.x
                self.waypoint[i][1] = msg.poses[i].pose.position.y
                self.waypoint[i][2] = msg.poses[i].pose.position.z
                phi, theta, psi = tf.transformations.euler_from_quaternion(
                    [msg.poses[i].pose.orientation.x, msg.poses[i].pose.orientation.y, msg.poses[i].pose.orientation.z,
                     msg.poses[i].pose.orientation.w], axes='sxyz')
                self.waypoint[i][3] = psi

            self.solution, self.val = unqp.unconstrained_qp(self.waypoint, self.time)

            self.start_time = rospy.get_time()
            self.last_time = self.start_time
            self.index = 0

            rospy.set_param("riseq/global_trajectory_update", False)

            # plot trajectory
            dt.plot_traj3D(self.solution, self.order, self.m, np.delete(self.waypoint, 3, 1))

    def get_global_trajectory(self):
        return self.solution, self.m, self.time

    def compute_reference_traj(self):
        # Compute trajectory at time = now
        time = rospy.get_time()

        # stay hover at the last waypoint position
        if self.index == self.m:
            hovering_point = self.waypoint[-1]
            ref_trajectory = hv.hovering_traj(hovering_point)

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


if __name__ == "__main__":
    # Init Node and Class
    rospy.init_node('riseq_ref_trajectory_publisher', anonymous=True)

    # Wait some time before running. This is to adapt to some simulators
    # which require some 'settling time'

    try:
        wait_time = int(rospy.get_param('riseq/trajectory_wait'))
    except:
        print('riseq/trajectory_wait_time parameter is unavailable')
        print('Setting a wait time of 0 seconds.')
        wait_time = 1

    # wait time for simulator to get ready...
    while rospy.Time.now().to_sec() < wait_time:
        if (int(rospy.Time.now().to_sec()) % 1) == 0:
            rospy.loginfo(
                "Starting Trajectory Generator in {:.2f} seconds".format(wait_time - rospy.Time.now().to_sec()))

    rospy.sleep(0.1)
    # IMPORTANT WAIT TIME!
    # If this is not here, the "start_time" in the trajectory generator is
    # initialized to zero (because the node has not started fully) and the
    # time for the trajectory will be degenerated
    traj_gen = GlobalTrajectory()
    solution, m, time = traj_gen.get_global_trajectory()
    #lr.LocalTrajectory(solution, m, time)

    try:
        rospy.loginfo("UAV Trajectory Publisher Created")
        rospy.sleep(1)
        # traj_gen.pub_traj()
    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass