#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from riseq_trajectory.msg import riseq_uav_trajectory
from riseq_common.msg import riseq_uav_state
import riseq_common.differential_flatness as df
import hovering as hv
import numpy as np
import tf


class TrajectoryGenerator(object):
    def __init__(self, order, time):
        self.order = order
        self.time = time

        self.solution = None
        self.waypoint = None
        self.state = None
        self.m = 0

        # create subscriber for subscribing way point
        self.point_sub = rospy.Subscriber('riseq/planning/uav_waypoint', Path, self.waypoint_update)

        # create subscriber for subscribing state
        self.state_sub = rospy.Subscriber('riseq/tests/uav_fg_true_state', riseq_uav_state, self.state_update)

        # waiting for subscribing first msg
        rospy.sleep(0.1)

        # create publisher for publishing ref trajectory
        self.traj_pub = rospy.Publisher('riseq/trajectory/uav_trajectory', riseq_uav_trajectory, queue_size=10)

        # Our flat output is 4
        self.n = 4

        # initialize time
        # It can be substituted with rospy.get_rostime().secs
        self.start_time = rospy.get_time()
        self.last_time = self.start_time

        # start in trajectory from initial position to final gate
        # start with Index = 0
        # It is index for where drone is now.
        self.index = 0

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
            solution = self.solution[self.n * (self.order + 1) * self.index: self.n * (self.order + 1) * (self.index + 1)]
            ref_trajectory = df.get_trajectory(solution, self.order, self.time[self.index], ref_time)
            # If the time is gone, segment should be changed
            # Index increases.
            if (time - self.last_time) > self.time[self.index]:
                self.index = self.index + 1  # trajectory to next gate
                self.last_time = time
        return ref_trajectory

    def pub_traj(self):
        hz = rospy.get_param('trajectory_update_rate', 200)

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
            rospy.loginfo(traj)
            rate.sleep()

    def waypoint_update(self, msg):
        # m is segment
        self.m = len(msg.poses) - 1
        self.waypoint = np.zeros((self.m + 1, 4))
        for i in range(0, self.m + 1):
            self.waypoint[i][0] = msg.poses[i].pose.position.x
            self.waypoint[i][1] = msg.poses[i].pose.position.y
            self.waypoint[i][2] = msg.poses[i].pose.position.z
            phi, theta, psi = tf.transformations.euler_from_quaternion(
                [msg.poses[i].pose.orientation.x, msg.poses[i].pose.orientation.y, msg.poses[i].pose.orientation.z,
                 msg.poses[i].pose.orientation.w], axes='sxyz')
            self.waypoint[i][3] = psi

    def state_update(self, msg):
        self.state = np.zeros((2, 4))
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        vx, vy, vz = msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z

        ori_quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
                    msg.pose.orientation.w]
        psi, theta, phi = tf.transformations.euler_from_quaternion(ori_quat, axes='rzyx')
        angular_velocity = np.array([[msg.twist.angular.x],[msg.twist.angular.y],[msg.twist.angular.z]])

        matrix = np.array([[1, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
                           [0, np.cos(phi), -1 * np.sin(phi)],
                           [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]])
        phi_dot, theta_dot, psi_dot = np.matmul(matrix, angular_velocity)

        position = np.array([x, y, z, psi])
        velocity = np.array([vx, vy, vz, psi_dot])

        self.state[0] = position
        self.state[1] = velocity