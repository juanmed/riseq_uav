#!/usr/bin/env python
import rospy
import numpy as np
import tf

import moving as mv
import hovering as hv
import differential_flatness as df

from riseq_trajectory.msg import riseq_uav_trajectory


class Hovering:
    """
    Publish trajectory for hovering after simple move.
    At first, drone moves simply like along z-axis or 2 coordinates.
    Then, drone will stop at final point to hover itself.
    """
    def __init__(self):
        # create publisher for publishing ref trajectory
        self.traj_publisher = rospy.Publisher('riseq_uav_trajectory', riseq_uav_trajectory, queue_size=10)

        # 4 output
        # 5th polynomial order
        self.n = 4
        self.order = 5

        # Make two way point for moving simply
        self.init_point = np.array([0, 0, 0, 0])
        self.final_point = np.array([1, 1, 2, 0])

        # Time to take to move.
        self.total_time = 1

        # Get Polynomial which goes upward
        # self.solution = mv.go_upward(self.order, self.total_time, self.final_point[2])

        # Get Polynomial which goes to final point.
        self.solution = mv.go_along(self.order, self.total_time, self.final_point)

        self.start_time = rospy.get_time()
        #self.start_time = rospy.get_rostime().secs

    def compute_reference_traj(self):
        # Compute trajectory at time = now
        time = rospy.get_time()
        ref_time = time - self.start_time

        # stay hover at the final point
        if ref_time >= self.total_time:
            # From hovering.py return ref_trajectory
            ref_trajectory = hv.publish_hovering(self.final_point)

        # In this polynomial, time variable has range [0, 1] at every segment
        # In other word, reference time should be time which is subtracted by last time
        else:
            # In this code, time scaling factor can be ignored  --> 1
            ref_trajectory = df.get_trajectory(self.solution, self.order, 1, ref_time)
        return ref_trajectory

    def pub_traj(self):
        # publish at Hz
        rate = rospy.Rate(200.0)

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
            self.traj_publisher.publish(traj)
            rospy.loginfo(traj)
            rate.sleep()

if __name__ == "__main__":
    # Init Node and Class
    rospy.init_node('riseq_ref_trajectory_publisher', anonymous=True)

    rospy.sleep(0.1)
    # IMPORTANT WAIT TIME!
    # If this is not here, the "start_time" in the trajectory generator is
    # initialized to zero (because the node has not started fully) and the
    # time for the trajectory will be degenerated

    hovering = Hovering()
    try:
        rospy.loginfo("UAV Trajectory Publisher Created")
        hovering.pub_traj()
    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass