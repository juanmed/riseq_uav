#!/usr/bin/env python
import rospy
import numpy as np
import tf

import get_solution as gs
import riseq_common.differential_flatness as df

from riseq_trajectory.msg import riseq_uav_trajectory


class TrajectoryGenerator:
    """
    Publish trajectory for hovering after simple move.
    At first, drone moves simply like along z-axis or 2 coordinates.
    Then, drone will stop at final point to hover itself.
    """
    def __init__(self):
        # create publisher for publishing ref trajectory
        self.traj_pub = rospy.Publisher('riseq/uav_polynomial_trajectory', riseq_uav_trajectory, queue_size=10)

        # 4 output
        # 5th polynomial order
        self.n = 4
        self.order = 7

        # Init_pose
        environment = rospy.get_param("riseq/environment")
        if (environment == "simulator"):
            self.init_pose = rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")
        elif (environment == "embedded_computer"):
            self.init_pose = rospy.get_param("riseq/init_pose")
        else:
            print("riseq/init_pose not available, defaulting to [0,0,0,0,0,0,1]")
            self.init_pose = [0,0,0,0,0,0,1]

        # Initialize heading
        init_quat = [self.init_pose[3], self.init_pose[4], self.init_pose[5], self.init_pose[6]]
        yaw, pitch, roll = tf.transformations.euler_from_quaternion(init_quat, axes="rzyx")

        self.start_point = np.array([self.init_pose[0], self.init_pose[1], self.init_pose[2], yaw])

        self.position = np.array([[0,0,0,0],[1,1,1,1],[2,2,2,2]])
        self.velocity = 0
        self.acceleration = 0

        # Time to take to move.
        self.time = [1, 1]
        self.waypoint = len(self.time)

        # Get Polynomial which goes to final point.
        self.solution = gs.get_solution(self.time, self.position, self.velocity, self.acceleration)

        self.start_time = rospy.get_time()
        #self.start_time = rospy.get_rostime().secs

        # start in trajectory from initial position to final gate
        # start with Index = 0
        # It is index for where drone is now.
        self.index = 0

    def compute_reference_traj(self):
        # Compute trajectory at time = now
        time = rospy.get_time()

        # stay hover at the last waypoint position
        if self.index == self.waypoint - 1:
            pos = self.position[-1]
            x = pos[0]
            y = pos[1]
            z = pos[2]
            psi = pos[3]

            pos = np.array([x, y, z])
            vel = np.array([0, 0, 0])
            acc = np.array([0, 0, 0])
            jerk = np.array([0, 0, 0])
            snap = np.array([0, 0, 0])
            yaw = psi
            yaw_dot = 0
            yaw_ddot = 0
            flat_output = [pos, vel, acc, jerk, snap, yaw, yaw_dot, yaw_ddot]
            ref_trajectory = df.compute_ref(flat_output)

        # In this polynomial, time variable has range [0, 1] at every segment
        # In other word, reference time should be time which is subtracted by last time
        else:
            ref_time = time - self.last_time
            solution = self.solution[self.n * (self.order + 1) * self.index: self.n * (self.order + 1) * (self.index + 1)]
            ref_trajectory = df.get_trajectory(solution, self.order, self.time[self.index], ref_time)

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

    environment = rospy.get_param("riseq/environment", -1)
    if environment == -1:
        rospy.logerr("environment is not specified!")
        rospy.signal_shutdown("[challenge_name] could not be read")

    if environment == "simulator":
        # select mode (easy, medium, hard, scorer)
        mode = rospy.get_param("/uav/challenge_name", -1)
        if mode == -1:
            rospy.logerr("mode is not specified!")
            rospy.signal_shutdown("[challenge_name] could not be read")

        if mode == "Challenge easy":
            time = [5]
        elif mode == "Challenge Medium":
            time = [5, 5]
        elif mode == "Challenge Hard":
            time = [5, 5, 5, 5]
        else:
            time = [5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5]

    rospy.sleep(0.1)
    # IMPORTANT WAIT TIME!
    # If this is not here, the "start_time" in the trajectory generator is
    # initialized to zero (because the node has not started fully) and the
    # time for the trajectory will be degenerated

    traj_gen = TrajectoryGenerator()
    try:
        rospy.loginfo("UAV Trajectory Publisher Created")
        traj_gen.pub_traj()
    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass