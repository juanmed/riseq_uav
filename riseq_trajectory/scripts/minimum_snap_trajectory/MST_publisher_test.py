#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from riseq_trajectory.srv import MakeTrajectory
from riseq_trajectory.msg import riseq_uav_trajectory
import riseq_trajectory.draw_trajectory as dt
import riseq_trajectory.hovering as hv
import riseq_common.differential_flatness as df
import numpy as np
import tf
import quadratic_programming as qp


class MinimumSnapTrajectory:
    """
    Publish minimum snap trajectory.
    """
    def __init__(self):
        # Our flat output is 4
        # For smooth curve of trajectory, change and adjust order of polynomial.
        # It is recommended to adjust 9 order polynomial at least to avoid deficient rank.
        self.order = 9

        self.solution = None
        self.waypoint = None
        self.state = None
        self.m = 0

        # create service server for generating trajectory
        rospy.Service('make_trajectory', MakeTrajectory, self.make_trajectory)

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

        # determine environment
        environment = rospy.get_param("riseq/environment", -1)
        if environment == "simulator":
            # select mode (easy, medium, hard, scorer)
            mode = rospy.get_param("/uav/challenge_name", -1)
            if mode == -1:
                rospy.logerr("mode is not specified!")
                rospy.signal_shutdown("[challenge_name] could not be read")

            # Time Segment and Scaling
            # In this case, the polynomial of trajectory has variable t whose range is [0, 1]
            # For using, time scaling method is applied
            if mode == "Challenge Easy":
                time_scaling = [5]
            elif mode == "Challenge Medium":
                time_scaling = [5, 5]
            elif mode == "Challenge Hard":
                time_scaling = [5, 2, 2, 5]
            else:
                time_scaling = [5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5]
        else:
            time_scaling = None

        self.time = time_scaling

    def make_trajectory(self, req):
        self.waypoint_update(req.path)
        self.index = 0
        time_scaling = np.ones(self.m) * 5
        self.time = time_scaling
        self.state = np.zeros((5, 4))
        solution, val = qp.qp_solution(self.order, self.waypoint, self.state, self.time)
        self.solution = solution
        return True

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

    def compute_reference_traj(self):
        if self.solution is not None:
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
        else:
            ref_trajectory = hv.hovering_traj([0, 0, 0, 0])
            return ref_trajectory

    def pub_traj(self):
        hz = rospy.get_param('riseq/trajectory_update_rate', 200)

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

    rospy.sleep(0.1)
    # IMPORTANT WAIT TIME!
    # If this is not here, the "start_time" in the trajectory generator is
    # initialized to zero (because the node has not started fully) and the
    # time for the trajectory will be degenerated
    traj_gen = MinimumSnapTrajectory()

    try:
        rospy.loginfo("UAV Trajectory Publisher Created")
        traj_gen.pub_traj()
    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass