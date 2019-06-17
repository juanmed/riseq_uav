#!/usr/bin/env python
import rospy
import numpy as np
import tf

import quadratic_programming as qp
import draw_trajectory as dt
import gate_event as ga

import riseq_common.differential_flatness as df
import riseq_perception.keyframe_generator as kg

from riseq_trajectory.msg import riseq_uav_trajectory
from riseq_common.msg import riseq_uav_state
from nav_msgs.msg import Path

class TrajectoryGenerator:
    def __init__(self):
        self.keyframe = []
        self.waypoint = 0
        self.gate_count = 0
        # create subscriber for subscribing way point
        self.point_sub = rospy.Subscriber('riseq/uav_waypoint', Path, self.waypoint_callback)

        # waiting for callback function complete
        rospy.sleep(1)

        # create publisher for publishing ref trajectory
        self.traj_pub = rospy.Publisher('riseq/uav_trajectory', riseq_uav_trajectory, queue_size=10)


        # Our flat output is 4
        # For smooth curve of trajectory, change and adjust order of polynomial.
        # It is recommended to adjust 6 order polynomial at least.
        self.order = 7
        self.n = 4

        # determine environment
        #self.environment = rospy.get_param("riseq/environment")

        #if self.environment == "simulator":
        #    # select mode (easy, medium, hard, scorer)
        #    mode = rospy.get_param("/uav/challenge_name", -1)
        #    if mode == -1:
        #        rospy.logerr("mode is not specified!")
        #        rospy.signal_shutdown("[challenge_name] could not be read")

            # For keyframe generation, It needs drone initial position and way point.
            # Initial orientation can be quaternion, so should be transformed to euler angle.
            # Checking way point count: n. Polynomial segment: m = gate_count
            #init_pose = rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")
            #self.gate_name = rospy.get_param("/uav/gate_names")
            #self.gate_count = len(self.gate_name)
            #self.gate_location = np.zeros((self.gate_count, 4, 3))
            #for i, g in enumerate(self.gate_name):
            #    self.gate_location[i] = np.asarray(rospy.get_param("/uav/%s/location" % g))
            #self.keyframe = kg.gate_keyframe(init_pose, self.gate_location, self.gate_count)
            #self.waypoint = self.gate_count + 1
        '''
            # TODO : change to ROS topic to get current state just when request for updating trajectory.
            # create publisher for publishing ref trajectory
            # self.state_sub = rospy.subscriber('riseq/estimator/uav_estimated_state', riseq_uav_state, state_update)
            # Current state would be used in many area
            # like checking gate, updating trajectory,
            # Maybe we can get velocity and acceleration from estimation
            current_pos = self.keyframe[0]  # x y z psi
            current_vel = np.array([0, 0, 0, 0])
            current_acc = np.array([0, 0, 0, 0])
            current_jerk = np.array([0, 0, 0, 0])
            current_snap = np.array([0, 0, 0, 0])
            self.current_state = np.vstack(
                (current_pos, current_vel, current_acc, current_jerk, current_snap))

            # Should count gate number because need to know next gate
            # Inflation means the scale of virtual cube including way point
            # For example, it will make cube space which is larger as much as gate times inflation.
            self.inflation = 2
            # Tolerance is like threshold to decide whether drone pass or not
            # If drone is close to gate within tolerance, it is determined as drone pass gate.
            self.tolerance = 1
            # Make array Class for counting gate which is traversed
            self.gate_events = []
            #for i in range(self.gate_count):
            #    self.gate_events.append(ga.GateEvent(self.gate_location[i], self.inflation, self.tolerance))
            # count pass
            self.gate_pass = 0

            # Time Segment and Scaling
            # In this case, the polynomial of trajectory has variable t whose range is [0, 1]
            # For using, time scaling method is applied
            if mode == "Challenge easy":
                self.time_scaling = [5]
                self.time_scaling = np.array(self.time_scaling)
            elif mode == "Challenge Medium":
                self.time_scaling = [5, 5]
                self.time_scaling = np.array(self.time_scaling)
            elif mode == "Challenge Hard":
                self.time_scaling = [5, 2, 2, 5]
                self.time_scaling = np.array(self.time_scaling)
            else:
                self.time_scaling = np.ones(self.gate_count) * 5
                # self.time_scaling = [5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5]
                # self.time_scaling = np.array(self.time_scaling)

        elif self.environment == "embedded_computer":
            pass
        else:
            pass
        '''
        #if self.environment == "simulator":
            # Solution for piecewise polynomial and draw trajectory in plot
            # drawing plot is needed only at simulator
        current_pos = self.keyframe[0]  # x y z psi
        current_vel = np.array([0, 0, 0, 0])
        current_acc = np.array([0, 0, 0, 0])
        current_jerk = np.array([0, 0, 0, 0])
        current_snap = np.array([0, 0, 0, 0])
        self.current_state = np.vstack(
            (current_pos, current_vel, current_acc, current_jerk, current_snap))
        self.time_scaling = np.ones(self.gate_count) * 5
        self.sol_x, self.val = qp.qp_solution(self.order, self.waypoint, self.keyframe, self.current_state, self.time_scaling)
        dt.draw_in_plot(self.sol_x, self.order, self.waypoint, self.keyframe)
        #elif self.environment == "embedded_computer":
        #    self.sol_x, self.val = qp.qp_solution(self.order, self.waypoint, self.keyframe, self.current_state, self.time_scaling)
        #else:
        #    pass

        # initialize time
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
        if self.index == self.waypoint - 1:
            pos = self.keyframe[-1]
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
            sol_x = self.sol_x[self.n * (self.order + 1) * self.index: self.n * (self.order + 1) * (self.index + 1)]
            ref_trajectory = df.get_trajectory(sol_x, self.order, self.time_scaling[self.index], ref_time)
            # If the time is gone as scaling factor, segment should be changed
            # Index increases.
            if (time - self.last_time) > self.time_scaling[self.index]:
                self.index = self.index + 1  # trajectory to next gate
                self.last_time = time
        return ref_trajectory

    def pub_traj(self):
        hz = rospy.get_param('trajectory_update_rate', 200)

        # publish at Hz
        rate = rospy.Rate(hz)

        while not rospy.is_shutdown():
            try:
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

            except Exception:
                rospy.loginfo('People...we have a problem: {}'.format(Exception))
                continue

    def waypoint_callback(self, msg):
        self.waypoint = len(msg.poses)
        self.gate_count = len(msg.poses) - 1
        self.keyframe = np.zeros((self.waypoint, 4))
        for i in range(0, self.waypoint):
            self.keyframe[i][0] = msg.poses[i].pose.position.x
            self.keyframe[i][1] = msg.poses[i].pose.position.y
            self.keyframe[i][2] = msg.poses[i].pose.position.z
            phi, theta, psi = tf.transformations.euler_from_quaternion(
                [msg.poses[i].pose.orientation.x, msg.poses[i].pose.orientation.y, msg.poses[i].pose.orientation.z, msg.poses[i].pose.orientation.w], axes='sxyz')
            self.keyframe[i][3] = psi


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

    traj_gen = TrajectoryGenerator()
    try:
        rospy.loginfo("UAV Trajectory Publisher Created")
        traj_gen.pub_traj()
    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass