#!/usr/bin/env python
import rospy
from riseq_trajectory.msg import riseq_uav_trajectory
from riseq_common.msg import riseq_uav_state

import riseq_common.differential_flatness as df
import riseq_perception.keyframe_generator as kg
import gate_event as ga
import hovering as hv
import numpy as np
import tf


class TrajectoryGenerator(object):
    def __init__(self, order, time):
        self.state = np.zeros((2, 4))

        # create publisher for publishing ref trajectory
        self.traj_pub = rospy.Publisher('riseq/trajectory/uav_reference_trajectory', riseq_uav_trajectory, queue_size=10)

        # create subscriber for subscribing state
        self.state_sub = rospy.Subscriber('riseq/tests/uav_fg_true_state', riseq_uav_state, self.state_update)

        # Our flat output is 4
        self.n = 4

        # Override property
        self.order = order
        self.time = time

        # determine environment
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

            # For keyframe generation, It needs drone initial position and way point.
            # Checking way point count: n. Polynomial segment: m = gate_count
            self.init_pose = rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")
            self.gate_name = rospy.get_param("/uav/gate_names")
            self.gate_count = len(self.gate_name)
            self.gate_location = np.zeros((self.gate_count, 4, 3))
            for i, g in enumerate(self.gate_name):
                self.gate_location[i] = np.asarray(rospy.get_param("/uav/%s/location" % g))
            self.keyframe = kg.gate_keyframe(self.init_pose, self.gate_location, self.gate_count)
            self.waypoint = self.gate_count + 1

            # Should count gate number because need to know next gate
            # Inflation means the scale of virtual cube including way point
            # For example, it will make cube space which is larger as much as gate times inflation.
            self.inflation = 2
            # Tolerance is like threshold to decide whether drone pass or not
            # If drone is close to gate within tolerance, it is determined as drone pass gate.
            self.tolerance = 1
            # Make array Class for counting gate which is traversed
            self.gate_events = []
            for i in range(self.gate_count):
                self.gate_events.append(ga.GateEvent(self.gate_location[i], self.inflation, self.tolerance))
            # count pass
            self.gate_pass = 0

        #TODO
        elif environment == "embedded_computer":
            self.init_pose = rospy.get_param("riseq/init_pose")
            # self.keyframe = None
            # self.waypoint = len(self.keyframe)
            self.time = [5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5]
        else:
            pass

        self.solution = 0

        # initialize time
        self.start_time = rospy.get_time()
        # self.start_time = rospy.get_rostime().secs

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
            hovering_point = self.keyframe[-1]
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

    def state_update(self, state):
        x, y, z = state.pose.position.x, state.pose.position.y, state.pose.position.z
        vx, vy, vz = state.twist.linear.x, state.twist.linear.y, state.twist.linear.z

        ori_quat = [state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z,
                    state.pose.orientation.w]
        psi, theta, phi = tf.transformations.euler_from_quaternion(ori_quat, axes='rzyx')
        angular_velocity = np.array([[state.twist.angular.x],[state.twist.angular.y],[state.twist.angular.z]])

        matrix = np.array([[1, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
                           [0, np.cos(phi), -1 * np.sin(phi)],
                           [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]])
        phi_dot, theta_dot, psi_dot = np.matmul(matrix, angular_velocity)

        position = np.array([x, y, z, psi])
        velocity = np.array([vx, vy, vz, psi_dot])

        self.state[0] = position
        self.state[1] = velocity
