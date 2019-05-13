#!/usr/bin/env python
import rospy
import numpy as np
import tf

import keyframe_generator as kg
import quadratic_programming as qp
import draw_trajectory as dt
import optimal_time as ot
import differential_flatness as df
import gate_event as ga

from riseq_trajectory.msg import riseq_uav_trajectory


import cProfile
from pycallgraph import PyCallGraph
from pycallgraph import output
from pycallgraph.output import GraphvizOutput
import graphviz


class TrajectoryGenerator:
    def __init__(self):
        ### ROS Publisher
        # create publisher for publishing ref trajectory
        self.traj_publisher = rospy.Publisher('riseq_uav_trajectory', riseq_uav_trajectory, queue_size=10)

        ### MODE
        # select mode (easy, medium, hard, scorer)
        mode = rospy.get_param("/uav/challenge_name", -1)

        ### Init Parameters
        # Our flat output is 4
        # For smooth curve of trajectory, change and adjust order of polynomial.
        # It is recommended to adjust 6 order polynomial at least.
        self.order = 7
        self.n = 4

        ### Keyframe Generation
        # For keyframe generation, It needs drone initial position and way point.
        # Initial orientation can be quaternion, so should be transformed to euler angle.
        # Checking way point count: n. Polynomial segment: m
        # m = waypoint - 1
        init_pose = rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")
        self.gate_name = rospy.get_param("/uav/gate_names")
        self.gate_count = len(self.gate_name)
        self.gate_location = np.zeros((self.gate_count, 4, 3))
        for i, g in enumerate(self.gate_name):
            self.gate_location[i] = np.asarray(rospy.get_param("/uav/%s/location" %g))
        self.keyframe = kg.keyframe_generation(init_pose, self.gate_location, self.gate_count)
        self.waypoint = self.gate_count + 1

        ### Gate Event
        # Should count gate number because need to know next gate
        # Inflation means the scale of virtual cube including way point
        # For example, it will make cube space which is larger as much as gate times inflation.
        self.inflation = 2
        # Tolerance is like threshold to decide whether drone pass or not
        # If drone is close to gate within tolerance, it is determined as drone pass gate.
        self.tolerance = 1
        # Make array of Class for counting gate which is traversed
        self.gate_events = []
        for i in range(self.gate_count):
            self.gate_events.append(ga.GateEvent(self.gate_location[i], self.inflation))
        # count pass
        self.gate_pass = 0

        ### Time Segment and Scaling
        # In this case, the polynomial of trajectory has variable t whose range is [0, 1]
        # For using, time scaling method is applied
        if mode == "Challenge easy":
            self.time_scaling = [5]
            self.time_scaling = np.array(self.time_scaling)
            #rospy.loginfo("Level Easy!")
        elif mode == "Challenge Medium":
            self.time_scaling = [5, 5]
            self.time_scaling = np.array(self.time_scaling)
            #rospy.loginfo("Level Medium!")
        elif mode == "Challenge Hard":
            self.time_scaling = [5, 2, 2, 5]
            self.time_scaling = np.array(self.time_scaling)
            #rospy.loginfo("Level Hard!")
        else:
            self.time_scaling = [5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5]
            self.time_scaling = np.array(self.time_scaling)
            #rospy.loginfo("Challenge Final!")

        ### Current State(pos, vel, acc, jerk, snap)
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

        ### Solution for piecewise polynomial and draw trajectory in plot
        self.sol_x, self.val = qp.qp_solution(self.order, self.waypoint, self.keyframe, self.current_state, self.time_scaling)
        dt.draw_in_plot(self.sol_x, self.order, self.waypoint, self.keyframe)

        ### ROS Time
        # initialize time
        self.start_time = rospy.get_time()
        self.last_time = self.start_time

        ### Index
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

    # checking whether drone pass gate or not
    # It needs current position of drone.
    def check_gate(self):
        if self.gate_events[self.gate_pass].isEvent(self.current_state[0], self.tolerance):
            self.gate_pass = self.gate_pass + 1
        # Check every gate not only next gate
        # for the case when drone skip gate and fly through another gate
        for i, gate in enumerate(self.gate_events[self.gate_pass:]):
            if gate.isEvent(self.current_state[0], self.tolerance):
                # i == 0 means that drone goes well
                if i > 0:
                    rospy.loginfo("Skipped %d events", i)
                    # Record gate which drone skip
                    for j in range(i):
                        rospy.loginfo("%s False", self.gate_name[self.gate_pass + j])
                rospy.loginfo("Reached %s at", self.gate_name[self.gate_pass + i])
                self.gate_pass += (i + 1)

            if self.gate_pass >= self.gate_count:
                rospy.loginfo("Completed the challenge")
                rospy.signal_shutdown("Challenge complete")

    def pub_traj(self):
        # publish at Hz
        rate = rospy.Rate(200.0)

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
                self.traj_publisher.publish(traj)
                rospy.loginfo(traj)
                rate.sleep()

            except Exception:
                rospy.loginfo('People...we have a problem: {}'.format(Exception))
                continue

    # TODO
    ### Function which search minimum time
    # Not Finish
    def optimal_time(self):
        sol_x = ot.random_search(self.order, self.waypoint, self.keyframe, self.current_state, self.time_scaling)
        #dt.draw_in_plot(sol_x, self.order, self.waypoint, self.keyframe)

    # TODO
    # It would be long time to study and apply GDM
    # so for now, postpone this work later
    ''' 
    # Constrained Gradient Descent method
    def CGDmethod(self, total_time, iteration, precision):
        # h is some small number
        h = 0.0000001
        # m is segment
        m = self.waypoint -1

        # First, time is set as equal at each segment.
        time = np.ones(m) * total_time / m
        for i in range(0, iteration):
            print "hi"
            prev_time = time
            sol_x, f_before = qp.qp_solution(self.order, self.waypoint, self.keyframe, self.current_state, prev_time)
            prev_val = f_before
            for j in range(0, m):
                g = np.ones(m) * (-1/(m-1))
                g[j] = 1
                print h*g
                print prev_time
                sol_x, f_after = qp.qp_solution(self.order, self.waypoint, self.keyframe, self.current_state, prev_time + h*g)
                print prev_time
                gradient = (f_after - f_before)/h
                print gradient
                time[j] = time[j] - gradient
                print prev_time
            sol_x, val = qp.qp_solution(self.order, self.waypoint, self.keyframe, self.current_state, time)
            if abs(val - prev_val) < precision:
                return time, sol_x
    '''


if __name__ == "__main__":
    ### Init Node and Class
    rospy.init_node('riseq_ref_trajectory_publisher', anonymous=True)
    traj_gen = TrajectoryGenerator()
    # traj_gen.optimal_time()

    ### CProfile method
    # cProfile.run('traj_gen.optimal_time()')

    ### GraphvizOutput (Recommended) : This code shows profile Graphically
    # graphviz = output.GraphvizOutput(output_file='profile.png')
    # with PyCallGraph(output=graphviz):
    #   traj_gen.optimal_time()

    rospy.sleep(0.1)
    # IMPORTANT WAIT TIME!
    # If this is not here, the "start_time" in the trajectory generator is
    # initialized to zero (because the node has not started fully) and the
    # time for the trajectory will be degenerated

    try:
        rospy.loginfo("UAV Trajectory Publisher Created")
        traj_gen.pub_traj()
    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass