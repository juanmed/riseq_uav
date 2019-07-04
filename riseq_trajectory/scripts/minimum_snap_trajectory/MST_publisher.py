#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from riseq_trajectory.trajectory_generator import TrajectoryGenerator
import riseq_trajectory.draw_trajectory as dt
import numpy as np
import tf
import quadratic_programming as qp


class MinimumSnapTrajectory(TrajectoryGenerator):
    """
    Publish minimum snap trajectory.
    """
    def __init__(self):
        # Our flat output is 4
        # For smooth curve of trajectory, change and adjust order of polynomial.
        # It is recommended to adjust 9 order polynomial at least to avoid deficient rank.
        order = 9

        # determine environment
        environment = rospy.get_param("riseq/environment")
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

        super(MinimumSnapTrajectory, self).__init__(order, time_scaling)

    def set_time(self):
        time_scaling = np.ones(len(self.waypoint)-1) * 5
        self.time = time_scaling

    def calculate_solution(self):
        self.state = np.zeros((5, 4))
        solution, val = qp.qp_solution(self.order, self.waypoint, self.state, self.time)
        self.solution = solution


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

    environment = rospy.get_param("riseq/environment")
    if environment != "simulator":
        traj_gen.set_time()
    traj_gen.calculate_solution()

    try:
        rospy.loginfo("UAV Trajectory Publisher Created")
        traj_gen.pub_traj()
    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass