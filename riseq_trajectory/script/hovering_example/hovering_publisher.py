#!/usr/bin/env python
import rospy
import numpy as np
import tf

import moving as mv
import hovering as hv
import riseq_common.differential_flatness as df

from riseq_trajectory.msg import riseq_uav_trajectory
from riseq_trajectory.trajectory_generator import TrajectoryGenerator


class Hovering(TrajectoryGenerator):
    """
    Publish trajectory for hovering after simple move.
    At first, drone moves simply like along z-axis or 2 coordinates.
    Then, drone will stop at final point to hover itself.
    """
    def __init__(self):
        """
         In this case, 6 constraints exist.
        Initial and final position, velocity, acceleration.
        Able to determine 6 coefficient... z(t) = at^5 + bt^4 + ct^3 + dt^2 + et + f
        """
        # 5th polynomial order`
        order = 5

        # Time to take to move.
        time = [3]

        super(Hovering, self).__init__(order, time)

    def calculate_solution(self):
        moving = [0, 0, 1, 0]

        orientation = tf.transformations.euler_from_quaternion(
            [self.init_pose[3], self.init_pose[4], self.init_pose[5], self.init_pose[6]], axes='sxyz')
        start_point = np.array([self.init_pose[0], self.init_pose[1], self.init_pose[2], orientation[2]])
        final_point = np.array([start_point[0] + moving[0], start_point[1] + moving[1],
                                start_point[2] + moving[2], start_point[3]])

        # Get Polynomial which goes to final point.
        solution = mv.go_along(self.order, self.time, start_point, final_point)

        # Override parent class property
        self.keyframe = np.vstack((start_point, final_point))
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

    hovering = Hovering()
    hovering.calculate_solution()
    try:
        rospy.loginfo("UAV Trajectory Publisher Created")
        hovering.pub_traj()
    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass