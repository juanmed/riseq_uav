#!/usr/bin/env python
import rospy

import moving as go
import differential_flatness as df

from riseq_trajectory.msg import riseq_uav_trajectory


class Hovering:
    def __init__(self):
        # Time to take to move.
        self.time = 5

        self.start_time = rospy.get_time()
        self.last_time = self.start_time


    def pub_traj(self):
        # publish at Hz
        rate = rospy.Rate(200.0)

        while not rospy.is_shutdown():
            try:
                time = rospy.get_time()
                ref_time = self.last_time - time



if __name__ == "__main__":
    ### Init Node and Class
    rospy.init_node('riseq_ref_trajectory_publisher', anonymous=True)
    hovering = Hovering()

    rospy.sleep(0.1)
    # IMPORTANT WAIT TIME!
    # If this is not here, the "start_time" in the trajectory generator is
    # initialized to zero (because the node has not started fully) and the
    # time for the trajectory will be degenerated

    try:
        rospy.loginfo("UAV Trajectory Publisher Created")
        hovering.pub_traj()
    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass