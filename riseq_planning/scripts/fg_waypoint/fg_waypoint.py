#!/usr/bin/env python
import rospy
from tf.transformations import euler_from_quaternion
import numpy as np
from riseq_planning.waypoint_publisher import WayPointPublisher


class FG_WayPoint(WayPointPublisher):
    """
    Class to publish way point only when flight goggle simulator
    """
    def __init__(self):
        """
        Function to construct way point
        """
        init_pose = rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")
        gate_name = rospy.get_param("/uav/gate_names")
        gate_count = len(gate_name)
        gate_location = np.zeros((gate_count, 4, 3))
        for i, g in enumerate(gate_name):
           gate_location[i] = np.asarray(rospy.get_param("/uav/%s/location" % g))

        # if pose is written as quaternion form, it needs to be transformed to euler form.
        # if current_pose == [ x y z x y z w ]   3 position 4 quaternion  -> 3 position and 1 yaw
        init_pose_orientation = euler_from_quaternion(
            [init_pose[3], init_pose[4], init_pose[5], init_pose[6]], axes='sxyz')
        init_waypoint = np.array([init_pose[0], init_pose[1], init_pose[2], init_pose_orientation[2]])

        waypoint = np.zeros((gate_count+1, 4))
        waypoint[0] = init_waypoint

        for i in range(0, gate_count):
            waypoint[i+1] = self.gate_point(gate_location[i])
        self.waypoint = self.compensate_direction(waypoint, gate_count)
        super(FG_WayPoint, self).__init__()

    def gate_point(self, gate):
        """
        Function to get way point of each gate
        return position and psi
        """
        # calculate center point of gate
        gate_x = np.sum(gate[:, 0]) / 4.0
        gate_y = np.sum(gate[:, 1]) / 4.0
        gate_z = np.sum(gate[:, 2]) / 4.0

        # cross product for getting gate direction
        p1 = gate[0, :]
        p2 = gate[1, :]
        p3 = gate[2, :]
        v1 = p3 - p1
        v2 = p2 - p1
        cp = np.cross(v1, v2)
        gate_psi = np.arctan2(cp[1], cp[0])

        gate_pose = np.array([gate_x, gate_y, gate_z, gate_psi])
        return gate_pose

    def compensate_direction(self, waypoint, gate_count):
        """
        Function to compensate direction of gate point.
        Sometimes there are reverse direction.
        """
        for i in range(gate_count):
            before_waypoint = waypoint[i]
            after_waypoint = waypoint[i+1]
            x_vector = after_waypoint[0] - before_waypoint[0]
            y_vector = after_waypoint[1] - before_waypoint[1]
            z_vector = after_waypoint[2] - before_waypoint[2]
            vector = np.array([x_vector, y_vector, z_vector])
            direction = ([np.cos(after_waypoint[3]), np.sin(after_waypoint[3]), 0])
            dotproduct = np.dot(vector, direction)
            cosangle = dotproduct / (np.linalg.norm(vector) * np.linalg.norm(direction))
            if cosangle < 0:
                if after_waypoint[3] < 0:
                    after_waypoint[3] = after_waypoint[3] + np.pi
                else:
                    after_waypoint[3] = after_waypoint[3] - np.pi
            waypoint[i+1][3] = after_waypoint[3]

        return waypoint


if __name__ == "__main__":
    # Init Node and Class
    rospy.init_node('riseq_waypoint_publisher', anonymous=True)

    # Wait some time before running. This is to adapt to some simulators
    # which require some 'settling time'

    try:
        wait_time = int(rospy.get_param('riseq/planning_wait'))
    except:
        print('riseq/planning_wait_time parameter is unavailable')
        print('Setting a wait time of 0 seconds.')
        wait_time = 1

    # wait time for simulator to get ready...
    while rospy.Time.now().to_sec() < wait_time:
        if (int(rospy.Time.now().to_sec()) % 1) == 0:
            rospy.loginfo(
                "Starting Waypoint Publisher in {:.2f} seconds".format(wait_time - rospy.Time.now().to_sec()))

    rospy.sleep(0.1)
    # IMPORTANT WAIT TIME!
    # If this is not here, the "start_time" in the trajectory generator is
    # initialized to zero (because the node has not started fully) and the
    # time for the trajectory will be degenerated

    way_point = FG_WayPoint()
    try:
        rospy.loginfo("UAV Waypoint Publisher Created")
        way_point.pub_point()
    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass