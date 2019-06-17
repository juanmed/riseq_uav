#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import matplotlib.pyplot as plt


class WayPointPublisher(object):
    """
        This class is to publish way point to trajectory generator
        It is parent class for other class of specific method
    """

    def __init__(self, waypoint):
        # create publisher for publishing way point
        self.point_pub = rospy.Publisher('riseq/planning/uav_waypoint', Path, queue_size=10)

        self.waypoint = waypoint


    def pub_point(self):
        hz = 10
        rate = rospy.Rate(hz)

        while not rospy.is_shutdown():

            point = Path()
            for i in range(0, len(self.waypoint)):
                pose = PoseStamped()
                pose.header.frame_id = str(i)
                pose.pose.position.x = self.waypoint[i][0]
                pose.pose.position.y = self.waypoint[i][1]
                pose.pose.position.z = self.waypoint[i][2]
                q = quaternion_from_euler(0, 0, self.waypoint[i][3])
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
                point.poses.append(pose)

            self.point_pub.publish(point)
            rate.sleep()
