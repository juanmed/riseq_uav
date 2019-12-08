#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL
import tf
import numpy as np

from riseq_perception import irosgate_searcher

current_state = State()
current_pose = PoseStamped()
local_waypoint = PoseStamped()


def state_cb(msg):
    global current_state
    current_state = msg


def pose_cb(msg):
    """
    callback function to receive current drone's pose.
    """
    global current_pose
    current_pose = msg


def local_waypoint_cb(msg):
    """
    callback function to receive gate x y z position
    :param msg: gate position
    """
    global local_waypoint
    local_waypoint = msg


if __name__ == "__main__":
    global current_state
    global current_pose
    global local_waypoint
    rospy.init_node('iros_node', anonymous=True)

    # Create Subscriber
    rospy.Subscriber("mavros/state", State, state_cb)
    rospy.Subscriber("mavros/local_position/pose", PoseStamped, pose_cb)
    rospy.Subscriber("/riseq/perception/local_waypoint", PoseStamped, local_waypoint_cb)
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

    arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
    landing_client = rospy.ServiceProxy('mavros/cmd/land', CommandTOL)
    rate = rospy.Rate(20)

    while not current_state.connected:
        print "not connected"
        rate.sleep()
    print "connected"

    goal_pose = PoseStamped()
    # set position here
    goal_pose.pose.position.x = 0
    goal_pose.pose.position.y = 0
    goal_pose.pose.position.z = 1.5

    for i in range(50):
        local_pos_pub.publish(goal_pose)
        rate.sleep()

    offb_set_mode = SetMode()
    offb_set_mode.custom_mode = "OFFBOARD"
    arm_cmd = CommandBool()
    arm_cmd.value = True

    resp1 = set_mode_client(0, offb_set_mode.custom_mode)
    arm_client_1 = arming_client(arm_cmd.value)
    print "OFFBOARD and ARMING"

    for i in range(200):
        local_pos_pub.publish(goal_pose)
        rate.sleep()

    irosgate_searcher = irosgate_searcher.IROSGateSearcher()
    print "irosgate searcher initialized"

    loop = 0
    step = 0  # 0 : go to gate, 1 : go 1m, 2 : back 1m, 3 : go origin
    heading = 0
    goal_update = True
    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
        if rospy.Time.now() - local_waypoint.header.stamp > rospy.Duration(1.0):
            irosgate_searcher.search_gate()
            print irosgate_searcher.yaw
            q = tf.transformations.quaternion_from_euler(0, 0, irosgate_searcher.yaw, axes='sxyz')
            goal_pose.pose.orientation.x = q[0]
            goal_pose.pose.orientation.y = q[1]
            goal_pose.pose.orientation.z = q[2]
            goal_pose.pose.orientation.w = q[3]

        goal_pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(goal_pose)

        rospy.sleep(2.0)
        # rate.sleep()

    print("Return")
    goal_pose.pose.position.x = 0
    goal_pose.pose.position.y = 0
    goal_pose.pose.position.z = 0.1
    for i in range(100):
        local_pos_pub.publish(goal_pose)
        rate.sleep()

    print("landing")
    land_cmd = CommandTOL()
    landing_client(0.0, 0.0, 0.0, 0.0, 0.0)

    rospy.sleep(5)

    print("disarming")
    arm_cmd.value = False
    arm_client_1 = arming_client(arm_cmd.value)
