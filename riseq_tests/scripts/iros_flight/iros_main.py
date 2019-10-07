#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL
import tf
import numpy as np


current_state = State()
current_pose = PoseStamped()
gate_pose = PoseStamped()


def state_cb(msg):
    global current_state
    current_state = msg


def pose_cb(msg):
    """
    callback function to receive current drone's pose.
    """
    global current_pose
    current_pose = msg


def gate_cb(msg):
    """
    callback function to receive gate x y z position
    :param msg: gate position
    """
    global gate_pose
    try:
        gate_pose = msg.poses[0]
        rospy.loginfo(gate_pose)
    except IndexError:
        pass

if __name__ == "__main__":
    rospy.init_node('iros_node', anonymous=True)

    # Create Subscriber
    rospy.Subscriber("mavros/state", State, state_cb)
    rospy.Subscriber("mavros/local_position/pose", PoseStamped, pose_cb)
    rospy.Subscriber("/riseq/perception/uav_mono_waypoint", Path, gate_cb)

    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

    print("Publisher and Subscriber Created")
    arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
    landing_client = rospy.ServiceProxy('mavros/cmd/land', CommandTOL)

    print("Clients Created")
    rate = rospy.Rate(20)

    while (not current_state.connected):
        rate.sleep()

    print("Creating pose")
    goal_pose = PoseStamped()
    # set position here
    goal_pose.pose.position.x = 0
    goal_pose.pose.position.y = 0
    goal_pose.pose.position.z = 1.5

    for i in range(100):
        local_pos_pub.publish(goal_pose)
        rate.sleep()

    print("Creating Objects for services")
    offb_set_mode = SetMode()
    offb_set_mode.custom_mode = "OFFBOARD"
    arm_cmd = CommandBool()
    arm_cmd.value = True

    print("OFFBOARD and ARMING")
    resp1 = set_mode_client(0, offb_set_mode.custom_mode)
    arm_client_1 = arming_client(arm_cmd.value)

    while current_state.mode != "OFFBOARD" or not current_state.armed:
        rospy.sleep(5.0)

    for i in range(100):
        local_pos_pub.publish(goal_pose)
        rate.sleep()

    wp = [[(+1, 0), (0, +3), (-6, 0)], [(-2, 0), (0, -4), (6, 0)]]
    heading = [[0, +np.pi / 2, +np.pi], [np.pi, np.pi /2 *3, 0]]
    gate_detecting = True
    gate_count = 0
    wp_len = len(wp[0])
    wp_index = 0
    update_goal = True
    while not rospy.is_shutdown():
        if update_goal is True:
            if gate_detecting is True:
                if gate_count == 0:
                    goal_pose.pose.position.x = current_pose.pose.position.x + gate_pose.pose.position.x
                    goal_pose.pose.position.y = current_pose.pose.position.y + gate_pose.pose.position.y
                else:
                    goal_pose.pose.position.x = current_pose.pose.position.x - gate_pose.pose.position.x
                    goal_pose.pose.position.y = current_pose.pose.position.y - gate_pose.pose.position.y
                goal_pose.pose.position.z = current_pose.pose.position.z + gate_pose.pose.position.z

                gate_detecting = False
                update_goal = False

            else:
                q = tf.transformations.quaternion_from_euler(0, 0, heading[gate_count][wp_index], axes='sxyz')

                goal_pose.pose.position.x = current_pose.pose.position.x + wp[gate_count][wp_index][0]
                goal_pose.pose.position.y = current_pose.pose.position.y + wp[gate_count][wp_index][1]
                goal_pose.pose.orientation.x = q[0]
                goal_pose.pose.orientation.y = q[1]
                goal_pose.pose.orientation.z = q[2]
                goal_pose.pose.orientation.w = q[3]

                wp_index = wp_index + 1
                if wp_index == wp_len:
                    gate_detecting = True
                    gate_count = gate_count + 1
                    gate_count = gate_count % 2
                    wp_index = 0

                update_goal = False

        if np.linalg.norm((current_pose.pose.position.x - goal_pose.pose.position.x, current_pose.pose.position.y - goal_pose.pose.position.y)) < 0.2:
            update_goal = True
        local_pos_pub.publish(goal_pose)
        rate.sleep()

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