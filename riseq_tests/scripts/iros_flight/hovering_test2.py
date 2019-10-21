#!/usr/bin/env python

import rospy
from geometry_msgs.msg import *
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL


current_state = State()


def state_cb(msg):
    global current_state
    current_state = msg
    print "callback"


def computed_gate_cb(msg):
    """
    callback function to receive gate x y z position
    :param msg: gate position
    """
    global computed_gate_pose
    computed_gate_pose = msg
    print "computed: %f" %msg.pose.position.x


def solvepnp_gate_cb(msg):
    """
    callback function to receive gate x y z position
    :param msg: gate position
    """
    global solvepnp_gate_pose
    solvepnp_gate_pose = msg
    print "solvepnp: %f" %msg.pose.position.x


if __name__ == "__main__":
    global current_state
    rospy.init_node('offb_node', anonymous=True)
    rospy.Subscriber("mavros/state", State, state_cb)
    local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    rospy.Subscriber("/riseq/perception/computed_position", PoseStamped, computed_gate_cb)
    rospy.Subscriber("/riseq/perception/solvepnp_position", PoseStamped, solvepnp_gate_cb)

    print("Publisher and Subscriber Created")
    arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
    landing_client = rospy.ServiceProxy('mavros/cmd/land', CommandTOL)

    print("Clients Created")
    rate = rospy.Rate(20)

    while (not current_state.connected):
        print(current_state.connected)
        rate.sleep()

    print("Creating pose")
    pose = PoseStamped()
    # set position heres
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 1

    for i in range(100):
        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        #print("Sending pose: {}".format(pose))
        rate.sleep()

    print("Creating Objects for services")
    offb_set_mode = SetMode()
    offb_set_mode.custom_mode = "OFFBOARD"
    arm_cmd = CommandBool()
    arm_cmd.value = True

    resp1 = set_mode_client(0, offb_set_mode.custom_mode)
    arm_client_1 = arming_client(arm_cmd.value)

    last_request = rospy.Time.now()
    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        # print current_state
        rate.sleep()
        #print("Sending pose: {}".format(pose))
    
        if rospy.Time.now() - start_time > rospy.Duration(17.0):
            pose.pose.position.x=1.0
        if rospy.Time.now() - start_time > rospy.Duration(34.0):
            pose.pose.position.y=1.0
        if rospy.Time.now() - start_time > rospy.Duration(51.0):
            pose.pose.position.x = 0.0
            pose.pose.position.y = 0.0
        if rospy.Time.now() - start_time > rospy.Duration(60.0):
        
        #if rospy.Time.now() - start_time > rospy.Duration(10.0):
            break

    print("Return")
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0.1
    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()

    print("landing")
    land_cmd = CommandTOL()
    landing_client(0.0, 0.0, 0.0, 0.0, 0.0)

    rospy.sleep(5)

    print("disarming")
    arm_cmd.value = False
    arm_client_1 = arming_client(arm_cmd.value)
