#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry




zedpose = None
drone_optipose = None
gate_optipose = None
drone_pose = None
gate_pose_mono = None

def zedpose_cb(odom):
    global zedpose

    zedpose = np.zeros(3)
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    z = odom.pose.pose.position.z
    zedpose[0] = x
    zedpose[1] = y
    zedpose[2] = z

def optitrack_cb(msg):
    global drone_optipose
    drone_optipose = np.zeros(3)
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    drone_optipose[0] = x
    drone_optipose[1] = y
    drone_optipose[2] = z

def optitrack_gate_cb(msg):
    global gate_optipose
    gate_optipose = np.zeros(3)
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    gate_optipose[0] = x
    gate_optipose[1] = y
    gate_optipose[2] = z    

def mavros_state(msg):
    global drone_pose
    drone_pose = np.zeros(3)
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    drone_pose[0] = x
    drone_pose[1] = y
    drone_pose[2] = z

def gate_monopose(msg):
    global gate_pose_mono
    gate_pose_mono = np.zeros(3)
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    gate_pose_mono[0] = x
    gate_pose_mono[1] = y
    gate_pose_mono[2] = z   

def main():
    global zedpose
    global drone_optipose 
    global gate_optipose
    global drone_pose
    global gate_pose_mono

    gatepose_pub = rospy.Publisher("/riseq/perception/gate_pose_computed", PoseStamped, queue_size = 10)
    rospy.Subscriber("/zed/zed_node/odom", Odometry, zedpose_cb)
    rospy.Subscriber("/vrpn_client_node/Fastquad/pose", PoseStamped, optitrack_cb)
    rospy.Subscriber("/vrpn_client_node/gatedown/pose", PoseStamped, optitrack_gate_cb)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, mavros_state)
    rospy.Subscriber("/riseq/perception/computed_position", PoseStamped, gate_monopose)

    camera_drone_vector = np.array([0.17, 0.0, 0.0])

    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        if (drone_optipose is not None) and (gate_optipose is not None) :
            vector = -(drone_optipose + camera_drone_vector)  + gate_optipose
            gate_msg = PoseStamped()
            gate_msg.header.stamp = rospy.Time.now()
            gate_msg.pose.position.x = vector[0]
            gate_msg.pose.position.y = vector[1]
            gate_msg.pose.position.z = vector[2]
            gatepose_pub.publish(gate_msg)
            drone_pose = None
            gate_pose_mono = None
            print("**Drone pose: {}, Gate Pose Mono: {}".format(drone_pose, gate_pose_mono))
        else:
            print("Drone pose: {}, Gate Pose Mono: {}".format(drone_pose, gate_pose_mono))
        r.sleep()


if __name__ == '__main__':
    try:
        # init node
        rospy.init_node('iros_gate_pose_Publisher', anonymous = True)
        main()
        rospy.loginfo('IROS Gate Pose Publisher Started')
        rospy.spin()
        rospy.loginfo('IROS Gate Pose Publisher Terminated')
    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass
