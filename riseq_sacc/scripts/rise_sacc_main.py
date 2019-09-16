#!/usr/bin/env python
"""
author:  Eugene Auh
version: 0.1.0
brief: 

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy of this
software and associated documentation files (the ""Software""), to deal in the 
Software without restriction, including without limitation the rights to use, copy, 
modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, 
and to permit persons to whom the Software is furnished to do so, subject to the 
following conditions:
The above copyright notice and this permission notice shall be included in all copies 
or substantial portions of the Software.
THE SOFTWARE IS PROVIDED *AS IS*, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF 
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE 
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

import rospy
import numpy as np
import geopy.distance
from std_msgs.msg import Int32
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, GlobalPositionTarget, HomePosition
from mavros_msgs.srv import CommandHome, CommandBool, SetMode, CommandTOL

current_state = State()
current_position = NavSatFix()
home = HomePosition()
wp0 = GlobalPositionTarget()
wp1 = GlobalPositionTarget()
wp2 = GlobalPositionTarget()
wp3 = GlobalPositionTarget()
wp4 = GlobalPositionTarget()

wp0.coordinate_frame = GlobalPositionTarget().FRAME_GLOBAL_INT
wp0.type_mask = GlobalPositionTarget().IGNORE_VX + GlobalPositionTarget().IGNORE_VY + GlobalPositionTarget().IGNORE_VZ + GlobalPositionTarget().IGNORE_AFX + GlobalPositionTarget().IGNORE_AFY + GlobalPositionTarget().IGNORE_AFZ + GlobalPositionTarget().FORCE + GlobalPositionTarget().IGNORE_YAW_RATE
wp0.latitude  = 37.565011
wp0.longitude = 126.628919
wp0.altitude  = 5.0
wp0.yaw       = np.pi/180.0 * 117.0

wp1.coordinate_frame = wp1.coordinate_frame
wp1.type_mask = wp1.type_mask
wp1.latitude  = 37.566025
wp1.longitude = 126.628206
wp1.altitude  = 100.0
wp1.yaw       = np.pi/180.0 * 117.0

wp2.coordinate_frame = wp1.coordinate_frame
wp2.type_mask = wp1.type_mask
wp2.latitude  = 37.565350
wp2.longitude = 126.626778
wp2.altitude  = 100.0
wp2.yaw       = np.pi/180.0 * 210.0

wp3.coordinate_frame = wp1.coordinate_frame
wp3.type_mask = wp1.type_mask
wp3.latitude  = 37.564700
wp3.longitude = 126.627628
wp3.altitude  = 2.0
wp3.yaw       = np.pi/180.0 * 313.0

wp4.coordinate_frame = wp1.coordinate_frame
wp4.type_mask = wp1.type_mask
wp4.latitude  = 37.565111
wp4.longitude = 126.628503
wp4.altitude  = 2.0
wp4.yaw       = np.pi/180.0 * 28.0

wp = wp0
wp_target = wp2
wp_target.yaw = wp3.yaw
wp_avoidance = wp3
wp_avoidance.yaw = wp4.yaw
wp_helical = wp4

waypoint = Int32()
process = Int32()


def homeCb(msg):
    home.geo.altitude = msg.geo.altitude

def stateCb(msg):
    current_state.mode = msg.mode

def positionCb(msg):
    current_position.latitude = msg.latitude
    current_position.longitude = msg.longitude
    current_position.altitude = msg.altitude


def targetCb(msg):
    wp_target.latitude = msg.latitude
    wp_target.longitude = msg.longitude
    wp_target.altitude = msg.altitude
    wp_target.yaw = msg.yaw

def avoidanceCb(msg):
    wp_avoidance.latitude = msg.latitude
    wp_avoidance.longitude = msg.longitude
    wp_avoidance.altitude = msg.altitude
    wp_avoidance.yaw = msg.yaw

def helicalCb(msg):
    wp_helical.latitude = msg.latitude
    wp_helical.longitude = msg.longitude
    wp_helical.altitude = msg.altitude
    wp_helical.yaw = msg.yaw


def getDistance():
    h = geopy.distance.vincenty((wp.latitude, wp.longitude), (current_position.latitude, current_position.longitude)).m
    v = wp.altitude - current_position.altitude
    print("Distance to next waypoint: %.1f, %.1f" %(h, v))
    return h, v


if __name__ == "__main__":
    # Initialize node
    rospy.init_node('riseq_sacc_main')

    # Set loop rate
    rate = 50
    r = rospy.Rate(rate)

    # Subscriber
    rospy.Subscriber('/mavros/state', State, stateCb)
    rospy.Subscriber('/mavros/home_position/home', HomePosition, homeCb)
    rospy.Subscriber('/mavros/global_position/global', NavSatFix, positionCb)
    
    # Publisher
    position_publisher = rospy.Publisher('/mavros/setpoint_position/global', GlobalPositionTarget, queue_size=10)
    waypoint_publisher = rospy.Publisher('/waypoint', Int32, queue_size=10)
    process_publisher = rospy.Publisher('/process', Int32, queue_size=10)

    # Service client
    set_home_client = rospy.ServiceProxy('/mavros/cmd/set_home', CommandHome)
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    landing_client = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

    rospy.sleep(1.0)


    # Set home position and update waypoints' altitude on ground
    while True:
        set_home = set_home_client(True, 0.0, 0.0, 0.0)
        r.sleep()
        if set_home.success is True:
            print(">>>>>Set home position<<<<<\n")
            break
    rospy.sleep(3.0)
    wp0.altitude = wp0.altitude + home.geo.altitude
    wp1.altitude = wp1.altitude + home.geo.altitude
    wp2.altitude = wp2.altitude + home.geo.altitude
    wp3.altitude = wp3.altitude + home.geo.altitude
    wp4.altitude = wp4.altitude + home.geo.altitude
    print(">>>>>Updated altitude<<<<\n")
    print(">>>>>Arm<<<<<\n")


    # Main loop publishing setpoint
    step = 0
    while not rospy.is_shutdown():
        # Takeoff
        if step == 0:
            wp = wp0
            err_h, err_v = getDistance()
            if (err_h <= 2.0) and (abs(err_v) <= 2.0):
                step += 1
            waypoint.data = 1

        # wp0 -> wp1
        elif step == 1:
            wp = wp1
            err_h, err_v = getDistance()
            if (err_h <= 2.0) and (abs(err_v) <= 2.0):
                step += 1
            waypoint.data = 1

        # wp1 -> wp2
        elif step == 2:
            wp = wp2
            err_h, err_v = getDistance()
            if (err_h <= 2.0) and (abs(err_v) <= 2.0):
                step += 1
            waypoint.data = 2

        # wp2 -> wp3, Track target
        elif step == 3:
            process_publisher.publish(process)
            wp = wp_target
            if (geopy.distance.vincenty((wp3.latitude, wp3.longitude), (current_position.latitude, current_position.longitude)).m <= 2.0) and (abs(wp3.altitude - current_position.altitude) <= 2.0):
                step += 1
                process.data = 2
            waypoint.data = 3

        # Fix orientation
        elif step == 4:
            wp = wp_avoidance
            for i in range(0, 150):
                wp.header.stamp = rospy.Time.now()
                position_publisher.publish(wp)
                r.sleep()
            step += 1
        # wp3 -> wp4, Obstacle avoidance
        elif step == 5:
            process_publisher.publish(process)
            wp = wp_avoidance
            if (geopy.distance.vincenty((wp4.latitude, wp4.longitude), (current_position.latitude, current_position.longitude)).m <= 2.0) and (abs(wp4.altitude - current_position.altitude) <= 2.0):
                step += 1
                process.data = 3
            waypoint.data = 4

        # Helical trajectory, Number segmentation
        elif step == 6:
            process_publisher.publish(process)
            wp = wp_helical
            if (current_position.altitude >= (home.geo.altitude + 32.5)):
                step += 1
            waypoint.data = 0

        # Return home
        elif step == 7:
            wp = wp0
            wp.altitude = home.geo.altitude + 35.0
            err_h, err_v = getDistance()
            if (err_h <= 2.0) and (abs(err_v) <= 2.0):
                step += 1
            waypoint.data = 0

        # Land
        elif step == 8:
            wp = wp0
            wp.altitude = home.geo.altitude + 2.0
            if (err_h <= 1.0) and (abs(err_v) <= 1.0) and (current_state.mode != "AUTO.LAND"):
                print(">>>>>Landing<<<<<\n")
                land_home = landing_client(0.0, 0.0, 0.0, 0.0, 0.0)
            waypoint.data = 0


        # Publish position and waypoint
        wp.header.stamp = rospy.Time.now()
        position_publisher.publish(wp0)
        waypoint_publisher.publish(waypoint)
        
        r.sleep()
