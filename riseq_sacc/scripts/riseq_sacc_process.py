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

import roslaunch
import rospy
import numpy as np
from std_msgs.msg import Int32


# Nodes to run
package = 'riseq_sacc'
exe_target = 'riseq_sacc_target_detection'
exe_avoidance = 'riseq_sacc_obstacle_avoidance'
exe_helical1 = 'riseq_sacc_number'
exe_helical2 = 'riseq_sacc_ladder_info'
exe_helical3 = 'riseq_helix_trajectory'

# roslaunch node api
node_target = roslaunch.core.Node(package, exe_target)
node_avoidance = roslaunch.core.Node(package, exe_avoidance)
node_helical1 = roslaunch.core.Node(package, exe_helical1)
node_helical2 = roslaunch.core.Node(package, exe_helical2)
node_helical3 = roslaunch.core.Node(package, exe_helical3)
launch_node = roslaunch.scriptapi.ROSLaunch()

# roslaunch launch api
uuid_darknet = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid_darknet)
launch_darknet = roslaunch.parent.ROSLaunchParent(uuid_darknet, ["/home/nvidia/catkin_ws/src/darknet_ros/darknet_ros/launch/darknet.launch"])

# Checker for running node once
check1 = True
check2 = True
check3 = True


def processCb(msg):
    global check1
    global check2
    global check3

    if msg.data == 1:
        if check1 is True:
            process_target = launch_node.launch(node_target)
            check1 = False

    elif msg.data == 2:
        if process_target.is_alive():
            process_target.stop()
        if check2 is True:
            process_avoidance = launch_node.launch(node_avoidance)
            check2 = False

    elif msg.data == 3:
        if process_avoidance.is_alive():
            process_avoidance.stop()
        if check3 is True:
            process_helical1 = launch_node.launch(node_helical1)
            process_helical2 = launch_node.launch(node_helical2)
            process_helical3 = launch_node.launch(node_helical3)
            launch_darknet.start()
            check3 = False

    elif msg.data == 4:
        if process_helical1.is_alive():
            process_helical1.stop()
        if process_helical2.is_alive():
            process_helical2.stop()
        if process_helical3.is_alive():
            process_helical3.stop()
        launch_darknet.shutdown()


if __name__ == "__main__":
    # Initialize node
    rospy.init_node('riseq_sacc_process')
    launch_node.start()

    # Set loop rate
    rate = 50
    r = rospy.Rate(rate)

    # Subscriber
    rospy.Subscriber('/process', Int32, processCb)

    # Main loop publishing setpoint
    while not rospy.is_shutdown():        
        r.sleep()
