
##TODO

|Section|Topic|Detail|
|---|---|---|
|Naming Convention| Variables, Functions and Classes| Add convention, maybe following a pythonic coding standard|
||||
||||
||||
||||
||||

# RISE-Q Quadrotor Control Packages Coding Standard

For the development of the accompanying code for autonomous navigation of multirotors, we have tried to follow as close as possible the following coding standard and styles. For most of these rules, and since we are working under the Robot Operating System, we have tried to follow the appropriate conventions, and were is not practical or (in our opinion) is better to depart from it, we have done so with the aim of keeping ease of understanding as the highest priority.

This is a living document. As such, our rules will change, and so might the coding style for different parts of the code. We will try, however, to keep most of the code updated, including its comments.

## General

### ROS

If an explicit convention is not mentioned here, please look for an already defined and widely used convention for ROS defined here: https://github.com/leggedrobotics/ros_best_practices/wiki


## Naming Conventions

### ROS

#### ROS Nodes

If the purpose of the node is to publish data:
"riseq_uav_<node_name>_publisher"    

if the node only subscribes to data:
"riseq_uav_<node_name>_subscriber>"

For example:

"riseq_uav_input_publisher"
"riseq_uav_true_state_publisher"
"riseq_uav_camera_subscriber"

#### ROS Topics

Every topic name will start with "riseq/<package_name>/uav_<topic_name>"

For example:
"riseq/control/uav_inputs"
"riseq/trajectory/uav_reference_trajectory"

#### ROS Node Files

If an script implements a ROS Node, then the name of the file should be the same as the name of the NODE. For example:

If the name of the node is "riseq_uav_input_publisher", then the name of the file should be "riseq_uav_input_publisher.py"

#### ROS Package Names

All ROS packages should be named using "riseq_<package_name>". For example:

riseq_control
riseq_estimation
riseq_firmware

### Variable, Function and Class Names

## Folder Structure Convention

### ROS

In the case of ROS packages, we should follow the common files and directories convention as stated here: http://wiki.ros.org/Packages#Common_Files_and_Directories






