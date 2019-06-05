
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


## File Naming Conventions

### ROS

###### ROS Nodes

If the purpose of the node is to publish data:
"riseq_uav_<node_name>_publisher"    

if the node only subscribes to data:
"riseq_uav_<node_name>_subscriber>"

For example:

"riseq_uav_input_publisher"
"riseq_uav_true_state_publisher"
"riseq_uav_camera_subscriber"

###### ROS Topics

Every topic name will start with "riseq/<package_name>/uav_<topic_name>"

For example:
"riseq/control/uav_inputs"
"riseq/trajectory/uav_reference_trajectory"

###### ROS Node Files

If an script implements a ROS Node, then the name of the file should be the same as the name of the NODE. For example:

If the name of the node is "riseq_uav_input_publisher", then the name of the file should be "riseq_uav_input_publisher.py"

###### ROS Package Names

All ROS packages should be named using "riseq_<package_name>". For example:

riseq_control
riseq++_estimation
riseq_firmware

###### ROS Parameters

Parameters stored in the parameter server must follow the following pattern:

'riseq/<parameter_name>'

For example:
'riseq/controller_mode'
'riseq/update_frequency'

## Variable, Function and Class Names

## Folder Structure Convention

###### ROS

In the case of ROS packages, we should follow the common files and directories convention as stated here: http://wiki.ros.org/Packages#Common_Files_and_Directories

## Documentation

For automated documentation generation we use [Doxygen](http://www.doxygen.nl/index.html). Documentation for python scripts is generated through doxygen with the help of [doxypypy](https://github.com/Feneric/doxypypy). This means all python scripts should follow the docstring format, while C++ scripts can and should use all the special commands doxygen offers.

***All scripts must have a header at the very beginning*** of the script file with the following block:

For C++:
```cpp
/**
@author  author's name
@version version of this script
@brief a description of the functionality this script implements

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

*/
```

For python:
```python
"""
author:  author's name
version: version of this script
brief: a description of the functionality this script implements

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
```

All methods in C++ or python must have the following documentation tags:

For C++
```cpp

/**
    @brief All functions must contain a brief but clear description. It should
    be placed right after the method signature. After the description, the 
    tags @param for input parameters, @return if the functions returns
    any result, must be present.

    @param x here goes x description. It must contain type of variable 
    for x (int, double, np.array, etc)
    @param y here goes y description. It must contain type of variable 
    for y (int, double, np.array, etc)
    @return the function's return value description. For example: result of 2+2 

*/
double foo(int x, int y):
     double a;
     return a = 2.0 + 2.0;
```

For python:
```python
def foo(x,y):
    """
    All functions must contain a brief but clear description. It should
    be placed right after the method signature. After the description, the 
    tags @param for input parameters, @return if the functions returns
    any result, must be present.

    Args:
        x: here goes x description. It must contain type of variable 
        for x (int, double, np.array, etc)
        y: here goes y description. It must contain type of variable 
        for y (int, double, np.array, etc)
    Returns:
        the function's return value description. For example: result of 2+2 
    """
    return 2+2
```







