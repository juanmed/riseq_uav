##TODO

|Section|Topic|Detail|
|---|---|---|
|Testing| Fill| Maybe add some testing using roslaunch command and any launch file|
||||
||||
||||


# RISE-Q UAV: Autonomous navigation algorithms for multirotors
Welcome! This repository contains code for autonomous navigation algorithms developed by the Robotics and Intelligent Systems (RISE) Lab at SKKU, by the Quadrotor Team. The scripts are developed using python2.7, ROS Kinetic, over Ubuntu16.04LTS and many python libraries (control, cvxopt, etc).


## Installation

1. Install [ROS-Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) and [git](https://help.ubuntu.com/lts/serverguide/git.html.en)
2. Since we will use the 'catkin build' commmand, install catkin build tools:

```bash
sudo apt-get install python-catkin-tools
```

3. Create a catkin workspace, clone our repository and build. For this, in the folder of your preference:

```bash
mkdir -p ~/riseq_ws/src
cd ~/riseq_ws/src
git clone https://github.com/juanmed/riseq_uav .
cd ..
catkin build
echo 'source ~/riseq_ws/devel/setup.bash' >> ~/.bashrc
source ~/.bashrc
```
4. Installing required packages. We have a requirements.txt to make package installation easier. Go to the 'src' folder of the previously created workspace and:


```bash
pip2 install --user -r requirements.txt
```

Otherwise, install by yourself the following required python packages:
* numpy
* control
* slycot
* cvxopt

You will also need the [ZED Camera ROS Wrapper](https://www.stereolabs.com/docs/ros/#installation) so follow the installation instructions in that link.



## Testing 


### Install PX4 SITL (Software-In-The-Loop simulation)
Follow the instructions as shown in the [ROS with Gazebo Simulation PX4 Documentation](https://dev.px4.io/master/en/simulation/ros_interface.html)
To check if the necessary environment is setup correctly, you can run the gazebo SITL using the following command

```bash
cd <Firmware_directory>
DONT_RUN=1 make px4_sitl_default gazebo
```
To source the PX4 environment, run the following commands

```bash
cd <Firmware_directory>
source ~/riseq_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
```

You can run the rest of the roslaunch files in the same terminal

```bash
 roslaunch px4 posix_sitl.launch
```

You will need to source the PX4 environment in every new terminal you open to launch riseq_uav. 


#### On a real drone

This code repository is still very crude and runs the algorithms directly with no user interface for common actions like (start, stop, arm, disarm, trajectory settings, etc). For this reason running this code base in any real platform MUST FOLLOW ALL SAFETY CAUTIONS.

We use [Optitrack](https://optitrack.com/) for object tracking, [Jetson Nano](https://developer.nvidia.com/embedded/buy/jetson-nano-devkit) as onboard computer and a custom drone named FastQuad. To install various required software on Jetson Nano (like PCA9685 PWM driver, VRPN Client, etc) refer to [this repository](https://github.com/juanmed/nano_gpio).

Then, connect to Jetson Nano through SSH and launch:

```bash
ssh user@<jetson_nano_ip_address>
roslaunch riseq_common fastquad.launch
```





## Contact

Main contact: Juan Medrano (juanmedrano.ec09@gmail.com), Eugene Auh (egauh@naver.com ), Yonghee Park(qkrdydgml93@naver.com)



