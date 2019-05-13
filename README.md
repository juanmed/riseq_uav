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





## Testing 

#### MIT FlightGoggles Simulator

We have developed our algorithms to work with the MIT's [flightgoggles simulator](https://github.com/mit-fast/FlightGoggles). Follow the steps in their [wiki](https://github.com/mit-fast/FlightGoggles/wiki) to install it. Then:

```bash
roslaunch riseq_common flightgoggles_control.launch
```

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



