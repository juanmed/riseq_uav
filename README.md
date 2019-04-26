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
2. Create a catkin workspace, clone our repository and build. For this, in the folder of your preference:

```bash
mkdir -p ~/riseq_ws/src
cd ~/riseq_ws/src
git clone https://github.com/juanmed/riseq_uav
cd ..
catkin build
echo 'source ~/riseq_ws/devel/setup.bash' >> ~/.bashrc
source ~/.bashrc
```
3. Installing required packages. We have a requirements.txt to make package installation easier. Go to the 'src' folder of the previously created workspace and:


```bash
pip2 install --user -r requirements.txt
```

Otherwise, install by yourself the following required python packages:
* numpy
* control
* slycot
* cvxopt





## Testing 


