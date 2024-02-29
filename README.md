# Stewart-Gough platform for aerial tooltip stabilisation

This repo contains the code needed to run and simulate a 6DoF parallel manipulator based on a Stewart platform with rotary actuators.

The implementation here is based on the work in https://github.com/daniel-s-ingram/stewart which demonstrates the use of .sdf files with ROS.

A .sdf file is used to describe the robot due to the ability to handle closed kinematic chains (and is easier to integrate with UAV models from the PX4-Autopilot stack). 

The inverse kinematics equations are taken from https://www.cambridge.org/core/journals/robotica/article/kinematic-and-dynamic-analysis-of-stewart-platformbased-machine-tool-structures/44227E02990F830098CB3897F3AED707 and adapted to rotary joints.

## Setup instructions
Clone this repo in your workspace and build:
```
cd ~/aam_ws/src
git clone https://github.com/lachie-aerialrobotics/stewart.git
catkin build
```
To test in gazebo, first update any changes to the .sdf:
```
cd gazebo/models/stewart_platform
erb model.sdf.erb > model.sdf
```
Then build the motor plugin:
```
cd gazebo/plugin
mkdir build
cd build
cmake ..
make
```
Add the following lines to `~/.bashrc`
```
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/aam_ws/src/stewart/gazebo/plugin/build
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/aam_ws/src/stewart/gazebo/models
```

Then run the simulation:
```
roslaunch stewart sim_manipulator.launch
```
Use dynamic reconfigure to send setpoints to the manipulator

## Dynamixel setup
Clone dynamixel workbench (can use my fork or the original repo)
```
cd ~/aam_ws/src
git clone https://github.com/lachie-aerialrobotics/dynamixel-workbench
git clone https://github.com/lachie-aerialrobotics/dynamixel-workbench-msgs
```
For fast servo comms, clone my branch of DynamixelSDK
```
git clone https://github.com/lachie-aerialrobotics/DynamixelSDK.git
cd DynamixelSDK/python
git checkout develop
sudo python3 setup.py install
```
To minimise usb delay,
```
nano /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```
and change value to 1. 

Alternatively:
```
sudo apt install setserial
sudo setserial /dev/ttyUSB0 low_latency
```