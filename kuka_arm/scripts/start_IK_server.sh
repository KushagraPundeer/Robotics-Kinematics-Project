#!/bin/bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
rosrun kuka_arm IK_server.py

