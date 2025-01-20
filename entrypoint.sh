#!/usr/bin/bash
source /opt/ros/noetic/setup.bash
source /opt/barracuda-control/catkin_ws/devel/setup.bash

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source /opt/barracuda-simulation/catkin_ws/devel/setup.bash" >> ~/.bashrc


roslaunch barracuda_control start_thruster_manager.launch --wait