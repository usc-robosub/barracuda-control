#!/usr/bin/bash
source /opt/ros/humble/setup.bash
source /opt/barracuda-control/dev_ws/install/setup.bash
ros2 launch barracuda_control joystick_wrench_controller.launch.py
# ros2 launch foxglove_bridge foxglove_bridge_launch.xml 
