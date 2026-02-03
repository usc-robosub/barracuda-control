#!/usr/bin/bash
source /opt/ros/humble/setup.bash
source /opt/barracuda-control/catkin_ws/install/setup.bash

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /opt/barracuda-control/catkin_ws/install/setup.bash" >> ~/.bashrc

echo "Checking for robot_state_publisher..."
ros2 node list | grep robot_state_publisher
if [ $? -ne 0 ]; then
  echo "WARNING: robot_state_publisher is not running!"
  echo "The thruster_manager node requires robot_description parameter from robot_state_publisher"
  echo "Please start robot_state_publisher first or the thruster_manager will wait indefinitely"
fi

echo "Starting thruster manager node..."
# Launch thruster manager with output logging
ros2 run thruster_manager thruster_manager_node 2>&1 | tee /tmp/thruster_manager.log &
THRUSTER_PID=$!

# Keep container running and allow interactive commands
exec bash