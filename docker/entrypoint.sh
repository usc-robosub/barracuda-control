# #!/usr/bin/bash
# source /opt/ros/humble/setup.bash
# source /opt/barracuda-control/dev_ws/install/setup.bash
# ros2 launch barracuda_control joystick_wrench_controller.launch.py

#!/bin/bash
set -e

echo "Installing dependencies..."
apt-get update

rosdep install --from-paths src --ignore-src -y

echo "Building ROS 2 workspace..."
colcon build --symlink-install

source install/setup.bash

echo "=========================================="
echo " Barracuda Control Workspace Ready! "
echo "=========================================="

exec "$@"